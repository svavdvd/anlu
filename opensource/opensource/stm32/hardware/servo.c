#include "servo.h"
#include "TimPwm.h"
#include "Delay.h"
#include "OLED.h"
#include <math.h>
#include <stdio.h>

#define DEG2RAD  0.01745329252f   /* 度 -> 弧度 */
#define RAD2DEG  57.29577951f     /* 弧度 -> 度 */

/* 舵机物理角每度对应脉冲微秒数: (2500-500)/180 ≈ 11.11 us/° */
#define SERVO_US_PER_DEG  ((SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180.0f)

/* 当前各关节角(度), 供平滑运动插值和 r≈0 时保持 θ1 使用。
 * 初始化为复位姿态 {θ1=0, θ2=90, θ3=90, θ4=0}(大/小臂竖直向上, 夹爪水平朝前) */
static ArmAngles g_now = {HOME_THETA1, HOME_THETA2, HOME_THETA3, HOME_THETA4};

/* 夹爪当前脉冲(us), 供平滑开合插值使用。
 * 初始化为 Servo_Init 后 TIM3_CH1 输出的中位 1500us */
static uint16_t g_gripper_pulse = 1500;

/* ---------------------------------------------------------------------------
 * 舵机物理角(0~180°) -> 脉冲(us), 并限幅
 * -------------------------------------------------------------------------*/
static uint16_t Phys2Pulse(float phys_deg)
{
    if (phys_deg < 0.0f)   phys_deg = 0.0f;
    if (phys_deg > 180.0f) phys_deg = 180.0f;

    return (uint16_t)(SERVO_PULSE_MIN + phys_deg * SERVO_US_PER_DEG);
}

/* ---------------------------------------------------------------------------
 * 各舵机: 绝对关节角 -> 脉冲
 *
 * 标定依据(pulse=500 时):
 *   肩: 大臂水平朝 +X                -> θ2 = 0
 *   肘: 小臂垂直于大臂向下            -> θ3 - θ2 = -90
 *   腕: 夹爪垂直于小臂向上            -> θ4 - θ3 = +90
 * -------------------------------------------------------------------------*/

/* 大臂旋转(底座): 中位 1500 = 朝 +X, 未实测方向, 符号可能需要调整 */
static uint16_t BasePulse(float theta1)
{
    return Phys2Pulse(theta1 + 90.0f + SERVO_OFFSET_BASE);      /* θ1: -90(左) ~ +90(右) */
}

/* 大臂俯仰(肩): 舵机物理角 = 大臂与水平面绝对角 θ2 + 固定偏移 */
static uint16_t ShoulderPulse(float theta2)
{
    return Phys2Pulse(theta2 + SERVO_OFFSET_SHOULDER);   /* θ2: 0(水平朝+X) ~ 180(水平朝-X) */
}

/* 小臂俯仰(肘): 舵机装在大臂上, 量的是小臂相对大臂的角 θ3-θ2 + 固定偏移 */
static uint16_t ElbowPulse(float theta2, float theta3)
{
    return Phys2Pulse((theta3 - theta2) + 90.0f + SERVO_OFFSET_ELBOW);   /* 相对角 -90(下) ~ +90(上) */
}

/* 腕俯仰: 舵机装在小臂上, 量的是夹爪相对小臂的角 θ4-θ3, 方向与肘相反 */
static uint16_t WristPulse(float theta3, float theta4)
{
    return Phys2Pulse(90.0f - (theta4 - theta3) + SERVO_OFFSET_WRIST);
}

/* ---------------------------------------------------------------------------
 * 初始化舵机 PWM: TIM2 四路(PA0~PA3) + TIM3 一路(PA6 夹爪)
 * -------------------------------------------------------------------------*/
void Servo_Init(void)
{
    TIM2_Servo4CH_Init();
    TIM3_Servo1CH_Init();
}

/* ---------------------------------------------------------------------------
 * 由四个绝对关节角计算并输出全部舵机脉冲
 * -------------------------------------------------------------------------*/
void Servo_SetAll(const ArmAngles *angles)
{
    if (angles == 0)
        return;

    TIM2_SetCH1Pulse(BasePulse(angles->theta1));                  /* 大臂旋转 */
    TIM2_SetCH2Pulse(ShoulderPulse(angles->theta2));              /* 大臂俯仰 */
    TIM2_SetCH3Pulse(ElbowPulse(angles->theta2, angles->theta3)); /* 小臂俯仰 */
    TIM2_SetCH4Pulse(WristPulse(angles->theta3, angles->theta4)); /* 腕俯仰   */
}

/* ---------------------------------------------------------------------------
 * 设置夹爪(开合), 直接给定脉冲值
 * -------------------------------------------------------------------------*/
void Servo_SetGripper(uint16_t pulse)
{
    TIM3_SetCH1Pulse(pulse);
}

/* ---------------------------------------------------------------------------
 * 平滑开合夹爪: 与关节运动共用 SERVO_STEP_DEG / SERVO_STEP_MS。
 * 把脉冲差换算成舵机角(度), 按同样的步长与延时插值
 * -------------------------------------------------------------------------*/
void Servo_MoveGripper(uint16_t target)
{
    uint16_t start;
    float    diff_us, diff_deg;
    int      steps, i;
    float    t;

    start   = g_gripper_pulse;
    diff_us = (float)target - (float)start;
    if (diff_us < 0.0f) diff_us = -diff_us;
    diff_deg = diff_us / SERVO_US_PER_DEG;

    steps = (int)(diff_deg / SERVO_STEP_DEG) + 1;
    if (steps < 1) steps = 1;

    for (i = 1; i <= steps; i++)
    {
        t = (float)i / (float)steps;
        Servo_SetGripper((uint16_t)((float)start
                        + ((float)target - (float)start) * t + 0.5f));
        Delay_ms(SERVO_STEP_MS);
    }

    g_gripper_pulse = target;
}

/* ---------------------------------------------------------------------------
 * 逆运动学解算
 *   x, y, z : 末端目标坐标(mm), 坐标原点在底座旋转轴, Z 轴向上
 *   angles  : 输出 4 个关节角(度, 绝对角)
 * 返回: ARM_OK 或 ARM_ERR_RANGE(超出可达范围, 此时已截断到最近可达点)
 * -------------------------------------------------------------------------*/
int Arm_IK(float x, float y, float z, ArmAngles *angles)
{
    float r, r_w, z_w;
    float px, py, D, D_raw;
    float D_min;
    float phi, delta;
    float t4;
    int   ret = ARM_OK;

    if (angles == 0)
        return ARM_ERR_RANGE;

    /* 1. 底座旋转角 θ1, 目标到旋转轴的水平距离 r */
    r = sqrtf(x * x + y * y);
    if (r > 1e-3f)
    {
        angles->theta1 = atan2f(y, x) * RAD2DEG;
    }
    else
    {
        /* 目标在旋转轴正上方, θ1 无定义, 保持当前角度 */
        angles->theta1 = g_now.theta1;
    }

    /* 2. 末端姿态角 θ4(可配置, 通常竖直向下), 反推腕关节坐标 */
    t4 = ARM_TOOL_ANGLE * DEG2RAD;
    angles->theta4 = ARM_TOOL_ANGLE;

    r_w = r  - ARM_L3 * cosf(t4);   /* 腕关节水平坐标 */
    z_w = z  - ARM_L3 * sinf(t4);   /* 腕关节竖直坐标 */

    /* 3. 肩 -> 腕 的二连杆逆解 */
    px = r_w;               /* 肩到腕的水平距离 */
    py = z_w - ARM_H0;      /* 肩到腕的竖直距离 */
    D_raw = sqrtf(px * px + py * py);

    /* 可达范围检查: 肘舵机相对行程只有 ±90°, 故肩->腕距离 D 的下界是
     * 肘完全折叠(相对角 = ±90°)时的 √(L1²+L2²), 而不是 |L1-L2|
     * (后者需要相对角 ±180°, 舵机物理上做不到)。
     * 旧代码用 |L1-L2| 作下界, 会让 D 过小、肘相对角 >90° 的不可达目标被
     * 错误接受, 肘舵机饱和到极限, 末端跑到很远的地方。 */
    D_min = sqrtf(ARM_L1 * ARM_L1 + ARM_L2 * ARM_L2);

    if (D_raw > ARM_L1 + ARM_L2)
    {
        D = ARM_L1 + ARM_L2;
        ret = ARM_ERR_RANGE;
    }
    else if (D_raw < D_min)
    {
        D = D_min;
        ret = ARM_ERR_RANGE;
    }
    else
    {
        D = D_raw;
    }

    phi   = atan2f(py, px);                                   /* 肩->腕方向角  */
    delta = acosf((ARM_L1 * ARM_L1 + D * D - ARM_L2 * ARM_L2)
                  / (2.0f * ARM_L1 * D));                     /* 大臂与肩腕连线夹角 */

    /* 大臂绝对角: 肘在上(+1) 或 肘在下(-1) */
    angles->theta2 = (phi + ARM_ELBOW_SIGN * delta) * RAD2DEG;

    /* 小臂绝对角: 由肘关节坐标反推 */
    angles->theta3 = atan2f(py - ARM_L1 * sinf(angles->theta2 * DEG2RAD),
                            px - ARM_L1 * cosf(angles->theta2 * DEG2RAD)) * RAD2DEG;

    return ret;
}

/* ---------------------------------------------------------------------------
 * 正运动学: 由关节角求末端坐标(用于校验 IK 是否正确)
 * -------------------------------------------------------------------------*/
void Arm_Forward(const ArmAngles *angles, float *x, float *y, float *z)
{
    float t1, t2, t3, t4, r;

    if (angles == 0)
        return;

    t1 = angles->theta1 * DEG2RAD;
    t2 = angles->theta2 * DEG2RAD;
    t3 = angles->theta3 * DEG2RAD;
    t4 = angles->theta4 * DEG2RAD;

    r = ARM_L1 * cosf(t2) + ARM_L2 * cosf(t3) + ARM_L3 * cosf(t4);

    if (x) *x = r * cosf(t1);
    if (y) *y = r * sinf(t1);
    if (z) *z = ARM_H0 + ARM_L1 * sinf(t2) + ARM_L2 * sinf(t3) + ARM_L3 * sinf(t4);
}

/* ---------------------------------------------------------------------------
 * 平滑运动: 从当前角 g_now 步进插值到目标角, 避免舵机突跳
 * -------------------------------------------------------------------------*/
void Servo_MoveToAngles(const ArmAngles *target)
{
    ArmAngles cur = g_now;
    float maxdiff, d1, d2, d3, d4;
    int   steps, i;
    float t;

    if (target == 0)
        return;

    d1 = fabsf(target->theta1 - cur.theta1);
    d2 = fabsf(target->theta2 - cur.theta2);
    d3 = fabsf(target->theta3 - cur.theta3);
    d4 = fabsf(target->theta4 - cur.theta4);

    maxdiff = d1;
    if (d2 > maxdiff) maxdiff = d2;
    if (d3 > maxdiff) maxdiff = d3;
    if (d4 > maxdiff) maxdiff = d4;

    steps = (int)(maxdiff / SERVO_STEP_DEG) + 1;
    if (steps < 1) steps = 1;

    for (i = 1; i <= steps; i++)
    {
        t = (float)i / (float)steps;
        ArmAngles a;
        a.theta1 = cur.theta1 + (target->theta1 - cur.theta1) * t;
        a.theta2 = cur.theta2 + (target->theta2 - cur.theta2) * t;
        a.theta3 = cur.theta3 + (target->theta3 - cur.theta3) * t;
        a.theta4 = cur.theta4 + (target->theta4 - cur.theta4) * t;

        Servo_SetAll(&a);
        Delay_ms(SERVO_STEP_MS);
    }

    g_now = *target;
}

/* ---------------------------------------------------------------------------
 * 逆解 + 平滑运动到末端目标点
 * -------------------------------------------------------------------------*/
void Arm_MoveTo(float x, float y, float z)
{
    ArmAngles a;
    Arm_IK(x, y, z, &a);
    Servo_MoveToAngles(&a);
}

/* ---------------------------------------------------------------------------
 * 复位: 平滑运动到复位姿态(机械臂竖直伸展), 夹爪张开
 * -------------------------------------------------------------------------*/
void Servo_Reset(void)
{
    ArmAngles home;

    Servo_MoveGripper(GRIPPER_OPEN_PULSE);

    home.theta1 = HOME_THETA1;
    home.theta2 = HOME_THETA2;
    home.theta3 = HOME_THETA3;
    home.theta4 = HOME_THETA4;

    Servo_MoveToAngles(&home);
}

/* ---------------------------------------------------------------------------
 * 抓取/放置任务:
 *   抓取点上方 -> 下降抓取 -> 夹爪闭合 -> 抬升
 *   -> 放置点上方 -> 下降放置 -> 夹爪张开 -> 抬升 -> 复位
 * -------------------------------------------------------------------------*/
void Task_PickAndPlace(float px, float py, float pz,
                       float qx, float qy, float qz)
{
    float lift = ARM_LIFT_MM;

    Arm_MoveTo(px, py, pz + lift);          /* 移到抓取点上方 */
    Arm_MoveTo(px, py, pz);                 /* 下降到抓取点   */
    Servo_MoveGripper(GRIPPER_CLOSE_PULSE); /* 夹爪闭合       */
    Delay_ms(500);
    Arm_MoveTo(px, py, pz + lift);          /* 抬升           */

    Arm_MoveTo(qx, qy, qz + lift);          /* 移到放置点上方 */
    Arm_MoveTo(qx, qy, qz);                 /* 下降到放置点   */
    Servo_MoveGripper(GRIPPER_OPEN_PULSE);  /* 夹爪张开       */
    Delay_ms(500);
    Arm_MoveTo(qx, qy, qz + lift);          /* 抬升           */

    Servo_Reset();                          /* 回到复位姿态   */
}
