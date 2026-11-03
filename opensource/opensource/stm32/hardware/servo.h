#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* ============================================================================
 * 四轴机械臂(5 舵机) 控制: 逆运动学 + 平滑运动 + 抓取/分拣任务
 *
 * 关节: 1-大臂旋转  2-大臂俯仰  3-小臂俯仰  4-腕俯仰  5-夹爪
 *
 * 几何模型(单位: mm), 坐标系: 原点在底座旋转轴上, Z 轴竖直向上,
 * X/Y 为水平面, 大臂/小臂/腕在同一竖直平面内运动(该平面由 θ1 决定):
 *
 *   Z(上)
 *   ^       大臂(L1)       小臂(L2)      腕(L3)
 *   |      肩 ────→ 肘 ────→ 腕 ────→ 末端(夹爪)
 *   |      / θ2         \ θ3          \ θ4
 *   |     /               \              \
 *   |    /  H0             \              \
 *   +───────────→ R(水平, 指向目标)
 *   底座旋转轴(θ1, 绕 Z 轴)
 *
 * 末端坐标:
 *   r = L1·cosθ2 + L2·cosθ3 + L3·cosθ4
 *   x = r·cosθ1,  y = r·sinθ1
 *   z = H0 + L1·sinθ2 + L2·sinθ3 + L3·sinθ4
 *
 * θ2/θ3/θ4 为各连杆与"水平面"的绝对夹角(度, 向上为正),
 * θ4(末端姿态角)由 ARM_TOOL_ANGLE 指定, 默认 -90° 表示夹爪竖直向下。
 * ==========================================================================*/

/* ===================== 机械臂几何参数 (单位: mm) ===================== */
#define ARM_L1     104.0f   /* 大臂长度: 肩关节中心 -> 肘关节中心        */
#define ARM_L2     147.0f   /* 小臂长度: 肘关节中心 -> 腕关节中心        */
#define ARM_L3     180.0f   /* 腕部长度: 腕关节中心 -> 末端(夹爪尖端)    */
#define ARM_H0     113.0f   /* 底座高度: 旋转轴 -> 肩关节中心的垂直距离  */

/* 末端姿态角(度, 与水平面绝对夹角): -90=竖直向下, 0=水平向前
 * 注意: 夹爪竖直向下(-90)仅在小臂水平或下垂(θ3<=0)时精确可达,
 *       否则腕舵机(相对小臂仅 ±90°)会限幅, 夹爪略偏垂直, 属硬件行程限制 */
#define ARM_TOOL_ANGLE   (-90.0f)

/* 肘关节构型: +1 = 肘在上(默认, 常见), -1 = 肘在下 */
#define ARM_ELBOW_SIGN   (1.0f)

/* 抓取/放置时的抬高量(接近/离开时抬升高度, mm) */
#define ARM_LIFT_MM      50.0f

/* ===================== 舵机脉冲范围(us) ===================== */
#define SERVO_PULSE_MIN  500     /* 对应舵机物理角 0°   */
#define SERVO_PULSE_MAX  2500    /* 对应舵机物理角 180° */

/* ===================== 零点偏移与重力补偿 (度) =====================
 * OFFSET = 命令角 − 实际角(需补正的量), 直接加到命令角上。
 *
 * 底座/腕: 底座绕竖直轴转无重力力矩; 腕姿态固定(θ4=-90)下垂近似常数,
 *          故只用常数零点偏移。 */
#define SERVO_OFFSET_BASE     0.0f    /* 大臂旋转 */
#define SERVO_OFFSET_WRIST    -4.0f  /* 腕俯仰   */

/* 固定偏移方案: 每个关节一个常数偏移(度), 让死区下界对齐目标角。
 * 标定: 命令某姿态 -> 量实际角 -> 偏移 = 命令 - 实际。
 * 注意: 肩/肘的下垂随姿态(cosθ)变化, 单一常数只能对准一个姿态;
 *       若不同姿态需要的偏移相差几度, 说明要回到 cos 补偿模型。 */
#define SERVO_OFFSET_SHOULDER 5.0f    /* 大臂俯仰(肩) 固定偏移 */
#define SERVO_OFFSET_ELBOW    5.3f    /* 小臂俯仰(肘) 固定偏移 */

/* 夹爪(第5个舵机, TIM3_CH1/PA6) 开合脉冲, 按实际标定修改 */
#define GRIPPER_OPEN_PULSE    800    /* 夹爪张开 */
#define GRIPPER_CLOSE_PULSE   1400   /* 夹爪闭合 */

/* ===================== 平滑运动参数 =====================
 * 运动速度 = SERVO_STEP_DEG / SERVO_STEP_MS (度/ms)。
 * 当前 3° / 40ms ≈ 75°/s。想更慢: 减小 SERVO_STEP_DEG 或增大 SERVO_STEP_MS。 */
#define SERVO_STEP_DEG    0.3f    /* 每步最大角度(度), 越小越平滑 */
#define SERVO_STEP_MS     20      /* 每步延时(ms)                */

/* ===================== 舵机通道映射 (TIM2/TIM3) =====================
 *   CH1(PA0)=大臂旋转  CH2(PA1)=大臂俯仰  CH3(PA2)=小臂俯仰  CH4(PA3)=腕俯仰
 *   TIM3_CH1(PA6)=夹爪
 *
 * 舵机标定(pulse=500 时的姿态):
 *   大臂俯仰: 大臂水平朝 +X             -> 物理角 = θ2
 *   小臂俯仰: 小臂垂直于大臂向下          -> 物理角 = (θ3-θ2) + 90
 *   腕 俯 仰: 夹爪垂直于小臂向上          -> 物理角 = 90 - (θ4-θ3)
 *   大臂旋转: 未标定, 默认中位(1500)=朝+X -> 物理角 = θ1 + 90  (需实测符号)
 * ==================================================================*/

/* ===================== 复位姿态 =====================
 * 大臂竖直向上、小臂竖直向上(均垂直于桌面), 夹爪水平朝前(平行于桌面)。
 * 注意: θ4=0 时腕舵机物理角约 176°(接近 180° 上限), 属腕相对行程 ±90° 的极限 */
#define HOME_THETA1  0.0f
#define HOME_THETA2  90.0f
#define HOME_THETA3  90.0f
#define HOME_THETA4  0.0f

/* 解算返回值 */
#define ARM_OK            0   /* 解算成功              */
#define ARM_ERR_RANGE    -1   /* 目标超出可达范围(已截断)*/

/* 解算结果: 四个关节角(度, 绝对角) */
typedef struct
{
    float theta1;   /* 大臂旋转角 */
    float theta2;   /* 大臂俯仰角 */
    float theta3;   /* 小臂俯仰角 */
    float theta4;   /* 腕俯仰角   */
} ArmAngles;

/* 初始化: 使能 TIM2 四路(PA0~PA3) + TIM3 一路(PA6 夹爪) PWM */
void Servo_Init(void);

/* 由四个绝对关节角计算并输出全部舵机脉冲(内部处理肘/腕的相对安装关系) */
void Servo_SetAll(const ArmAngles *angles);

/* 设置夹爪(开合), 直接给定脉冲值 */
void Servo_SetGripper(uint16_t pulse);

/* 平滑开合夹爪: 与关节运动共用 SERVO_STEP_DEG / SERVO_STEP_MS 参数 */
void Servo_MoveGripper(uint16_t target);

/* 逆运动学: 由末端目标坐标(x,y,z)解算 4 个关节角 */
int Arm_IK(float x, float y, float z, ArmAngles *angles);

/* 正运动学(校验用): 由关节角求末端坐标 */
void Arm_Forward(const ArmAngles *angles, float *x, float *y, float *z);

/* 平滑运动到指定关节角(内部做步进插值, 避免突跳) */
void Servo_MoveToAngles(const ArmAngles *target);

/* 逆解 + 平滑运动到末端目标点 */
void Arm_MoveTo(float x, float y, float z);

/* 复位: 平滑运动到复位姿态, 夹爪张开 */
void Servo_Reset(void);

/* 抓取/放置任务: 从抓取点(p)抓取 -> 抬升 -> 移到放置点(q) -> 放下 -> 复位 */
void Task_PickAndPlace(float px, float py, float pz,
                       float qx, float qy, float qz);

#endif /* __SERVO_H */
