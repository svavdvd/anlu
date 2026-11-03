#include "stm32f10x.h"   // Device header
#include "Delay.h"
#include "OLED.h"
#include "servo.h"
#include "Usart.h"
#include <stdio.h>

/* ===================== 像素坐标 -> 机械臂实际坐标(mm) =====================
 * 摄像头固定俯视工作区(方形像素近似), 摄像头在机械臂坐标系 (CAM_X_MM, CAM_Y_MM)。
 *
 * FPGA 侧已把目标质心换算成"屏幕底边中点为原点"的像素坐标:
 *   px : 向右为正, 范围约 [-640, +640]
 *   py : 向上为正, 范围约 [0, 720]
 *
 * 标定依据:
 *   - 屏幕底边落到实际区域的长度为 BOTTOM_EDGE_MM(431mm);
 *   - 摄像头在机械臂坐标系 Y 轴上, y = CAM_Y_MM(450mm);
 *   - 方形像素: X/Y 比例相同, MM_PER_PX = 431 / 1280 ≈ 0.3367 mm/px。
 *
 * 换算公式(图像中心正对摄像头投影点):
 *   x_arm = CAM_X_MM + px * MM_PER_PX
 *   y_arm = CAM_Y_MM + (py - IMG_H/2) * MM_PER_PX
 * 即: 屏幕底边中点对应机械臂 (0, 450 - 360*MM_PER_PX) ≈ (0, 328.8)mm。
 *
 * 方向修正: 若实测机械臂运动方向与图像相反(镜像/安装朝向), 把 DIR_X / DIR_Y
 * 改成 -1.0f 即可, 无需改公式。
 * =====================================================================*/
#define IMG_W           1280.0f   /* 图像宽(像素) */
#define IMG_H           720.0f    /* 图像高(像素) */
#define BOTTOM_EDGE_MM  431.0f    /* 屏幕底边对应实际宽度(mm) */
#define CAM_X_MM        0.0f      /* 摄像头在机械臂坐标系 X(mm), 图像中心正下方 */
#define CAM_Y_MM        450.0f    /* 摄像头在机械臂坐标系 Y(mm), 图像中心正下方 */
#define DIR_X           (1.0f)    /* X 方向: +1 同向, -1 镜像 */
#define DIR_Y           (1.0f)    /* Y 方向: +1 同向, -1 镜像 */
#define MM_PER_PX       (BOTTOM_EDGE_MM / IMG_W)   /* ≈0.3367 mm/px */

/* 把 FPGA 传来的像素坐标(底边中点原点)转为机械臂实际坐标(mm) */
static void PixelToArm(float px, float py, float *ax, float *ay)
{
    *ax = CAM_X_MM + DIR_X * px * MM_PER_PX;
    *ay = CAM_Y_MM + DIR_Y * (py - IMG_H * 0.5f) * MM_PER_PX;
}

/* ===================== 分拣任务配置 (按实际修改) ===================== */
#define PICK_Z   30.0f    /* 抓取高度(mm), 物块所在桌面高度 */
#define DROP_X    0.0f    /* 放置点 x(mm), 固定 */
#define DROP_Y  -150.0f   /* 放置点 y(mm), 固定 */
#define DROP_Z   30.0f    /* 放置高度(mm) */

/* 分类字符 -> OLED 显示行号: R=2, G=3, B=4 */
static uint8_t LineOf(char type)
{
    switch (type)
    {
        case 'R': return 2;
        case 'G': return 3;
        case 'B': return 4;
        default:  return 0;
    }
}

/* 在 OLED 上显示收到的像素坐标 + 颜色。
 * 每收到一条命令刷新对应颜色那一行; 下一次按 KEY4 发来新批次时自然覆盖刷新。 */
static void ShowTarget(char type, float px, float py)
{
    uint8_t line = LineOf(type);
    char    buf[17];

    if (line == 0)
        return;

    OLED_ShowString(line, 1, "                ");          /* 先清空该行, 避免残留 */
    snprintf(buf, sizeof(buf), "%c:%+d,%d", type, (int)px, (int)py);
    OLED_ShowString(line, 1, buf);
}

int main(void)
{
    char  line[64];
    float px, py, ax, ay;
    char  type;

    OLED_Init();
    OLED_ShowString(1, 1, "Arm Init");

    Servo_Init();                       // 初始化 5 路舵机 PWM
    Delay_ms(500);
    Servo_Reset();                      // 复位到竖直姿态, 夹爪张开

    Usart1_Init(115200);                // 初始化串口1
    Usart1_SendString("robot arm ready\r\n");

    OLED_ShowString(1, 1, "Arm Ready");
    OLED_ShowString(2, 1, "R:              ");
    OLED_ShowString(3, 1, "G:              ");
    OLED_ShowString(4, 1, "B:              ");

    while (1)
    {
        /* 收到完整一行才处理; 运动期间新命令会被丢弃, 请等 "done" 再发下一条 */
        if (Usart1_ReadLine(line, sizeof(line)))
        {
            /* 命令格式: px,py,分类   例: -123,456,R
             * px/py 为 FPGA 传来的像素坐标(原点=屏幕底边中点)。 */
            if (sscanf(line, "%f,%f, %c", &px, &py, &type) == 3)
            {
                ShowTarget(type, px, py);               /* OLED 显示收到的坐标+颜色 */

                PixelToArm(px, py, &ax, &ay);           /* 像素 -> 机械臂实际坐标 */

                Task_PickAndPlace(ax, ay, PICK_Z,
                                  DROP_X, DROP_Y, DROP_Z);   /* 统一搬到 (0,-150,30) */

                Usart1_SendString("done\r\n");          /* 本条完成, 通知 FPGA 发下一条 */
            }
            else
            {
                Usart1_SendString("ERR bad format\r\n");
            }
        }
    }
}
