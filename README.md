# 安陆二 · 摄像头采集与 DSI 显示工程

摄像头图像采集 → FPGA 内部图像信号处理（ISP）→ MIPI DSI 送屏显示的 FPGA 工程，
配套 STM32 固件用于摄像头/屏幕上电时序与参数配置。

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `V1.7代码/td_project/` | 紫光同创 PDS 工程 `camera_to_dsi_display.al` 与时序约束 `camera_to_dsi_display.sdc` |
| `V1.7代码/user_source/hdl_source/` | 自研逻辑：`mipi_dphy_rx`、`isp`（BLC / demosaic / AWB）、`dsi_tx_packet`、`vtc`、`uics500_cfg`、`uial2axis`、`ph1p35_ddr`、`uart` |
| `V1.7代码/user_source/ip_source/` | IP 核源码（含除法器、时钟等） |
| `V1.7代码/user_source/constraints_source/` | 引脚约束（`pin.adc`） |
| `V1.7代码/FPGAarmrun/` | Keil MDK（STM32）工程：`project.uvprojx`，含 `hardware`、`library`、`start`、`system`、`user`、`tools` |
| `opensource/opensource/` | 参考代码（`fpga/rtl`、`fpga/constraints`、`stm32`），带独立 LICENSE |

## 图像处理链路

```text
CMOS 传感器 --MIPI CSI-2--> mipi_dphy_rx --> uial2axis
      --> ISP: BLC(黑电平) -> demosaic(去马赛克) -> AWB(自动白平衡) 及数据位宽转换
      --> dsi_tx_packet --> MIPI DSI --> 屏幕
                        vtc 提供视频时序，uics500_cfg 负责传感器寄存器配置
```

## 开发环境

- **FPGA**：紫光同创 Pango Design Suite（PDS），工程文件 `camera_to_dsi_display.al`
- **MCU**：Keil MDK，工程 `FPGAarmrun/project.uvprojx`（STM32F10x 库）
- 约束文件：`camera_to_dsi_display.sdc`（时序）+ `constraints_source/pin.adc`（物理引脚）

## 未纳入版本管理

综合与布局布线产生的日志（`*.log`、`*.logw`、`*.timingsummary`，约 8 MB）、
Keil 编译中间件（`Objects/`、`Listings/`）以及工程备份文件（`*.bak_*`）已通过
`.gitignore` 排除，可由工程重新生成。

## 许可证

本仓库中 `V1.7代码/` 目录下的自有代码采用 **MIT License**，详见根目录 [LICENSE](LICENSE)。

`opensource/opensource/` 目录为第三方参考代码，遵循其自带的 **MIT License**
（见 `opensource/opensource/LICENSE`，版权归原作者所有），本仓库仅作整理与引用，
不改变其原始授权。