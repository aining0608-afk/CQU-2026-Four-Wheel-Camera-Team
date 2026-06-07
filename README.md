# 重庆大学26年四轮摄像头组

本项目是基于逐飞科技 TC264 开源库、英飞凌 AURIX TC264D 和 TASKING
工具链的四轮摄像头智能车程序。工程由 mini 组程序迁移而来，保留了巡线、
模糊控制、速度环、编码器、IMU、IPS200 显示和 MT9V03X 摄像头等功能。

## 重要安全提示

**当前代码处于电机测试状态，不能直接落地上电。**

- `user/cpu0_main.c` 中当前设置为 `CarInfo.Mode = MOTORTEST`。
- `user/isr.c` 中已经绕过 P33.13 拨码开关，系统上电后自动计时。
- `code/motor.c` 中 `system_ms > 3000` 后，两侧电机会固定输出 `-820`。

因此，当前固件上电约 3 秒后车轮会自动转动。首次烧录和调试时必须架空车轮，
或断开电机动力电源，并确保急停方式可用。

## 硬件与开发环境

| 项目 | 配置 |
| --- | --- |
| MCU | Infineon AURIX TC264D / TC26xD B-Step |
| 主板 | 逐飞科技 TC264 & 3xx 主板 V2.5 |
| 摄像头 | MT9V03X |
| IMU | IMU660RA |
| 屏幕 | IPS200 SPI |
| IDE | AURIX Development Studio，原工程环境为 ADS v1.9.20 |
| 编译器 | TASKING TriCore |
| 下载调试 | DAS / UDAS，JTAG0 |

## 目录结构

```text
.
├─ code/          车辆业务代码：巡线、控制、编码器、电机、UI、IMU
├─ user/          CPU0/CPU1 入口、中断配置
├─ libraries/     逐飞驱动、设备组件和 Infineon iLLD
├─ docs/          引脚和硬件接线说明
├─ .project       ADS/Eclipse 工程入口
├─ .cproject      TASKING 编译与链接配置
└─ Lcf_Tasking_Tricore_Tc.lsl
```

`Debug/`、`Release/`、临时备份和本机调试配置均为可再生成文件，不提交到 Git。

## 编译与烧录流程

1. 安装 AURIX Development Studio，并确认 TASKING TriCore 编译器、DAS/UDAS
   驱动可用。
2. 在 ADS 中选择 `File > Import > Existing Projects into Workspace`。
3. 选择本项目根目录，导入名为 `Seekfree_TC264_Opensource_Library` 的工程。
4. 选择 `Debug` 配置并执行 `Build Project`。
5. 编译产物会生成到 `Debug/`，主要文件为：
   - `Seekfree_TC264_Opensource_Library.elf`
   - `Seekfree_TC264_Opensource_Library.hex`
6. 连接 TC264 主板与调试器，在 ADS 中新建或更新调试配置：
   - Target：`Generic Infineon AURIX Board`
   - Configuration：`TC26x`
   - Interface：`DAS / UDAS`
   - Port：`JTAG0`
7. 烧录前架空车轮，确认供电、方向和急停条件，再下载并运行。

## 程序运行流程

### CPU0

`user/cpu0_main.c` 中的 `core0_main()` 负责主初始化和低频任务：

1. 初始化时钟、调试串口、IPS200、电机与舵机。
2. 初始化 IMU、编码器、摄像头、电压采集、蜂鸣器、按键和滤波器。
3. 初始化 PID 与 PIT 周期中断。
4. 等待双核同步后进入主循环。
5. 主循环执行 LED 心跳、UI/图像显示，以及可选的图传、示波器和远程调参。

### CPU1

`user/cpu1_main.c` 中的 `core1_main()` 等待摄像头完成一帧采集，然后执行
`findline()` 提取赛道边线和中线，再由 `caculate_err()` 计算巡线偏差。

### 周期中断

`user/isr.c` 中的 PIT 中断负责实时控制：

| 周期 | 主要任务 |
| --- | --- |
| 1 ms | 任务计时、蜂鸣器；双环开启时执行电流环 |
| 2 ms | 姿态计算、模糊舵机控制 |
| 5 ms | 编码器测速、左右速度环、电机输出 |
| 100 ms | 按键扫描 |

## 从电机测试切换到正常巡线

不要只修改模式后直接落地运行。应按以下顺序操作：

1. 架空车轮，确认左右电机方向和编码器方向一致。
2. 恢复 `user/isr.c` 中 P33.13 运行使能门，避免上电自动输出。
3. 完成舵机中位、机械极限、IMU 偏置和编码器系数标定。
4. 将 `user/cpu0_main.c` 中的 `CarInfo.Mode` 从 `MOTORTEST` 改为 `STAND`。
5. 使用最低速度进行离地测试，再落地低速测试。
6. 每次只调整一组参数，确认稳定后再提高速度。

## 上电前检查清单

- 车轮已架空，或电机动力电源已断开。
- 左右电机 PWM、方向引脚和实际车轮对应关系正确。
- 舵机不会碰到机械限位，`pout0` 与 `delt_pout_max` 已标定。
- 编码器计数方向、每米计数和里程换算系数已标定。
- IMU 安装方向和静态偏置已标定。
- 电流采样零偏和电池电压采集正常。
- 摄像头图像方向、曝光和边线识别正常。
- P33.13 运行使能或其他可靠急停方式已生效。

## 需要标定的关键参数

代码中的 `TODO[4WHEEL-CAL]` 和 `TODO[4WHEEL-VERIFY]` 标出了主要检查项：

| 文件 | 参数/内容 |
| --- | --- |
| `code/control.c` | `speed_diff`、`pout0`、`delt_pout_max`、`Kp_max`、`Kd_max` |
| `code/encode.h` | `L_QD_UNIT`、`R_QD_UNIT` |
| `code/encode.c` | 里程换算系数 |
| `code/gyroscope.c` | IMU 安装偏置 |
| `code/motor.h` | 电机引脚、电流采样零偏 |
| `code/findline.c` | `qianzhan` 与三种速度参数 |

PowerShell 自检命令：

```powershell
rg -n "TODO\[4WHEEL-CAL\]|TODO\[4WHEEL-VERIFY\]" code user
rg -n "CarInfo\.Mode|MOTORTEST|AUTO-START|P33_13" code user
```

## 当前主要引脚

| 功能 | 引脚 |
| --- | --- |
| 左电机 PWM / DIR | P02.5 / P02.4 |
| 右电机 PWM / DIR | P02.7 / P02.6 |
| 编码器 1 计数 / 方向 | P20.3 / P20.0 |
| 编码器 2 计数 / 方向 | P10.3 / P10.1 |
| 舵机 PWM | P33.9 |
| 蜂鸣器 | P33.10 |
| UI 拨码 / 运行使能 | P33.12 / P33.13 |
| 摄像头 VSYNC | P02.0 |
| 摄像头 D0-D7 | P00.0-P00.7 |

最终接线必须以实际主板原理图和万用表测量结果为准。

## 开发注意事项

- 工程默认文本编码为 GBK，`user/cpu0_main.c` 单独配置为 UTF-8。不要批量转换
  源码编码，否则中文注释可能损坏。
- 新增业务代码直接放在 `code/`，并在 ADS 工程中确认文件已参与编译。
- 不要用 `.bak` 文件保存历史版本，修改记录交给 Git。
- 不提交 `Debug/`、`Release/`、`.elf`、`.hex`、`.o`、`.d` 等构建产物。
- 修改引脚、PWM 极性、舵机范围或控制模式后，必须先做离地测试。
- 逐飞库的许可证与版本信息位于 `libraries/doc/`。
