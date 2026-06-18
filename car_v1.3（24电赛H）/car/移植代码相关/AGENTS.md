# AGENTS.md — 2024 电赛 H 题 自动行驶小车

## 项目概览

- **MCU**: STM32F103C8T6 (Cortex-M3, 64K Flash, 20K SRAM)
- **IDE**: Keil MDK v5 + ARMCC (V5.06)
- **HAL**: 标准外设库 (SPL, `Library/`)，**非** HAL 库
- **编码**: GB2312/GBK 中文注释

## 目录结构

| 目录 | 内容 |
|------|------|
| `Start/` | 启动文件 (`startup_stm32f10x_md.s`)、`system_stm32f10x.c`、`core_cm3.c` |
| `System/` | `Delay.c/h` (SysTick 延时)、`sys.c/h` |
| `Library/` | STM32F10x SPL（不改动） |
| `Hardware/` | 外设驱动，见下方 |
| `User/` | `main.c`、`stm32f10x_conf.h`、`stm32f10x_it.c/h` |

## 硬件驱动 (`Hardware/`)

| 模块 | 文件 | 功能 |
|------|------|------|
| 电机 | `Motor.c/h`, `PWM.c/h` | TIM2 CH2(右)/CH3(左) PWM, PA3/4/5/0 方向控制，速度范围 ±50 |
| 循迹 | `TRACK.c/h` | 8 路红外 (PA15/8/11/7/6 + PB15/14/13)，返回 8 位状态字节 |
| 控制 | `Cnotorl.c/h` | PID 循迹 (`Kp=2.5, Kd=3.5`) + 角度 PD (`ka2p=0.5, ka2d=3.5`) |
| 陀螺仪 | `mpu6050.c/h`, `MPU6050_I2C.c/h`, `inv_mpu.c/h` | I2C+DMP，获取 Pitch/Roll/Yaw |
| OLED | `OLED.c/h`, `OLED_Font.h` | 软件 I2C (PB10 SCL, PB11 SDA) |
| 按键 | `Key.c/h` | PB3(Key2), PB4(Key1)，消抖 20ms |
| 其他 | `LED.c/h`, `buzzer.c/h`, `Timer.c/h`, `uart.c/h` | |

## 代码约定

1. **命名模式**: `ModuleName_Init()`, 引脚用 `#define` 宏在头文件中定义
2. **Delay 函数**: 驼峰 `Delay_ms()` + 小写别名 `delay_ms()`，两者等价
3. **主循环**: 状态机驱动 (`all_state`)，按键 1 切换赛题步骤，按键 2 设 bit4 启动
4. **PID 输出**: 位置式 PD，`Final_Speed(base, pid_out)` 计算左右轮速度差
5. **角度控制**: `pid_angle2(target, yaw)` 处理 0~360° 环绕，输出叠加到电机 PWM

## 编译/烧录

- 打开 `Project.uvprojx` → Keil MDK → Build (F7)
- 使用 ST-Link / J-Link 烧录
- 调试配置: `DebugConfig/Target_1_STM32F103C8_1.0.0.dbgconf`

## 注意事项

- 中文注释在非 GBK 环境下可能乱码，模型回答请用简体中文
- ARMCC 是 C89/C90 风格编译器，变量声明必须在块开头
- `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 释放了 PA15/JTDI 用作普通 IO
- MPU6050 DMP `MPU6050_DMP_Get_Data()` 返回非零时需循环重试（I2C 时序敏感）

## 可用技能 (Skills)

在 Chat 中输入 `/` 可以调用以下专项技能：

| 技能命令 | 说明 |
|---------|------|
| `/stm32-line-tracking` | 红外循迹 PD 控制专项（8路传感器 → PID → 差速） |
| `/stm32-mpu6050` | MPU6050 陀螺仪 DMP 姿态解算专项（I2C → DMP → 欧拉角） |
| `/stm32-project-overview` | 项目全景（架构、状态机、引脚表、四道赛题） |

## 可用智能体 (Agents)

| 智能体 | 说明 |
|--------|------|
| `stm32-port-assistant` | STM32 代码移植助手：帮你在新硬件平台上复刻本项目代码 |

## 可用提示词 (Prompts)

| 提示词 | 说明 |
|--------|------|
| `/移植电赛巡线代码` | 生成给另一个 Copilot 的结构化提示词，配合知识文档完成跨项目代码移植 |

## 知识文档

以下 MD 文件可供 AI 阅读以深入理解各模块：

| 文档 | 内容 |
|------|------|
| `项目全景说明书.md` | 目录分层、引脚全表、四道赛题状态机、两套 PD 控制 |
| `循线控制逻辑详解.md` | 五层调用链、查表映射、PD 算法、调参指南 |
| `MPU6050陀螺仪逻辑详解.md` | 四层文件架构、DMP 初始化、四元数转欧拉角、角度 PD |
- MPU6050 DMP 初始化有时失败（I2C 时序敏感），`main()` 中持续重试
