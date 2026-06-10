# AGENTS.md — car_v1.2 智能小车巡线控制系统

## 项目概述

基于 STM32F103C8T6 的智能小车巡线系统，具备 **编码器速度闭环 PID**、MPU6050 姿态感知、HMC5883L 磁力计、SSD1306 OLED 显示。

**关键硬件**: TB6612 电机驱动 ×2、编码器 ×2、8 路灰度传感器（亚博智能）、MPU6050、HMC5883L。

## 项目结构

```
car_v1.2/
├── code/                    # 应用层（巡线、PID、电机、滤波）
├── ml_libs/                 # 底层驱动库（GPIO/PWM/TIM/UART/I2C/OLED 等）
│   └── headfile.h           #   **统一头文件**，所有 .c 都 include 它
├── menu/                    # OLED 菜单系统（⚠️ 源文件缺失，仅 .o 残留）
├── skills/                  # AI Agent 调试方法论 → [skills/SKILL.md](skills/SKILL.md)
├── sys/                     # CMSIS 系统文件
├── user/                    # 入口（main.c, isr.c）
└── README.md                # 完整引脚表
```

## 构建系统

- **Keil MDK**，工程文件 `user/Project.uvprojx`
- Include 路径: `..\user;..\ml_libs;..\code;..\sys`（平铺式，头文件直接 include 文件名）
- **编译器**: ARM Compiler 5 (`__CC_ARM`)，预定义宏 `STM32F10X_MD`

## 开发工作流（AI 代理须知）

> ⚠️ **AI 代理无法编译或烧录代码**。所有代码由用户在本机 Keil MDK 中编译和烧录。

### 修改代码后的标准流程
1. 修改完成后，**明确告知用户需要做什么**（编译？烧录？测试某个功能？）
2. 如果新增了 `.c` 文件 → 提醒用户加入 Keil 工程（右键 Add Existing Files）
3. 如果修改了 `headfile.h` 的 include → 提醒用户检查 Keil include 路径
4. 改完立即让用户编译验证，不要攒一堆改动一起编译

### 新增文件时的完整清单
参考仓库记忆 [checklist](memories/repo/checklist.md)：
- 新 `.c` → 加入 Keil 工程 + 在 `headfile.h` 加 `#include "xxx.h"`
- 新全局变量 → `.c` 定义 + `.h` 加 `extern` 声明
- 废弃 `.c` → 从 Keil 工程移除，否则链接报错

### 串口独占规则
`printf()` 和 `datavision_send()` 共享 USART1，**不可同时开启**。调试时二选一：
- 看文本输出 → 用 `printf()`，注释掉 `datavision_send()`
- 看波形图 → 用 `datavision_send()`，注释掉 `printf()`

## ⚠️ 文件编码警告

**本项目中文源文件使用 GB2312 编码**。在 VS Code 中打开含中文的文件（注释、字模数据、OLED 字体）时，UTF-8 下中文会显示为乱码。

- ❌ **禁止用 VS Code 编辑含 `>0x7F` 字节的文件**（如 `oled_font.c`、字模数据），会破坏 GB2312 编码导致 Keil 编译报错 `invalid multibyte character sequence`
- ✅ 修改此类文件时，用脚本处理（转换编码或 hex 编辑）
- ✅ 新增代码中的显示字符串用英文，注释可用中文（确保 GB2312 兼容）

## 编码规范

### 头文件与依赖
- **`headfile.h`** 是所有模块的统一头文件（相当于 AllHeader.h），包含所有标准库和项目头文件
- 每个 .c 文件开头只需 `#include "headfile.h"`
- 头文件保护: `__filename_h_`（双下划线+小写+`_h_`）如 `__gray_track_h_`，或 `_filename_h`（如 `_ml_gpio_h_`）

### 命名约定

| 层级 | 风格 | 示例 |
|------|------|------|
| `ml_libs/` 库函数 | `snake_case` | `gpio_init()`, `pwm_init()`, `uart_sendbyte()` |
| `code/` 应用层 | `PascalCase` 或 `snake_case` | `Grayscale_Sensor_Init()`, `motor_target_set()`, `pid_cal()` |
| 类型/结构体 | `snake_t` | `pid_t`, `KF_t` |
| 枚举 | `PascalCase` + `_enum` | `GPIOn_enum`, `Pinx_enum` |
| 宏/常量 | `UPPER_CASE` | `GRAYSCALE_SENSOR_CHANNELS`, `OUT_PP` |

### ml_libs 库风格（硬件抽象层）
不同于 STM32 StdPeriph 库，ml_libs 使用自定义枚举简化 API：
```c
gpio_init(GPIO_A, Pin_5, OUT_PP);       // 而非 GPIO_Init(&GPIO_InitStructure)
pwm_init(TIM_2, TIM2_CH1, 1000);        // 定时器 + 通道 + 频率
uart_init(UART_1, 115200, 0);           // 串口号 + 波特率 + 优先级
exti_init(EXTI_PA6, FALLING, 0);        // 引脚 + 触发沿 + 优先级
```
引脚模式枚举: `OUT_PP`(推挽输出), `OUT_OD`(开漏输出), `IU`(输入上拉), `IN_FLOATING`(浮空输入) 等。

### 注释风格
- 中文注释为主，部分英文
- 函数头用 `/* */` 块注释描述功能
- 行内用 `//` 简注

## 引脚速查（详见 README.md）

| 功能 | 引脚 |
|------|------|
| 灰度 AD0/AD1/AD2 | PB13/PB14/PB15 |
| 灰度 OUT | PA15 |
| I2C SCL/SDA | PB6/PB7（MPU6050 + HMC5883L + OLED 共用总线） |
| 电机 PWMA/PWMB | PA0(TIM2_CH1) / PA1(TIM2_CH2) |
| 电机方向 AIN1/AIN2/BIN1/BIN2 | PA5/PA7/PA4/PA3 |
| 编码器1 A/B | PA6(EXTI) / PA2 |
| 编码器2 A/B | PB4(EXTI) / PB5 |
| 串口 TX/RX | PA9/PA10（USART1，printf 硬编码） |
| MPU6050 INT | PB0(EXTI) |

## 关键架构决策

- **速度闭环**: PID 控制器（增量式 DELTA_PID），默认参数 P=60, I=40, D=0（在 `user/main.c` 中 `pid_init()` 设置）
- **巡线逻辑**: **分支判断法**（非 README 中的加权公式）— 8 种传感器状态 → 查表输出左右轮速度，详见 [code/gray_track.c](code/gray_track.c)
  - 状态: `GO`(直行), `R1/R2/R3`(右转三档), `L1/L2/L3`(左转三档), `FIND`(丢线反向搜索), `CROSS`(十字路口直行)
  - 速度档位宏在 `gray_track.c` 顶部: `TRACK_SPEED_FAST=25, MED=18, SLOW=12, SHARP=8, MIN=4`
  - ⚠️ README.md 中的加权差比和公式是**设计目标**，当前未实现
- **PID 周期**: TIM3 定时中断（100ms / 10Hz）调用 `pid_control()`
- **主循环频率**: ~12.5Hz（`delay_ms(80)`），非 README 中的 50Hz
- **I2C 总线共享**: 一条 I2C 总线挂 MPU6050(0xD0) + HMC5883L(0x3C) + OLED(0x78)
- **printf 重定向**: 硬编码 USART1，在 `ml_uart.c` 中实现 `fputc()`

## 关键全局变量

| 变量 | 定义位置 | 说明 |
|------|----------|------|
| `g_sensor_data[8]` | `gray_track.c` | 8 路灰度传感器值（0=白, 1=黑线） |
| `g_track_state[8]` | `gray_track.c` | 巡线状态字符串，OLED 第 2 行显示 |
| `Encoder_count1/2` | `motor.c` | 编码器脉冲计数（EXTI 累加，PID 周期清零） |
| `motorA`, `motorB` | `pid.c` | 电机 PID 结构体（含 target/now/out 等） |
| `motorA_dir`, `motorB_dir` | `motor.c` | 电机方向标志（统一：0=正转, 1=反转，见 `pid.c`） |
| `g_tim3_tick` | `isr.c` | TIM3 中断计数器（调试用） |

## OLED 显示布局约定

4 行 × 16 列（SSD1306 128×32），使用行列坐标 API `OLED_ShowNum(row, col, ...)`（非像素坐标），`main.c` 主循环更新：

| 行 (row) | 内容 | 示例 |
|----------|------|------|
| 1 | 8 路传感器值 (D1~D8) | `0 0 1 0 0 0 0 0` |
| 2 | 巡线状态 + 目标速度 | `GO  A: 25 B: 25` |
| 3 | PID 估算速度 (motorA/B.now) | `E1  12 E2  14` |
| 4 | 清空 | `                ` |

> ⚠️ 菜单系统（`Key_Init`/`menu_task_show`/`menu_task_switch`）已从 `main.c` 移除，OLED 不再显示菜单

## 注意事项

- **无 STM32 StdPeriph 库**: 本项目不依赖 FWLib，直接使用 ml_libs 封装层 + CMSIS 寄存器定义
- **GB2312 编码**: 参见上方 [⚠️ 文件编码警告](#️-文件编码警告)，部分文件含 GB2312 中文，UTF-8 下乱码
- **电机方向已统一**: `motorA_dir=0` 正转，`motorB_dir=0` 正转（见 `pid.c` 中 `motor_target_set()`）— 如实际方向反了就交换对应 `motorX_dir` 的 0/1
- **编码器 B 相检查**: 如果 OLED 上 E1/E2 正转反转都显示同一符号（全正或全负），说明编码器 B 相线松了或没接。用万用表量 PB5(Enc2 B) 和 PA2(Enc1 B) 通断
- **PWM 仅正向**: `pidout_limit()` 限幅 0~MAX_DUTY(50000)，无负值输出
- **HMC5883L 已禁用**: `isr.c` 中 `HMC5883L_GetData()` 被注释，`yaw_hmc` 恒为 0
- **EXTI0 未用也开着**: `exti_init(EXTI_PB0, ...)` 已调用但 MPU6050 未初始化，PB0 浮空可能误触发 ISR 干扰 I2C。不用 MPU6050 时应一并注释
- **菜单系统已停用**: `main.c` 中 `Key_Init()`/`menu_task_*` 已移除，`headfile.h` 中 `key.h`/`menu.h` 的 include 已注释。OLED 无菜单交互。`../menu/` 源文件不在仓库中（仅 `user/Objects/` 下有 .o/.crf 残留）
- **调试方法论**: 改代码前务必阅读 [skills/SKILL.md](skills/SKILL.md) — 含四步调试法、反例速查、硬性规则
- **AD2 引脚**: 代码和 README 均使用 PB15，原始代码误写为 PB3（`grayscale_sensor.h/c` 已修正）
- **不要在此仓库运行代码**: 本机无 Keil 环境，代码由用户烧录测试
- **VS Code IntelliSense**: `.vscode/c_cpp_properties.json` 已配置 include 路径和宏 (`STM32F10X_MD`, `__CC_ARM`)

## 常见调试与调参

### 调巡线速度
编辑 `code/gray_track.c` 顶部宏：
```c
#define TRACK_SPEED_FAST    25   // 直道
#define TRACK_SPEED_MED     18   // 中速转弯
#define TRACK_SPEED_SLOW    12   // 慢速转弯
#define TRACK_SPEED_SHARP    8   // 急转弯
#define TRACK_SPEED_MIN      4   // 原地微调
```

### 调 PID 参数
编辑 `user/main.c` 中的 `pid_init()` 调用：
```c
pid_init(&motorA, DELTA_PID, 60, 40, 0);  // P, I, D
pid_init(&motorB, DELTA_PID, 60, 40, 0);
```

### 串口调试（山外多功能调试助手）
- `printf()` 自动通过 USART1 (PA9/PA10, 115200) 输出。启动时发送 `printf("Car v1.2 Ready\r\n")` 验证串口通断
- ⚠️ `main.c` 主循环中的 `printf()` 已注释（"看波形时注释掉printf, 让datavision_send独占串口"），**printf 和 datavision_send 不可同时开启**
- `sensor_test()` 在 `gray_track.c` 中，可打印 8 路传感器原始值
- `datavision_send()` 在 `pid.c` 的 `pid_control()` 中每 100ms 调用一次，发 **山外虚拟示波器协议**: `0x03 0xFC` + 4×uint8 + `0xFC 0x03`
  - 4 字节依次: `(uint8_t)motorA.target`, `motorA.now`, `motorB.target`, `motorB.now`
  - ⚠️ 负速度被 `(uint8_t)` 强转后变成 250+ 的大数，属正常现象
  - ⚠️ 如果串口无输出，检查 `datavision_send()` 是否被注释掉了
  - ⚠️ 波特率必须 115200，9600 下 ISR 阻塞过久会卡死 OLED
- **山外助手设置**: 波特率 115200/8N1 → 打开"虚拟示波器" → 4通道 uint8

### 启停功能
- 巡线功能: 确保 `main.c` 中 `Grayscale_Sensor_Read_All()` 和 `track()` 未被注释
- MPU6050: 取消 `MPU6050_Init()` 注释 + 确保 `exti_init(EXTI_PB0, ...)` 已调用
- HMC5883L: 取消 `HMC5883L_Init()` 和 `isr.c` 中 `HMC5883L_GetData()` 注释
