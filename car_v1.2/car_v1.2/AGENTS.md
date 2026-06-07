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
├── sys/                     # CMSIS 系统文件
├── user/                    # 入口（main.c, isr.c）
└── README.md                # 完整引脚表
```

## 构建系统

- **Keil MDK**，工程文件 `user/Project.uvprojx`
- Include 路径: `..\user;..\ml_libs;..\code;..\sys`（平铺式，头文件直接 include 文件名）

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
| 电机 STBY | PB12（TB6612 使能，高电平有效） |
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
| `motorA_dir`, `motorB_dir` | `motor.c` | 电机方向标志（1=正转, 0=反转，A/B 统一） |
| `g_tim3_tick` | `isr.c` | TIM3 中断计数器（调试用） |

## OLED 显示布局约定

4 行 × 16 列（SSD1306 128×32 模式），`main.c` 主循环中更新：

| 行 | 内容 | 示例 |
|----|------|------|
| Line 1 | 8 路传感器值 (D1~D8) | `0 0 1 0 0 0 0 0` |
| Line 2 | 巡线状态 + 目标速度 | `GO   A:25 B:25` |
| Line 3 | 编码器实际速度 | `E1 12 E2 14` |
| Line 4 | 预留（当前清空） | |

## 注意事项

- **头文件编码**: 部分文件含 GB2312 中文注释，UTF-8 下可能乱码
- **无 STM32 StdPeriph 库**: 本项目不依赖 FWLib，直接使用 ml_libs 封装层 + CMSIS 寄存器定义
- **电机方向不对称**: `motorA_dir=0` 正转，但 `motorB_dir=1` 正转（见 `motor_target_set()`）— 与实际接线相关
- **编码器方向**: 在 `isr.c` 中根据 B 相电平增减 `Encoder_count1/2`
- **PWM 仅正向**: `pidout_limit()` 限幅 0~MAX_DUTY(50000)，无负值输出
- **HMC5883L 已禁用**: `isr.c` 中 `HMC5883L_GetData()` 被注释，`yaw_hmc` 恒为 0
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

### 串口调试
- `printf()` 自动通过 USART1 (PA9/PA10, 115200) 输出
- `sensor_test()` 在 `gray_track.c` 中，可打印 8 路传感器原始值
- `datavision_send()` 在 `pid.c` 中，发送二进制波形数据（协议: 0x03 0xFC + 4字节数据 + 0xFC 0x03）

### 启停功能
- 巡线功能: 确保 `main.c` 中 `Grayscale_Sensor_Read_All()` 和 `track()` 未被注释
- MPU6050: 取消 `MPU6050_Init()` 注释 + 确保 `exti_init(EXTI_PB0, ...)` 已调用
- HMC5883L: 取消 `HMC5883L_Init()` 和 `isr.c` 中 `HMC5883L_GetData()` 注释
