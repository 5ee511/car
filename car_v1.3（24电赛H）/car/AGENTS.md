# AGENTS.md — car_v1.2 智能小车巡线控制系统

## 项目概述

基于 STM32F103C8T6 的智能小车巡线系统，具备 **编码器速度闭环 PID**、MPU6050 姿态感知、HMC5883L 磁力计、SSD1306 OLED 显示。

**关键硬件**: TB6612 电机驱动 ×2、编码器 ×2、8 路灰度传感器（亚博智能）、MPU6050、HMC5883L。

## 项目结构

```
car_v1.2/
├── code/                    # 应用层（巡线、PID、电机、滤波、UART调参）
├── ml_libs/                 # 底层驱动库（GPIO/PWM/TIM/UART/I2C/OLED 等）
│   └── headfile.h           #   **统一头文件**，所有 .c 都 include 它
├── skills/                  # AI Agent 调试方法论 → [skills/SKILL.md](skills/SKILL.md)
├── sys/                     # CMSIS 系统文件
├── user/                    # 入口（main.c, isr.c）
└── README.md                # 完整引脚表
```

## 构建系统

- **Keil MDK**，工程文件 `user/Project.uvprojx`
- Include 路径: `..\user;..\ml_libs;..\code;..\sys`（平铺式，头文件直接 include 文件名）
- **编译器**: ARM Compiler 5 (`__CC_ARM`)，预定义宏 `STM32F10X_MD`

## 时序架构（关键！）

系统有两条时间线并行运行：

| 时间线 | 周期 | 触发源 | 做什么 |
|--------|------|--------|--------|
| **PID 速度环** | 100ms | TIM3 中断 → `pid_control()` | 读编码器 → 增量 PID 计算 → 更新 PWM 占空比 |
| **主循环** | ~80ms | `main()` while(1) | 读 8 路灰度 → `track()` 巡线 → OLED 刷新 → `uart_tune_process()` |

- 编码器计数值每个 PID 周期清零 → 速度单位 = 脉冲/100ms
- 不要在 TIM3 ISR 中加耗时操作，会阻塞主循环
- ⚠️ **`motor_target_set()` 只设置 target/direction，不直接输出 PWM**：实际 PWM 在下一个 TIM3 中断（最多 100ms 后）才由 `pid_control()` 更新。这意味着主循环中多次调用 `motor_target_set()` 只有最后一次生效

## 巡线算法（gray_track.c）— 四层架构

详见 [循线控制逻辑详解.md](循线控制逻辑详解.md)，这里只给速览：

1. **防抖**: 1 帧确认传感器状态后才更新 error（单帧毛刺被过滤），但 ≥2 灯同时翻转立即响应。仅 80ms 延迟
2. **加权差比和（不除 cnt！）**: 8 路传感器加权求和，直接用 `sum_L - sum_R` 作 error（范围 ±12）。灯越多=弯越急→error越大→转向越猛
3. **丢线/十字处理**: 全白 → FIND 状态（惯性保持），全黑 → CROSS 状态（十字路口）
4. **转向 PD**: `steer_out = Kp*error + Kd*(error - prev_error)`，简洁无多余逻辑。限幅 ±60
5. **差速分配**: `L = base - steer_out, R = base + steer_out`，内轮可降到 0（停转），外轮锁死 `g_speed_fast`。弯道整体降速 70%

运行时状态显示在 OLED: `GO / R1~R3 / L1~L3 / FIND / CROSS`

## 开发工作流（AI 代理须知）

> ⚠️ **AI 代理无法编译或烧录代码**。所有代码由用户在本机 Keil MDK 中编译和烧录。

### 修改代码后的标准流程
1. 修改完成后，**明确告知用户需要做什么**（编译？烧录？测试某个功能？）
2. 如果新增了 `.c` 文件 → 提醒用户加入 Keil 工程（右键 Add Existing Files）
3. 如果修改了 `headfile.h` 的 include → 提醒用户检查 Keil include 路径
4. 改完立即让用户编译验证，不要攒一堆改动一起编译

### 新增文件时的完整清单
- 新 `.c` → 加入 Keil 工程（右键 Add Existing Files）+ 在 `headfile.h` 加 `#include "xxx.h"`
- 新全局变量 → `.c` 定义 + `.h` 加 `extern` 声明 + 需 ISR 访问的加 `volatile`
- 废弃 `.c` → 从 Keil 工程移除，否则链接报错
- 修改引脚 → 同步更新 `README.md` 引脚表 + 本 AGENTS.md 引脚速查
- 新增头文件保护宏 → 用 `__filename_h_` 格式

### 串口独占规则
`printf()` 和 `datavision_send()` 共享 USART1，**不可同时开启**。调试时二选一：
- 看文本输出 → 用 `printf()`，注释掉 `datavision_send()`
- 看波形图 → 用 `datavision_send()`，注释掉 `printf()`

### UART3 实时调参（`code/uart_tune.c`）
UART3（PB10/PB11，115200）支持运行时热调 PID 和巡线参数，无需重新编译。详见 [uart_tune_速查卡.txt](code/uart_tune_速查卡.txt)：
- `P<I<D<value>` — 两电机 PID 增益
- `S<value>` — 直道基准速度（其余三档自动缩放）
- `KP<KD<value>` — 巡线转向 P/D
- `?` — 打印全部参数
- **典型调参流程**: `? → S30 → KP4 → KD5 → P250 → ?`

## ⚠️ 文件编码警告

**本项目中文源文件使用 GB2312 编码**。在 VS Code 中打开含中文的文件（注释、字模数据、OLED 字体）时，UTF-8 下中文会显示为乱码。

- ❌ **禁止用 VS Code 编辑含 `>0x7F` 字节的文件**（如 `oled_font.c`、字模数据），会破坏 GB2312 编码导致 Keil 编译报错 `invalid multibyte character sequence`
- ✅ 修改此类文件时，用脚本处理（转换编码或 hex 编辑）
- ✅ 新增代码中的显示字符串用英文，注释可用中文（确保 GB2312 兼容）

## 关键数据结构

| 结构体 | 定义位置 | 用途 |
|--------|---------|------|
| `pid_t` | `code/pid.h` | PID 控制器（含 target/now/error[3]/p/i/d/out 等） |
| `KF_t` | `code/filter.h` | 卡尔曼滤波器（协方差矩阵 P、K1/K2 增益） |
| 全局 `pid_t motorA, motorB` | `code/pid.c` | 左右轮速度 PID 实例（DELTA_PID 模式） |
| 全局 `KF_t KF_Yaw, KF_Roll, KF_Pitch` | `code/filter.c` | 姿态角卡尔曼融合 |
| `g_sensor_data[8]` | `code/gray_track.c` | 8 路灰度原始值 0/1 |

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

- **速度闭环**: PID 控制器（增量式 DELTA_PID），默认参数 P=200, I=88, D=15（在 `user/main.c` 中 `pid_init()` 设置），详见上方 [时序架构](#时序架构关键)
- **PID 周期**: TIM3 定时中断（100ms / 10Hz）调用 `pid_control()`
- **主循环频率**: ~12.5Hz（`delay_ms(80)`），非 README 中的 50Hz
- **I2C 总线共享**: 一条 I2C 总线挂 MPU6050(0xD0) + HMC5883L(0x3C) + OLED(0x78)
- **printf 重定向**: 硬编码 USART1，在 `ml_uart.c` 中实现 `fputc()`
- **`RUN_Q1` 直行模式**: `main.c` 中 `#define RUN_Q1` 开启后，主循环不再调用 `track()`，而是用 `pid_angle(Q1_TARGET_ANGLE, g_yaw)` 做角度 PD 保持航向直行。检测到线（cnt>1）则停车。此模式用于测试 MPU6050 角度控制，正常巡线时需注释掉
- **Yaw 角仅用陀螺仪积分**: `KF_Yaw` 虽已定义但 ISR 中未使用卡尔曼融合，yaw 角为纯陀螺仪积分 `g_yaw += rate * 0.01f`（每 10ms 累加）。roll/pitch 使用卡尔曼融合（`KF_Roll`/`KF_Pitch`）
- **OLED 更新时关闭 EXTI0**: `main.c` 在 OLED 刷新前 `exti_init(EXTI_PB0, DISABLE, 0)` 防止 MPU6050 INT 打断 I2C 导致 OLED 花屏，更新完后恢复。如需在 OLED 刷新期间访问 I2C 设备，务必遵循此模式

## RUN_Q 步进表架构（电赛 H 题 Q2/Q3/Q4）

`main.c` 使用**数据驱动状态机**完成赛题流程。通过 `#define RUN_Q2` / `RUN_Q3` / `RUN_Q4` 切换：

```
编码规则: angle=0 → 循迹模式 (track), angle≠0 → 角度模式 (pid_angle)
每4步一个循环: 角度→循迹→角度→循迹
```

| 宏 | 步数 | 示例 | 说明 |
|----|------|------|------|
| `RUN_Q2` | 4 | `{0, 0, 168, 0}` | 0=循迹, 168°=掉头 |
| `RUN_Q3` | 4 | `{50, 0, 130, 0}` | 50°/130° 变角度绕桩 |
| `RUN_Q4` | 16 | 4 圈递增角度 | 角度从 50° 递增到 150° |

**步骤过渡逻辑**（`main.c` while(1)）:
1. 当前步完成条件：角度步 → 踩线(cnt>1)停车；循迹步 → 脱线(cnt=0)停车
2. 过渡帧：停车 + 蜂鸣 160ms → 清零 PID/巡线状态 → `track_reset()` → 步数+1
3. 完成后蜂鸣 4 声停车

修改赛题步骤只需改 `qx_angle[]` / `qx_speed[]` 数组，无需改动状态机逻辑。

## 关键全局变量

| 变量 | 定义位置 | 说明 |
|------|----------|------|
| `g_sensor_data[8]` | `gray_track.c` | 8 路灰度传感器值（0=白, 1=黑线） |
| `g_track_state[8]` | `gray_track.c` | 巡线状态字符串，OLED 第 2 行显示 |
| `g_track_kp` | `gray_track.c` | 转向 PD 的 Kp（float，默认 3.0） |
| `g_track_kd` | `gray_track.c` | 转向 PD 的 Kd（float，默认 4.0） |
| `g_speed_fast/slow/min` | `gray_track.c` | 巡线速度变量（int16_t，从宏初始化） |
| `Encoder_count1/2` | `motor.c` | 编码器脉冲计数（EXTI 累加，PID 周期清零） |
| `motorA`, `motorB` | `pid.c` | 电机 PID 结构体（含 target/now/p/i/d/out 等） |
| `motorA_dir`, `motorB_dir` | `motor.c` | 电机方向标志（统一：0=正转, 1=反转，见 `pid.c`） |
| `g_tim3_tick` | `isr.c` | TIM3 中断计数器（调试用） |

## OLED 显示布局约定

4 行 × 16 列（SSD1306 128×32），使用行列坐标 API `OLED_ShowNum(row, col, ...)`（非像素坐标），`main.c` 主循环更新：

| 行 (row) | 内容 | 示例 |
|----------|------|------|
| 1 | 8 路传感器值 (D1~D8) | `0 0 1 0 0 0 0 0` |
| 2 | 巡线状态 + 目标速度 | `GO  A: 25 B: 25` |
| 3 | PID 估算速度 (motorA/B.now) | `E1  12 E2  14` |
| 4 | PID 实时 P/I 值 | `A200/088B200/088` |

> ⚠️ 菜单系统（`Key_Init`/`menu_task_show`/`menu_task_switch`）已从 `main.c` 移除，OLED 不再显示菜单。`menu/` 源文件不在仓库中（仅 `user/Objects/` 下有 .o/.crf 残留）。

## 注意事项

- **无 STM32 StdPeriph 库**: 本项目不依赖 FWLib，直接使用 ml_libs 封装层 + CMSIS 寄存器定义
- **GB2312 编码**: 参见上方 [⚠️ 文件编码警告](#️-文件编码警告)，部分文件含 GB2312 中文，UTF-8 下乱码
- **电机方向已统一**: `motorA_dir=0` 正转，`motorB_dir=0` 正转（见 `pid.c` 中 `motor_target_set()`）— 如实际方向反了就交换对应 `motorX_dir` 的 0/1。⚠️ 注意 `motor.c` 中旧注释写的是 `1`=正转，但实际代码逻辑为 `0`=正转，以 `pid.c` 中 `motor_target_set()` 的实现为准
- **编码器 B 相检查**: 如果 OLED 上 E1/E2 正转反转都显示同一符号（全正或全负），说明编码器 B 相线松了或没接。用万用表量 PB5(Enc2 B) 和 PA2(Enc1 B) 通断
- **PWM 仅正向**: `pidout_limit()` 限幅 0~MAX_DUTY(50000)，无负值输出
- **HMC5883L 已禁用**: `isr.c` 中 `HMC5883L_GetData()` 被注释，`yaw_hmc` 恒为 0
- **EXTI0 未用也开着**: `exti_init(EXTI_PB0, ...)` 已调用但 MPU6050 未初始化，PB0 浮空可能误触发 ISR 干扰 I2C。不用 MPU6050 时应一并注释
- **菜单系统已停用**: `main.c` 中 `Key_Init()`/`menu_task_*` 已移除，`headfile.h` 中 `key.h`/`menu.h` 的 include 已注释。OLED 无菜单交互。`menu/` 源文件不在仓库中（仅 `user/Objects/` 下有 .o/.crf 残留）
- **UART3 调参**: `code/uart_tune.c/h` 提供实时 PID/速度调参。在 `uart_tune.h` 中 `#define USE_UART_TUNE` 启用。`main.c` 中 `uart_tune_init(115200)` 和主循环 `uart_tune_process()` 已调用。UART3 ISR (`ml_uart.c`) 中调用 `uart_tune_rx_isr()` 喂字节到环形缓冲区。详见 [code/uart_tune_速查卡.txt](code/uart_tune_速查卡.txt)
- **调试方法论**: 改代码前务必阅读 [skills/SKILL.md](skills/SKILL.md) — 含四步调试法、反例速查、硬性规则
- **编码器符号陷阱 ⚠️**: `pid_control()` 中 `motorA_dir=0` 表示正转，此时编码器正向计数（Encoder_count1>0），所以 `motorA.now = Encoder_count1`（不取反）。`motorA_dir=1` 反转时编码器反向计数（<0），`motorA.now = -Encoder_count1` 使其回正。修改电机方向或编码器逻辑时必须同步检查这四处
- **AD2 引脚**: 代码和 README 均使用 PB15，原始代码误写为 PB3（`grayscale_sensor.h/c` 已修正）
- **不要在此仓库运行代码**: 本机无 Keil 环境，代码由用户烧录测试
- **VS Code IntelliSense**: `.vscode/c_cpp_properties.json` 已配置 include 路径和宏 (`STM32F10X_MD`, `__CC_ARM`)
- **README.md 与代码不一致**: README 中描述的主循环频率为 ~50Hz、菜单系统为活跃状态——这些与实际代码不符。以本 AGENTS.md 和实际代码为准。README 的引脚表是准确的

## 常见调试与调参

### 调巡线速度与转向 PD
编辑 `code/gray_track.c` 顶部宏和全局变量：
```c
#define TRACK_SPEED_FAST    30   // 直道/基准速度 (弯道拐不过就先降这个)
#define TRACK_SPEED_SLOW    18   // 丢线搜索速度
#define TRACK_SPEED_MIN      2   // 弯道base保底速度
float g_track_kp = 5.8f;        // 转向P: 越大转弯越猛
float g_track_kd = 3.0f;        // 转向D: 越大越稳
#define STEER_MAX        60     // 转向输出上限
```

或通过 UART3 实时调参（无需重新编译）:
- `KP<值>` / `KD<值>` — 实时修改转向 PD
- `S<值>` — 按比例缩放全部速度档位

### 调 PID 参数
编辑 `user/main.c` 中的 `pid_init()` 调用：
```c
pid_init(&motorA, DELTA_PID, 200, 88, 15);  // P, I, D
pid_init(&motorB, DELTA_PID, 200, 88, 15);
```

或通过 UART3 实时调参: `P<值>` / `I<值>` / `D<值>`（双电机），`PA<值>` / `IA<值>` / `DA<值>`（仅电机A），`PB<值>` / `IB<值>` / `DB<值>`（仅电机B）。

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
