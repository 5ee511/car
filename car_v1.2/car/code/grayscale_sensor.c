 #include "headfile.h"


static void _delay_us(volatile uint32_t us)
{
    delay_us(us);
}

// 选择传感器通道 select sensor channel
static void _select_channel(uint8_t channel)
{
    SENSOR_AD0_WRITE((channel >> 0) & 0x01);  // bit0 -> AD0
    SENSOR_AD1_WRITE((channel >> 1) & 0x01);  // bit1 -> AD1
    SENSOR_AD2_WRITE((channel >> 2) & 0x01);  // bit2 -> AD2
}

// 读取OUT引脚的值 read the value of OUT pin
static uint16_t Read_OUT_value(void)
{
    return SENSOR_OUT_READ();
}

// 初始化灰度传感器所需的GPIO / init GPIO for grayscale sensor
void Grayscale_Sensor_Init(void)
{
    // ==== 禁用JTAG，保留SWD ====  Disable JTAG, keep SWD
    // PA15(JTDI)/PB3(JTDO)/PB4(JNTRST) 释放为普通GPIO  Release as normal GPIO
    RCC->APB2ENR |= 1 << 0;          // 使能AFIO时钟  Enable AFIO clock
    AFIO->MAPR &= 0xF8FFFFFF;        // 清除SWJ_CFG[2:0]  Clear SWJ config
    AFIO->MAPR |= 0x02000000;        // SWJ_CFG=010: 仅SWD使能, JTAG禁用  SWD only, JTAG off

    // 初始化 AD0, AD1, AD2 为推挽输出  AD0(PB13), AD1(PB14), AD2(PB15)
    gpio_init(GPIO_B, Pin_13, OUT_PP);
    gpio_init(GPIO_B, Pin_14, OUT_PP);
    gpio_init(GPIO_B, Pin_15, OUT_PP);  // AD2 -> PB15
    // 初始化 OUT 引脚为浮空输入 (与亚博官方一致)  OUT pin as floating input (same as Yahboom official)
    // 注: gpio_init的IF枚举值与OUT_PP冲突(bug), 此处直接写寄存器
    // Note: IF enum conflicts with OUT_PP in gpio_init (bug), write register directly
    RCC->APB2ENR |= 1 << 2;                         // 确保GPIOA时钟已开  Ensure GPIOA clock on
    GPIOA->CRH &= ~(0x0000000FUL << ((15-8)*4));      // 清除PA15配置  Clear PA15 config
    GPIOA->CRH |= 0x00000004UL << ((15-8)*4);         // CNF=01 MODE=00 → 浮空输入  Floating input
}

// 读取所有8个通道的灰度值 read all 8 channels grayscale values
void Grayscale_Sensor_Read_All(uint16_t* sensor_values)
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        _select_channel(i);
        _delay_us(50);
        sensor_values[i] = Read_OUT_value();
    }
}

// 读取单个指定通道的灰度值 read single specified channel grayscale value
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS)
    {
        return 0; // 无效通道 // Invalid channel
    }
    _select_channel(channel);
    _delay_us(50);
    return Read_OUT_value();
}
