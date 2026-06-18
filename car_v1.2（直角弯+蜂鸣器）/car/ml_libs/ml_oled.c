#include "ml_oled_font.h"
#include "math.h"
#include "headfile.h"

// OLED驱动代码参考自江协科技 特此说明

// 引脚配置
#define OLED_W_SCL(x)   gpio_set(OLED_GPIO,OLED_SCL_Pin,x)
#define OLED_W_SDA(x)   gpio_set(OLED_GPIO,OLED_SDA_Pin,x)

// 引脚初始化
void OLED_I2C_Init(void)
{
    gpio_init(OLED_GPIO,OLED_SCL_Pin,OUT_OD);
    gpio_init(OLED_GPIO,OLED_SDA_Pin,OUT_OD);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

// I2C开始
void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

// I2C停止
void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

// I2C发送一个字节
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(Byte & (0x80 >> i));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);  // 额外的一个时钟，不处理应答信号
    OLED_W_SCL(0);
}

// OLED写命令
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);     // 从机地址
    OLED_I2C_SendByte(0x00);     // 写命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

// OLED写数据
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);     // 从机地址
    OLED_I2C_SendByte(0x40);     // 写数据
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

// OLED设置光标位置
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    // 设置Y位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    // 设置X位置低4位
    OLED_WriteCommand(0x00 | (X & 0x0F));            // 设置X位置高4位
}

// OLED清屏
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

// OLED显示一个字符
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);       // 设置光标在上半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);            // 显示上半部分
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);   // 设置光标在下半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);       // 显示下半部分
    }
}

// OLED显示字符串
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

// 幂函数：返回X的Y次方
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

// OLED显示数字（十进制，正数）
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

// OLED显示有符号数字（十进制）
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

// OLED显示十六进制数字
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

// OLED显示二进制数字
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

// OLED显示浮点数
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t int_Length, uint8_t float_Length)
{
    int temp1, temp2;
    temp1 = (int)Number;
    temp2 = (Number - temp1) * pow(10, float_Length);
    OLED_ShowSignedNum(Line, Column, temp1, int_Length);
    OLED_ShowString(Line, Column+int_Length+1, ".");
    OLED_ShowNum(Line, Column+int_Length+2, temp2, float_Length);
}

// OLED初始化
void OLED_Init(void)
{
    uint32_t i, j;

    for (i = 0; i < 1000; i++)      // 上电延时
    {
        for (j = 0; j < 1000; j++);
    }

    OLED_I2C_Init();            // 端口初始化

    OLED_WriteCommand(0xAE);    // 关闭显示

    OLED_WriteCommand(0xD5);    // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    // 设置多路复用率
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    // 设置显示偏移
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);    // 设置显示开始行

    OLED_WriteCommand(0xA1);    // 设置左右方向，0xA1正常 0xA0左右反置

    OLED_WriteCommand(0xC8);    // 设置上下方向，0xC8正常 0xC0上下反置

    OLED_WriteCommand(0xDA);    // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);    // 设置对比度
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);    // 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);    // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);    // 设置整个显示打开/关闭

    OLED_WriteCommand(0xA6);    // 设置正常/反色显示

    OLED_WriteCommand(0x8D);    // 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);    // 开启显示

    OLED_Clear();               // 清屏
    OLED_SetCursor(0, 0);       // 设置光标位置
}