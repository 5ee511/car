#include "headfile.h"

#ifdef USE_UART_TUNE

// ============================================================
// 环形缓冲区 — 接收 UART3 字节
// ============================================================
#define RX_BUF_SIZE  64

static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;   // ISR 写入位置
static uint8_t  rx_tail = 0;           // 主循环读取位置

// 命令行缓冲区
static char  cmd_buf[32];
static uint8_t cmd_idx = 0;

// PID 默认值 (用于比例缩放参考)
// （保留以备将来使用）
// static const float PID_P_DEFAULT = 60.0f;
// static const float PID_I_DEFAULT = 40.0f;
// static const float PID_D_DEFAULT = 0.0f;

// ============================================================
// 前向声明
// ============================================================
static void tune_apply_speed_scale(int16_t fast_val);
static void tune_print_status(void);
static void tune_print_help(void);
static int  tune_parse_float(const char *s, float *out);

// ============================================================
// uart_tune_init — 初始化 UART3
// ============================================================
void uart_tune_init(uint32_t baud)
{
    uart_init(UART_3, (int)baud, 1);   // 优先级 1（低于 TIM3=0）
    uart_sendstr(UART_3, "\r\n=== UART3 Tune Ready ===\r\n");
    uart_sendstr(UART_3, "Type ? for status, HELP for commands\r\n");
}

// ============================================================
// uart_tune_rx_isr — 在 USART3 中断中调用，存入环形缓冲区
// ============================================================
void uart_tune_rx_isr(uint8_t byte)
{
    uint8_t next = (rx_head + 1) % RX_BUF_SIZE;
    if (next != rx_tail)                // 缓冲区未满
    {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
    // 缓冲区满则丢弃（避免死锁）
}

// ============================================================
// uart_tune_process — 主循环中调用，解析命令
// ============================================================
void uart_tune_process(void)
{
    // 从环形缓冲区取字节
    while (rx_tail != rx_head)
    {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        // 以 \r 或 \n 为命令结束符
        if (byte == '\r' || byte == '\n')
        {
            if (cmd_idx > 0)
            {
                cmd_buf[cmd_idx] = '\0';
                cmd_idx = 0;

                // ---- 解析命令 ----
                char c0 = cmd_buf[0];
                char c1 = cmd_buf[1];

                if (c0 == '?' || (c0 == 'S' && c1 == 'T'))   // ? 或 STATUS
                {
                    tune_print_status();
                }
                else if (c0 == 'H' || (c0 == 'h'))            // HELP
                {
                    tune_print_help();
                }
                // --- PID 双电机 ---
                else if (c0 == 'P' && (c1 < 'A' || c1 > 'Z'))
                {
                    float v; if (tune_parse_float(cmd_buf + 1, &v))
                    { motorA.p = v; motorB.p = v; }
                }
                else if (c0 == 'I' && (c1 < 'A' || c1 > 'Z'))
                {
                    float v; if (tune_parse_float(cmd_buf + 1, &v))
                    { motorA.i = v; motorB.i = v; }
                }
                else if (c0 == 'D' && (c1 < 'A' || c1 > 'Z'))
                {
                    float v; if (tune_parse_float(cmd_buf + 1, &v))
                    { motorA.d = v; motorB.d = v; }
                }
                // --- PID 单电机A ---
                else if (c0 == 'P' && c1 == 'A')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorA.p = v; }
                }
                else if (c0 == 'I' && c1 == 'A')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorA.i = v; }
                }
                else if (c0 == 'D' && c1 == 'A')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorA.d = v; }
                }
                // --- PID 单电机B ---
                else if (c0 == 'P' && c1 == 'B')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorB.p = v; }
                }
                else if (c0 == 'I' && c1 == 'B')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorB.i = v; }
                }
                else if (c0 == 'D' && c1 == 'B')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { motorB.d = v; }
                }
                // --- 巡线速度一键缩放 ---
                else if (c0 == 'S')
                {
                    int v = 0;
                    for (uint8_t i = 1; cmd_buf[i] >= '0' && cmd_buf[i] <= '9'; i++)
                        v = v * 10 + (cmd_buf[i] - '0');
                    if (v > 0 && v <= 200)
                        tune_apply_speed_scale((int16_t)v);
                }
                // --- 转向 PD: KP / KD ---
                else if (c0 == 'K' && c1 == 'P')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { g_track_kp = v; }
                }
                else if (c0 == 'K' && c1 == 'D')
                {
                    float v; if (tune_parse_float(cmd_buf + 2, &v))
                    { g_track_kd = v; }
                }
                else
                {
                    // 未知命令
                    uart_sendstr(UART_3, "? unknown (try HELP)\r\n");
                }
            }
        }
        else if (cmd_idx < sizeof(cmd_buf) - 1)
        {
            cmd_buf[cmd_idx++] = (char)byte;
        }
    }
}

// ============================================================
// tune_apply_speed_scale — 按比例缩放全部巡线速度
// ============================================================
static void tune_apply_speed_scale(int16_t fast_val)
{
    // 基于当前 fast 值的比例缩放
    if (g_speed_fast == 0) return;
    float ratio = (float)fast_val / (float)g_speed_fast;

    g_speed_fast  = fast_val;
    g_speed_slow  = (int16_t)((float)g_speed_slow  * ratio + 0.5f);
    g_speed_min   = (int16_t)((float)g_speed_min   * ratio + 0.5f);

    char buf[80];
    sprintf(buf, "Speed: F=%d S=%d N=%d\r\n",
            g_speed_fast, g_speed_slow, g_speed_min);
    uart_sendstr(UART_3, buf);
}

// ============================================================
// tune_print_status — 打印当前 PID 和速度
// ============================================================
static void tune_print_status(void)
{
    char buf[120];

    // PID 值
    sprintf(buf,
        "--- PID Status ---\r\n"
        "MotorA: P=%.1f I=%.1f D=%.1f\r\n"
        "MotorB: P=%.1f I=%.1f D=%.1f\r\n",
        motorA.p, motorA.i, motorA.d,
        motorB.p, motorB.i, motorB.d);
    uart_sendstr(UART_3, buf);

    // 速度值
    sprintf(buf,
        "Speed : F=%d S=%d N=%d\r\n"
        "Steer : KP=%.1f KD=%.1f\r\n"
        "Encoder: A=%d/%d B=%d/%d\r\n"
        "State : %s\r\n",
        g_speed_fast, g_speed_slow, g_speed_min,
        g_track_kp, g_track_kd,
        (int)motorA.target, (int)motorA.now,
        (int)motorB.target, (int)motorB.now,
        g_track_state);
    uart_sendstr(UART_3, buf);
}

// ============================================================
// tune_print_help
// ============================================================
static void tune_print_help(void)
{
    char buf[200];
    sprintf(buf,
        "\r\n=== Commands (UART3 115200) ===\r\n"
        " P/I/D<val>     both motors PID      (now: P=%.0f I=%.0f D=%.0f)\r\n"
        " PA/IA/DA<val>  motorA only\r\n"
        " PB/IB/DB<val>  motorB only\r\n"
        " S<int>         scale all speeds      (now: F=%d S=%d N=%d)\r\n"
        " KP<float>      steer P               (now: %.1f)\r\n"
        " KD<float>      steer D               (now: %.1f)\r\n"
        " ?              print full status\r\n"
        " HELP           this message\r\n",
        motorA.p, motorA.i, motorA.d,
        g_speed_fast, g_speed_slow, g_speed_min,
        g_track_kp, g_track_kd);
    uart_sendstr(UART_3, buf);
}

// ============================================================
// tune_parse_float — 从字符串解析浮点数，返回 1=成功
// ============================================================
static int tune_parse_float(const char *s, float *out)
{
    if (!s || !*s) return 0;

    int   sign = 1;
    float val  = 0.0f;

    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    // 整数部分
    while (*s >= '0' && *s <= '9')
        val = val * 10.0f + (float)(*s++ - '0');

    // 小数部分
    if (*s == '.')
    {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9')
        {
            val += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }

    *out = val * (float)sign;
    return 1;
}

#endif // USE_UART_TUNE
