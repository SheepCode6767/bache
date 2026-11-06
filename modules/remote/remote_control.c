#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

/* 接收缓冲的大小(REMOTE_CONTROL_FRAME_SIZE)在头文件里定义:
 * 要同时装得下 DBUS 的18字节和 SBUS 的25字节,所以取大的那个。 */

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

/* 调试镜像(说明见头文件),Ozone 的 watch 窗口里直接敲名字就能看 */
volatile int16_t rc_dbg_ch[RC_SBUS_CH_NUM] = {0};
volatile uint8_t rc_dbg_protocol           = RC_PROTO_UNKNOWN;
volatile uint32_t rc_dbg_dbus_frames       = 0;
volatile uint32_t rc_dbg_sbus_frames       = 0;
volatile uint32_t rc_dbg_bad_frames        = 0;
volatile int16_t rc_dbg_sw_left_raw        = 0;
volatile int16_t rc_dbg_sw_right_raw       = 0;
volatile int16_t rc_dbg_dial_raw           = 0;
volatile int16_t rc_dbg_ld_raw             = 0;
volatile int16_t rc_dbg_rd_raw             = 0;
volatile uint8_t rc_dbg_sbus_flags         = 0;
volatile uint8_t rc_dbg_ch_dump_req        = 0; /* 置1就把16个通道原始值打出来(见robot_cmd.c) */

/* SBUS 的 flags 字节 bit3=失控保护、bit2=丢帧。接收机在发射机关机后仍可能继续发
 * 25 字节的"失控保护帧";如果不看这一位,固件会把预设的失控值当成正常遥控数据。
 * 这里一旦看到这两位就认为遥控器不在线,应用层会走急停。 */
static volatile uint8_t rc_sbus_failsafe = 0;

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660||abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i))<10)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief DBUS(大疆 DT7 + DR16 接收机)数据解析
 *
 * @param sbus_buf 接收buffer(形参名是历史遗留,这里装的其实是 DBUS 帧)
 */
static void dbus_frame_to_rc(const uint8_t *sbus_buf)
{
    rc_sbus_failsafe = 0; // DBUS 帧没有 SBUS 的 flags 字节
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;  
    /* DBUS 没有 LD/RD,给中点作为安全默认值;本车速度映射使用 ET08A 时才会读到真实旋钮。 */
    rc_ctrl[TEMP].rc.knob_ld = RC_STICK_FULL_SCALE / 2;
    rc_ctrl[TEMP].rc.knob_rd = RC_STICK_FULL_SCALE / 2;
    
    // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    rc_ctrl[TEMP].rc_update_flag = 1;
}

/**
 * @brief 把 SBUS 一帧里的 16 个 11 位通道拆出来
 *
 * @param buf 一帧 25 字节:buf[0]=0x0F 帧头, buf[1..22]=通道数据, buf[23]=标志位, buf[24]=0x00 帧尾
 * @param ch  输出:16 个通道的原始值(0~2047)
 */
static void SBUSUnpackChannels(const uint8_t *buf, int16_t *ch)
{
    for (uint8_t i = 0; i < RC_SBUS_CH_NUM; i++) {
        uint16_t bit  = (uint16_t)i * 11u; /* 该通道的起始bit(相对数据区) */
        uint16_t byte = bit >> 3;          /* 起始字节,相对数据区;真正的下标要+1,因为 buf[0] 是帧头 */
        uint8_t shift = bit & 0x07u;
        /* 11位最多跨3个字节,干脆把3个字节都拼进一个32位变量再移位取值(多拼进来的高位会被掩码滤掉) */
        uint32_t acc = (uint32_t)buf[byte + 1] | ((uint32_t)buf[byte + 2] << 8) | ((uint32_t)buf[byte + 3] << 16);
        ch[i] = (int16_t)((acc >> shift) & 0x07FFu);
    }
}

/**
 * @brief SBUS 通道原始值(0~2047)换算成和 DBUS 一样的"中位0、满量程±660"
 *
 * @note 换算比例取 660/(1696-1024):一端 352 -> -660,另一端 1696 -> +660。
 *       这样 app 层(robot_cmd 等)拿到的量纲和用 DT7 时完全一样,不用改。
 */
static int16_t SBUSChToStick(int16_t raw)
{
    int32_t v = ((int32_t)raw - RC_SBUS_CH_MID) * RC_STICK_FULL_SCALE / (RC_SBUS_CH_MAX - RC_SBUS_CH_MID);
    if (v > RC_STICK_FULL_SCALE)
        v = RC_STICK_FULL_SCALE;
    if (v < -RC_STICK_FULL_SCALE)
        v = -RC_STICK_FULL_SCALE;
    return (int16_t)v;
}

/**
 * @brief 把 SBUS 旋钮通道换算为 0~660 的单向行程
 *
 * @note LD/RD 只使用旋钮的完整行程,不按摇杆中位处理。最小值 352 -> 0,
 *       最大值 1696 -> 660;超出范围的抖动会被限幅。
 */
static int16_t SBUSChToKnob(int16_t raw)
{
    int32_t v = ((int32_t)raw - RC_SBUS_CH_MIN) * RC_STICK_FULL_SCALE /
                (RC_SBUS_CH_MAX - RC_SBUS_CH_MIN);
    if (v < 0)
        v = 0;
    if (v > RC_STICK_FULL_SCALE)
        v = RC_STICK_FULL_SCALE;
    return (int16_t)v;
}

/**
 * @brief 两档开关(A/D)专用:只接受接近两端的值
 *
 * @return RC_SW_UP / RC_SW_DOWN;通道没分配、接触不良或落在中位时返回 0(无效)
 * @note 之所以不把中位当成"下档":通道填错时中位是常见结果,直接当"下档"会在
 *       "另一路被读成上档"时进入定速/变速档,现场表现就是双下仍转。返回 0 后
 *       应用层会把任何一路无效都当成急停。
 */
static uint8_t SBUSChToTwoPosSwitch(int16_t raw, uint8_t invert)
{
    if (invert)
        raw = (int16_t)(2 * RC_SBUS_CH_MID - raw);
    if (raw > RC_SBUS_SW_TH_HIGH)
        return RC_SW_UP;
    if (raw < RC_SBUS_SW_TH_LOW)
        return RC_SW_DOWN;
    return 0;
}

/**
 * @brief SBUS(ET08A + RF206S)数据解析
 *
 * @param buf 一帧 25 字节,见 SBUSUnpackChannels 的说明
 * @note ET08A 的帧里没有鼠标/键盘数据(DBUS 里那部分是图传链路注进去的),这里一律清零,
 *       免得留着上一帧的残留值。
 */
static void sbus_frame_to_rc(const uint8_t *buf)
{
    int16_t ch[RC_SBUS_CH_NUM];

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键/换档判断

    SBUSUnpackChannels(buf, ch);
    rc_dbg_sbus_flags = buf[23];
    rc_sbus_failsafe  = (buf[23] & 0x0Cu) ? 1u : 0u; // bit2=丢帧, bit3=失控保护
    for (uint8_t i = 0; i < RC_SBUS_CH_NUM; i++)
        rc_dbg_ch[i] = ch[i]; // 镜像给调试器:核对"哪个开关落在哪个通道"用
    rc_dbg_sw_left_raw  = ch[RC_SBUS_CH_SW_LEFT];
    rc_dbg_sw_right_raw = ch[RC_SBUS_CH_SW_RIGHT];
    rc_dbg_dial_raw     = ch[RC_SBUS_CH_DIAL];
    rc_dbg_ld_raw       = ch[RC_SBUS_CH_LD];
    rc_dbg_rd_raw       = ch[RC_SBUS_CH_RD];

    rc_ctrl[TEMP].rc.rocker_r_ = SBUSChToStick(ch[RC_SBUS_CH_RIGHT_H]);
    rc_ctrl[TEMP].rc.rocker_r1 = SBUSChToStick(ch[RC_SBUS_CH_RIGHT_V]);
    rc_ctrl[TEMP].rc.rocker_l_ = 0; /* CH4 已分配给 RD */
    rc_ctrl[TEMP].rc.rocker_l1 = 0; /* 当前发射机没有左摇杆竖直通道 */
    rc_ctrl[TEMP].rc.dial      = SBUSChToStick(ch[RC_SBUS_CH_DIAL]);
    rc_ctrl[TEMP].rc.knob_ld   = SBUSChToKnob(ch[RC_SBUS_CH_LD]);
    rc_ctrl[TEMP].rc.knob_rd   = SBUSChToKnob(ch[RC_SBUS_CH_RD]);
    if (RC_SBUS_INVERT_DIAL) /* C 是模拟量:反转 = 值取负(上和下互换) */
        rc_ctrl[TEMP].rc.dial = (int16_t)(-rc_ctrl[TEMP].rc.dial);
    if (RC_SBUS_INVERT_LD)
        rc_ctrl[TEMP].rc.knob_ld = (int16_t)(RC_STICK_FULL_SCALE - rc_ctrl[TEMP].rc.knob_ld);
    if (RC_SBUS_INVERT_RD)
        rc_ctrl[TEMP].rc.knob_rd = (int16_t)(RC_STICK_FULL_SCALE - rc_ctrl[TEMP].rc.knob_rd);

    RectifyRCjoystick(); // 和 DBUS 一样:|v|>660 或 |v|<10 都当0(无效值过滤 + 死区)

    rc_ctrl[TEMP].rc.switch_left  = SBUSChToTwoPosSwitch(ch[RC_SBUS_CH_SW_LEFT], RC_SBUS_INVERT_SW_LEFT);
    rc_ctrl[TEMP].rc.switch_right = SBUSChToTwoPosSwitch(ch[RC_SBUS_CH_SW_RIGHT], RC_SBUS_INVERT_SW_RIGHT);

    rc_ctrl[TEMP].mouse.x       = 0;
    rc_ctrl[TEMP].mouse.y       = 0;
    rc_ctrl[TEMP].mouse.press_l = 0;
    rc_ctrl[TEMP].mouse.press_r = 0;
    memset(rc_ctrl[TEMP].key, 0, sizeof(rc_ctrl[TEMP].key)); // 按下/带ctrl/带shift 三个状态都清零

    rc_ctrl[TEMP].rc_update_flag = 1;
}

/* ================= 协议自动识别 =================
 * SBUS 的帧头帧尾很有特征(第1个字节 0x0F、最后一个字节 0x00),DBUS 帧没有固定头,
 * 所以识别办法是:连续 RC_SBUS_CONFIRM 帧都像 SBUS 才切到 SBUS;SBUS 模式下连续丢掉
 * RC_SBUS_MISS_MAX 帧(半包/干扰)就退回去重新识别。
 * DBUS 帧里偶尔也会正好出现"0x0F 开头"的巧合,所以不能"看到一次就切"。 */
#define RC_SBUS_CONFIRM  3u // 连续像 3 帧才认
#define RC_SBUS_MISS_MAX 3u // SBUS 模式下连丢 3 帧就重新识别

static uint8_t rc_sbus_confirm = 0;
static uint8_t rc_sbus_miss    = 0;

/**
 * @brief 串口每收到一帧就进这里一次(注册给 bsp_usart 的回调)
 *
 * @note 这一帧是 DBUS 还是 SBUS 由上面那段自动识别决定;两种协议都解析成同一个 RC_ctrl_t,
 *       所以 app 层不需要关心现在插的是哪个遥控器。
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance); // 先喂狗
    uint8_t *buf     = rc_usart_instance->recv_buff;
    uint8_t is_sbus  = (buf[0] == RC_SBUS_HEADER && buf[RC_SBUS_FRAME_SIZE - 1] == RC_SBUS_FOOTER);

    if (rc_dbg_protocol == RC_PROTO_SBUS) {
        if (is_sbus) {
            rc_sbus_miss = 0;
            sbus_frame_to_rc(buf);
            rc_dbg_sbus_frames++;
            return;
        }
        /* 半包或有干扰:这一帧直接丢掉,控制量保持上一次的值不动,等下一帧(不拿垃圾数据去开车) */
        rc_dbg_bad_frames++;
        if (++rc_sbus_miss >= RC_SBUS_MISS_MAX) {
            rc_dbg_protocol = RC_PROTO_UNKNOWN;
            rc_sbus_miss    = 0;
            rc_sbus_confirm = 0;
        }
        return;
    }

    if (is_sbus) {
        /* 还在确认中:这几帧先不解析,免得把 SBUS 帧当 DBUS 解析、把垃圾值喂给控制层 */
        if (++rc_sbus_confirm < RC_SBUS_CONFIRM)
            return;
        rc_dbg_protocol = RC_PROTO_SBUS;
        rc_sbus_confirm = 0;
        sbus_frame_to_rc(buf);
        rc_dbg_sbus_frames++;
        return;
    }
    rc_sbus_confirm = 0;

    rc_dbg_protocol = RC_PROTO_DBUS;
    dbus_frame_to_rc(buf);
    rc_dbg_dbus_frames++;
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    rc_sbus_failsafe = 0;
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);
    
    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 45, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    // rc_daemon_instance->temp_count--;
    // if(rc_daemon_instance->temp_count <= 0){
    //     rc_daemon_instance->temp_count = 0;
    // }
    if (rc_init_flag)
        return (uint8_t)(!rc_sbus_failsafe && DaemonIsOnline(rc_daemon_instance));
    return 0;
}
