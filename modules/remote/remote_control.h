/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- 接收机协议(两种都支持,自动识别) ------------------------
 * 车上可以插两种接收机,插哪个用哪个,不用改代码、不用重新编译:
 *   DBUS: 大疆 DT7 发射机 + DR16 接收机(原来那套),一帧 18 字节
 *   SBUS: ET08A 发射机 + RF206S 接收机(新遥控器,把最后一个端口设成 W.BUS 模式即可,
 *         说明书上写的是"W.BUS:兼容S.BUS"),一帧 25 字节
 * 两种在电气上是一样的(100000 波特、偶校验、反相),所以接同一根线(USART3 / PC11),
 * 硬件不用动。接法、对码步骤见 modules/remote/remote.md。 */
#define RC_DBUS_FRAME_SIZE 18u
#define RC_SBUS_FRAME_SIZE 25u
/* 接收缓冲要能装下最大的那一帧(SBUS 25 字节)。接收是"一帧结束就中断一次"(IDLE 中断),
 * DBUS 的 18 字节帧照样只占 18 个字节,两种协议不会混在一起。 */
#define REMOTE_CONTROL_FRAME_SIZE RC_SBUS_FRAME_SIZE

#define RC_SBUS_HEADER 0x0Fu /* SBUS 帧头 */
#define RC_SBUS_FOOTER 0x00u /* SBUS 帧尾 */
#define RC_SBUS_CH_NUM 16u   /* SBUS 一帧里有 16 个通道 */

/* DBUS 摇杆的满量程:364~1684,中位1024,上下各660(robot_cmd.c 里的 RC_STICK_FULL 是同一个数) */
#define RC_STICK_FULL_SCALE 660
/* ET08A/RF206S 实测 SBUS 通道端点:352~1696,中位1024。模块层会把 SBUS 换算成和 DBUS 一样的
 * "中位0、满量程±660",所以 app 层(robot_cmd 等)一行都不用改。 */
#define RC_SBUS_CH_MIN 352
#define RC_SBUS_CH_MID 1024
#define RC_SBUS_CH_MAX 1696

/* ================= SBUS(ET08A)的通道映射 =================
 * 下面这组值按 ET08A 说明书里的【固定翼 + 摇杆模式1】默认连接填写:
 *   本车实测:CH1=右摇杆水平, CH2=右摇杆竖直, CH3=LD(旋钮), CH4=RD(旋钮)
 *   CH5=SD(右两档), CH6=SA(左两档), CH7=SC(旋钮/三档), CH8=未使用
 *
 * 实机核对时发现本车的 A/D 与说明书默认分配相反:A 实际落在 CH5、D 实际落在 CH6,
 * 而且两路都是"上=低值、下=高值"。所以下面按**实测**写成 A=CH5、D=CH6,
 * 并把两路都反向。若以后在发射机里重新分配,以 rc_dbg_ch/rc_dbg_sw_*_raw 为准,
 * 同时把 RC_SBUS_INVERT_SW_LEFT/RIGHT 改回 0。
 *
 * C(SC) 必须先在发射机的【通用功能】→【通道设置】里分配给 CH7:
 *   CH7 辅助2 -> 控制 SC -> C
 * 这样 A=CH5、D=CH6、C=CH7,和下面三个宏一一对应。
 *
 * LD/RD 也必须在【通道设置】里分别分配到 CH3/CH4:
 *   LD 在匀速档控制云台目标速度,在变速档控制加速度;RD 控制底盘速度。
 * 这样 CH1 的 J1 右摇杆水平仍可继续用于底盘转弯。
 *
 * 不要只照着"常见固定翼 CH5/CH6/CH7"去猜:ET08A 的默认固定翼模型里,
 * CH5 是 SD,而 CH6/CH7 并没有分配给 SA/SC;本车实际又把 A/D 分配成了 CH5/CH6。
 * 通道没分配时可能输出中位或端点,固件就会把"没接开关"误判成上/下档。
 *
 * 核对方法:Ozone 的 watch 里看 rc_dbg_ch[0..15](原始值 0~2047),一次只动一个
 * 摇杆/开关,哪个下标跟着变,就把对应宏改成那个下标(0 = 第1通道)。 */
#define RC_SBUS_CH_LD       2 /* CH3: LD 旋钮(云台速度目标) */
#define RC_SBUS_CH_RD       3 /* CH4: RD 旋钮(底盘速度目标) */
#define RC_SBUS_CH_RIGHT_H  0 /* CH1: J1 右摇杆水平(底盘转弯) */
#define RC_SBUS_CH_RIGHT_V  1 /* CH2: 右摇杆竖直(底盘前后) */
#define RC_SBUS_CH_SW_RIGHT 5 /* CH6: 实测为 D(右两档开关) */
#define RC_SBUS_CH_SW_LEFT  4 /* CH5: 实测为 A(左两档开关) */
#define RC_SBUS_CH_DIAL     6 /* CH7: 需在发射机里分配给 SC -> C(三档开关) */

/* 如果 ET08A 当前选的是"多旋翼 + 摇杆模式1",说明书里 CH2=J2、CH3=J3,
 * 那么左右摇杆的通道号需要重新核对;LD/RD 仍按实际分配填写。
 * 开关/接收机端口同样按实际分配核对,不要只照通道号猜。 */

#if (RC_SBUS_CH_RIGHT_H >= RC_SBUS_CH_NUM) || (RC_SBUS_CH_RIGHT_V >= RC_SBUS_CH_NUM) ||    \
    (RC_SBUS_CH_DIAL >= RC_SBUS_CH_NUM) || (RC_SBUS_CH_SW_LEFT >= RC_SBUS_CH_NUM) ||     \
    (RC_SBUS_CH_SW_RIGHT >= RC_SBUS_CH_NUM) || (RC_SBUS_CH_LD >= RC_SBUS_CH_NUM) ||      \
    (RC_SBUS_CH_RD >= RC_SBUS_CH_NUM)
#error "SBUS channel mapping out of range: check RC_SBUS_CH_* in remote_control.h"
#endif

/* 开关的判定阈值:本车 ET08A 的通道端点约为 352 / 1024 / 1696。
 * A/D 是两档开关,固件只接受接近两端的值;如果该通道实际落在中位(未分配/接错),
 * 会返回 0(无效)并走急停,不会拿一个"猜出来的上档"去驱动电机。
 * 实测"上/下"方向反了不用改这里:去发射机【通用功能】→【正反设置】把该通道反转即可。 */
#define RC_SBUS_SW_TH_LOW  592  /* 低于它 = 下档 */
#define RC_SBUS_SW_TH_HIGH 1392 /* 高于它 = 上档 */

/* 通道方向:上面两个阈值假设"上档 = 高值、下档 = 低值"。实测发现"物理朝下却被读成上"时,
 * 有两种修法,任选其一:
 *   1) 发射机侧(推荐,不用改代码):【通用功能】→【正反设置】把 A 所在通道反转;
 *   2) 代码侧:把下面这个宏改成 1(不动发射机设置也能生效,发射机换机型/复位也不会丢)。
 * C(旋钮/三档)那一路同理,只是它是模拟量,反转 = 值取负(上↔下互换)。 */
/* 实机:A/D 都是"上=低、下=高",与上面的阈值假设相反,所以两路都反向。 */
#define RC_SBUS_INVERT_SW_LEFT  1
#define RC_SBUS_INVERT_SW_RIGHT 1
/* C/CH7 实机是"下=高值、上=低值",所以这里反向,让下档回到 -660(停)、上档到 +660(来回)。 */
#define RC_SBUS_INVERT_DIAL     1
/* LD/RD 旋钮若实际方向与期望相反,把对应宏改成 1(0 ↔ 满量程互换)。 */
#define RC_SBUS_INVERT_LD       0
#define RC_SBUS_INVERT_RD       0

/* ================= 调试镜像(给 Ozone 的 watch 窗口) =================
 * 都是全局变量,直接敲名字加进 watch 窗口就能看,不用去动模块层的静态变量 */
typedef enum
{
    RC_PROTO_UNKNOWN = 0, /* 还没认出来 */
    RC_PROTO_DBUS    = 1,
    RC_PROTO_SBUS    = 2,
} RC_Protocol_e;

extern volatile int16_t rc_dbg_ch[RC_SBUS_CH_NUM]; /* SBUS 每通道原始值(0~2047) —— 核对通道分配用 */
extern volatile uint8_t rc_dbg_protocol;           /* 认到的协议:0=还没认出来 1=DBUS 2=SBUS */
extern volatile uint32_t rc_dbg_dbus_frames;       /* 累计收到多少帧 DBUS */
extern volatile uint32_t rc_dbg_sbus_frames;       /* 累计收到多少帧 SBUS */
extern volatile uint32_t rc_dbg_bad_frames;        /* SBUS 模式下丢掉的异常帧(半包/干扰) */
extern volatile int16_t rc_dbg_sw_left_raw;        /* A/CH5 原始值(0~2047) */
extern volatile int16_t rc_dbg_sw_right_raw;       /* D/CH6 原始值(0~2047) */
extern volatile int16_t rc_dbg_dial_raw;           /* C/CH7 原始值(0~2047) */
extern volatile int16_t rc_dbg_ld_raw;             /* LD/CH1 原始值(0~2047) */
extern volatile int16_t rc_dbg_rd_raw;             /* RD/CH8 原始值(0~2047) */
extern volatile uint8_t rc_dbg_sbus_flags;         /* SBUS 第24字节:bit2=丢帧 bit3=失控保护 */
/* 在 Ozone 的 watch 窗口里把这个置 1,固件会把当前 16 个通道的原始值(0~2047)通过 RTT 打出来,
 * 打完自己清 0 —— 核对"哪个摇杆/开关落在哪个通道"时不用在 watch 里一个个抄 16 个数。 */
extern volatile uint8_t rc_dbg_ch_dump_req;

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度
typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t dial;      // 侧边拨轮
        int16_t knob_ld;   // LD 旋钮:0~660,对应旋钮最小~最大行程
        int16_t knob_rd;   // RD 旋钮:0~660,对应旋钮最小~最大行程

        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
    uint8_t rc_update_flag;
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

#endif
