#include "robot_cmd.h"

#include "robot_def.h"
#include "remote_control.h"
#include "message_center.h"
#include "user_lib.h"
#include "bsp_dwt.h" // 右上档线性扫速需要 dt

/* 遥控器摇杆的满量程;DBUS摇杆范围是-660~660,模块层已经把|v|<10的死区滤掉了 */
#define RC_STICK_FULL 660.0f

static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回
static Publisher_t *gimbal_cmd_pub;       // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 要发给云台的控制信息

/* "匀速档"(A下 D上)的目标速度:初值是 YAW_SPEED_ROTATE,之后用右摇杆上/下"拨一下"按
 * YAW_SPEED_STEP 加减,并限幅在 [YAW_SPEED_MIN, YAW_SPEED_MAX]。
 * 故意不加 static:这样可以在 Ozone 的 watch 窗口里直接写一个速度做阶跃响应(调速度环用)。 */
float yaw_mid_speed_ref = YAW_SPEED_ROTATE;
/* 摇杆要先回到中间区,才允许下一次加减(= 拨一下动一次,按住不会连加) */
static uint8_t stick_armed = 1;

/* 右上档:目标速度在 [YAW_SPEED_SWEEP_MIN, YAW_SPEED_SWEEP_MAX] 之间线性来回摆动 */
static float sweep_speed = YAW_SPEED_SWEEP_MIN; // 当前扫到的速度
static float sweep_dir   = 1.0f;          // +1:朝 SWEEP_MAX 扫, -1:朝 SWEEP_MIN 扫

static uint8_t  last_sw_left  = 0xFF; // 上一次的左拨杆(=A)位置,用于换档时复位
static uint8_t  last_sw_right = 0xFF; // 上一次的右拨杆(=D)位置
static uint32_t cmd_dwt_cnt   = 0;    // DWT 计时(算 dt 用)

/* ================= 无遥控器调试档 =================
 * 遥控器不在手上时,把 yaw_debug_enable 置 1(Ozone 的 watch 窗口里直接改),就不去读遥控器,
 * 而是用下面这几个变量模拟拨杆/摇杆:
 *   yaw_dbg_a_up  : A 拨杆(1=上, 0=下)
 *   yaw_dbg_d_up  : D 拨杆(1=上, 0=下)   —— 只想让它匀速转: a=0, d=1
 *   yaw_dbg_stick : 右摇杆竖直值(-660~660): 先给 >400 再回 0 = "往上拨一下"(+一个步长),反之为减速
 * 遥控器到手后把 yaw_debug_enable 改回 0 即可,不用重新编译。
 * 调"匀速转动"时:yaw_debug_enable=1, yaw_dbg_a_up=0, yaw_dbg_d_up=1,
 * 然后直接在 watch 里改 yaw_mid_speed_ref(例如 50/100/200)看阶跃响应。 */
volatile uint8_t yaw_debug_enable = 0;
volatile uint8_t yaw_dbg_a_up     = 0;
volatile uint8_t yaw_dbg_d_up     = 1;
volatile int16_t yaw_dbg_stick    = 0;

/**
 * @brief 急停:云台不给电流
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.mode         = YAW_ZERO_FORCE;
    gimbal_cmd_send.target_speed = 0.0f;
}

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // C板上遥控器接收机接在USART3

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    DWT_GetDeltaT(&cmd_dwt_cnt); // 清一次计时,避免第一次算出的 dt 特别大
}

void RobotCMDTask()
{
    float dt = DWT_GetDeltaT(&cmd_dwt_cnt); // [s],扫速用
    uint8_t sw_left, sw_right, a_up, d_up;
    float stick;

    if (yaw_debug_enable) {
        /* 调试档:用 watch 里的变量代替遥控器 */
        a_up     = yaw_dbg_a_up ? 1 : 0;
        d_up     = yaw_dbg_d_up ? 1 : 0;
        stick    = (float)yaw_dbg_stick;
        sw_left  = a_up ? RC_SW_UP : RC_SW_DOWN;
        sw_right = d_up ? RC_SW_UP : RC_SW_DOWN;
    } else {
        /* 遥控器掉线时数据会被清零,此时两个开关都是0,也按急停处理 */
        if (!RemoteControlIsOnline() ||
            (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)) {
            EmergencyHandler();
            PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
            return;
        }
        sw_left  = rc_data[TEMP].rc.switch_left;
        sw_right = rc_data[TEMP].rc.switch_right;
        a_up     = switch_is_up(sw_left) ? 1 : 0;
        d_up     = switch_is_up(sw_right) ? 1 : 0;
        stick    = (float)rc_data[TEMP].rc.rocker_r1;
    }

    /* 拨杆语义(A=左拨杆/SA, D=右拨杆/SD, 两档):
     *   A下 D下 -> 停机(急停)      A下 D上 -> 定速旋转(原右中档)
     *   A上 D下 -> 线性扫速(原右上档) A上 D上 -> 锁角度 */

    /* A、D 双下:停机 */
    if (!a_up && !d_up) {
        EmergencyHandler();
        PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
        return;
    }

    /* 拨杆位置变化:复位"拨动"的边沿状态;进扫速档时让扫速从下限重新开始 */
    if (sw_left != last_sw_left || sw_right != last_sw_right) {
        last_sw_left  = sw_left;
        last_sw_right = sw_right;
        stick_armed   = 1;
        if (a_up && !d_up) { // 刚进扫速档
            sweep_speed = YAW_SPEED_SWEEP_MIN;
            sweep_dir   = 1.0f;
        }
    }

    if (a_up && d_up) {
        /* A上 D上:锁角度(锁住进入时的角度,由 gimbal 侧捕获当前角度) */
        gimbal_cmd_send.mode         = YAW_ANGLE_LOCK;
        gimbal_cmd_send.target_speed = 0.0f;
    } else if (a_up) {
        /* A上 D下:目标速度在 [YAW_SPEED_SWEEP_MIN, YAW_SPEED_SWEEP_MAX] 之间线性来回摆动 */
        gimbal_cmd_send.mode = YAW_SPEED_CTRL;
        sweep_speed += YAW_SPEED_SWEEP_RATE * sweep_dir * dt;
        if (sweep_speed >= YAW_SPEED_SWEEP_MAX) { // 扫到上限就掉头
            sweep_speed = YAW_SPEED_SWEEP_MAX;
            sweep_dir   = -1.0f;
        } else if (sweep_speed <= YAW_SPEED_SWEEP_MIN) { // 扫到下限就掉头
            sweep_speed = YAW_SPEED_SWEEP_MIN;
            sweep_dir   = 1.0f;
        }
        gimbal_cmd_send.target_speed = sweep_speed;
    } else {
        /* A下 D上:按 yaw_mid_speed_ref 匀速旋转,右摇杆"拨一下"加减一个步长 */
        gimbal_cmd_send.mode = YAW_SPEED_CTRL;
        float r1             = stick;
        if (stick_armed) {
            if (r1 > YAW_STICK_UP_TH) { // 往上拨一下:加速
                yaw_mid_speed_ref += YAW_SPEED_STEP;
                stick_armed = 0;
            } else if (r1 < YAW_STICK_DOWN_TH) { // 往下拨一下:减速
                yaw_mid_speed_ref -= YAW_SPEED_STEP;
                stick_armed = 0;
            }
        } else if (r1 < YAW_STICK_MID_ZONE && r1 > -YAW_STICK_MID_ZONE) {
            stick_armed = 1; // 摇杆回中,允许下一次加减
        }
        yaw_mid_speed_ref = float_constrain(yaw_mid_speed_ref, YAW_SPEED_MIN, YAW_SPEED_MAX); // 最大/最小限幅
        gimbal_cmd_send.target_speed = yaw_mid_speed_ref;
    }

    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
