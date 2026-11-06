# robot_cmd

把遥控器(DBUS,接在USART3)的操作翻译成**云台**和**底盘**的控制指令,在1kHz的任务里运行。
这是整车**唯一**读遥控器的地方:云台和底盘都在这里解析,再分别通过`gimbal_cmd`/`chassis_cmd`话题发布出去。

## 拨杆语义(A = 左拨杆/SA,D = 右拨杆/SD,两档)

> 本车 ET08A 实测映射是 **A=CH5、D=CH6、C=CH7**,并且 A/D 都是
> **上=低值、下=高值**,所以代码里两路都做了反向。具体表见 `modules/remote/remote.md`。
> A/D 任一路读到中位(未分配/接错)都会走急停,
> 不会把"中位"猜成下档继续跑。

| A | D | 云台做什么 | 发给gimbal的模式 |
| --- | --- | --- | --- |
| 下 | 下 | 急停,输出电流置零(遥控器掉线时也走这条) | `YAW_ZERO_FORCE` |
| 下 | 上 | 匀速旋转:目标由 LD/CH3 映射到 `YAW_SPEED_LD_MIN_RAD~YAW_SPEED_LD_MAX_RAD` | `YAW_SPEED_CTRL` |
| 上 | 下 | 没有命令 = **卸电**:直接断电流,云台可以被手自由推动(和"下下"的急停一样;以前这里是"速度环给0通电顶住") | `YAW_ZERO_FORCE` |
| 上 | 上 | 变速转动:LD 映射为 `0~10rad/s²` 正向加速度,执行 `0→+7→0→+7rad/s` | `YAW_SPEED_CTRL` |

> 匀速档仍由 LD 从 `0~660` 映射速度,负半程钳位为 `0`。
> 变速档把 LD 全行程映射为 `0~10rad/s²`,只使用正向加速度,不进入负方向。
> 因此 LD 决定到达 `±7rad/s` 限幅点、以及从限幅点减回0的快慢。
> 目标速度到0时仍保持速度闭环和电机使能,不卸电。
>
> **"双下"和"A上D下"的区别**:双下是整车急停(`CHASSIS_STOP_ON_AD_ESTOP=1`),
> 云台和底盘都断电;A上D下只让云台卸电,底盘仍按 C 拨杆和左摇杆的控制走。

> CH1 右摇杆水平属于底盘,C 中档用于差速转弯;它不参与云台目标角速度。
>
> 定速档也是速度环(和变速档同一个模式),起步为什么不冲、转速静差怎么消,见`application/gimbal/gimbal.md`。

## 拨杆语义(底盘:C拨杆 + 左摇杆,三档)

| C | 底盘做什么 | 发给chassis的模式 |
| --- | --- | --- |
| 下 | 急停(断电):两个轮子电流置零,和云台的急停一样 | `CHASSIS_ZERO_FORCE_CTRL` |
| 中 | 右摇杆竖直 = RD 映射的 `0~1.0m/s` 前后走;**右摇杆水平** = 差速原地转弯(幅值同样跟随 RD);**两轴回中 = 保持底盘角度** | `CHASSIS_STICK_CTRL` |
| 上 | 以固定 `1.0m/s` 来回走 `CHASSIS_SWING_DISTANCE_M`;右摇杆水平叠加最大约 `0.05m/s` 的左右微调 | `CHASSIS_SWING_CTRL` |

> C中档前后和转弯速度由 RD/CH4 映射到 `CHASSIS_SPEED_RD_MIN_MPS~CHASSIS_SPEED_RD_MAX_MPS`
> (当前 `0.1~0.6m/s`)。C上档主速度不跟随 RD,固定为 `CHASSIS_SWING_SPEED_MPS=1.0m/s`。
> 转弯为什么用左右轮差速、来回距离怎么量,见`application/chassis/chassis.md`。
> 转弯死区由 `CHASSIS_TURN_DEAD_ZONE` 控制(当前 180);超过死区后按摇杆行程比例加到
> `chassis_turn_speed`,不会一过门槛就满转弯。C上档的微调最大值为
> `CHASSIS_SWING_MICRO_TURN_SPEED_MPS=0.05m/s`。
>
> **"C"是独立的第三路拨杆**(不是A也不是D):它从遥控器的哪一路取,由 `chassis_c_source` 决定
> (0=不从遥控器取=底盘一直急停,1=拨轮/CH7,2/3=左右开关)。默认是 **1**,
> 上电即可用遥控器控制;为什么会这么绕、以及怎么确认C落在哪一路,
> 见 `application/chassis/chassis.md` 的"C到底从哪一路进来"。
>
> A下D下(急停档)现在**云台和底盘一起断电**(整车急停),由`robot_def.h`里的`CHASSIS_STOP_ON_AD_ESTOP`控制(默认1)。
>
> **遥控器掉线时**:数据会被清成0,C拨杆被当成"下档",所以底盘会**急停**(不会因为丢帧继续跑)。

## 没有遥控器时怎么调

把`yaw_debug_enable`置1(Ozone的watch窗口里直接改,不用重新编译),就不再读遥控器,改用下面几个变量模拟:

| 变量 | 作用 |
| --- | --- |
| `yaw_dbg_a_up` | 代替A拨杆(1=上, 0=下) |
| `yaw_dbg_d_up` | 代替D拨杆(1=上, 0=下) |
| `yaw_dbg_stick` | 右摇杆竖直镜像;当前不参与云台目标 |

四个档对应的拨杆:急停 `a=0 d=0`、定速 `a=0 d=1`、卸电(暂称"停云台") `a=1 d=0`、变速 `a=1 d=1`。
> 注:`a=0 d=0` 和 `a=1 d=0` 现在都是**卸电(断电流)**;区别只是前者被当成"遥控器的急停档"、后者是"没有命令的停"。

只想让它匀速转:`yaw_debug_enable=1`、`yaw_dbg_a_up=0`、`yaw_dbg_d_up=1`。
匀速档目标由 LD 旋钮映射到 `YAW_SPEED_LD_MIN_RAD~YAW_SPEED_LD_MAX_RAD`(当前 `0~20rad/s`)。

### 模拟云台的完整参数(和当前 `robot_def.h` 对齐)

| 变量/参数 | 当前值 | 说明 |
| --- | --- | --- |
| `yaw_debug_enable` | 1 | 打开模拟;设回 0 就恢复真实遥控 |
| `chassis_debug_enable` | 0 | 调云台时不要让底盘也进模拟 |
| `yaw_dbg_stick` | 0 | 右摇杆镜像;当前不参与云台目标 |
| `gimbal_ld_target_speed` | 0~1145.9 | LD 映射后的云台目标 [°/s] |
| `chassis_rd_target_speed` | 0~1.0 | RD 映射后的底盘目标 [m/s] |
| `YAW_SPEED_SLEW` | 1200 | 设定值变化率 [°/s²];0→360 约 0.3s |
| `YAW_SPEED_MIN` / `YAW_SPEED_MAX` | -1800 / +1800 | 速度限幅 [°/s] |
| `YAW_SPEED_SWEEP_MAX_RAD` | 7 | 变速档速度绝对值上限 [rad/s] |
| `YAW_SWEEP_ACCEL_MIN_RAD` / `YAW_SWEEP_ACCEL_MAX_RAD` | 0 / 10 | LD 映射后的正向加速度范围 [rad/s²] |
| `gimbal_yaw_speed_pid->Kp` | 10 | 速度环比例(当前源码值;Ozone 里可能改成 8.5,以在线值为准) |
| `gimbal_yaw_speed_pid->Ki` | 1.5 | 速度环积分(定速档斜坡结束后可直接在 Ozone 中在线修改) |
| `gimbal_yaw_speed_pid->Kd` | 0 | 速度环微分 |
| `gimbal_yaw_speed_pid->DeadBand` | 0 | 实测 30 会造成吱吱声;当前用 0 |
| `yaw_sweep_kp` / `yaw_sweep_ki` | 6.1 / 2.0 | 变速档专用速度环参数,可在 Ozone 里直接改 |
| `yaw_sweep_kp_effective` / `yaw_sweep_ki_effective` | 实际生效值 | 当前不做限幅,等于你写入的 `yaw_sweep_kp/ki` |
| `yaw_constant_kp` / `yaw_constant_ki` | 捕获值 | 切到变速档时保存的定速档参数,切回来恢复 |
| `gimbal_active_profile` | 0/1 | 当前使用的参数档:0=定速,1=变速 |
| `yaw_accel_ff_used` | 0 / 随变速档变化 | 斜坡前的惯量前馈;匀速档恒为0,变速档来自 `yaw_sweep_accel_ff_gain` |
| `yaw_constant_friction_ff` | 0 / ±匀速档前馈幅值 | 匀速档方向静摩擦前馈;变速档恒为0 |
| `yaw_sweep_accel_ff_gain` | 8.0 | 变速档专用惯量前馈增益,独立在线调 |
| `yaw_unbalance_ff_amp` / `_phase` | 0 / 246 | 变速档一圈一次扰动补偿;当前关闭 |
| `yaw_ff_ramp_rate` | 5 | 变速档前馈启动斜坡 [1/s];约 0.2s 到满 |

模拟时看这几个值就能和真实遥控逐项对齐:

| 物理档 | `yaw_dbg_a_up` | `yaw_dbg_d_up` | 期望 `rc_dbg_yaw_gear` | 期望 `gimbal_yaw_mode` | 目标速度 |
| --- | --- | --- | --- | --- | --- |
| A下D下 | 0 | 0 | 0 | 0 | 0(急停) |
| A下D上 | 0 | 1 | 1 | 1 | LD 映射值(0~20rad/s) |
| A上D下 | 1 | 0 | 2 | 0 | 0(卸电) |
| A上D上 | 1 | 1 | 3 | 1 | `sweep_speed`(`0→+7→0→+7rad/s`,加速度由 LD 控制) |

> `rc_dbg_yaw_a_up` / `rc_dbg_yaw_d_up` / `rc_dbg_yaw_stick_used` 是**控制实际采用**的值。
> 真实遥控时先记录这三个值,再在模拟模式里照填,就能判断"差异是输入解码还是速度环"。

### 变速档的"设定速度"在Ozone里怎么看

变速档的设定值是加速度控制的正/负三角波,watch里加这几个变量就能把整条链路都看清(都是非static的全局变量,
直接把名字敲进 Ozone 的 watch/符号窗口即可,数值每拍都在刷新):

| 变量 | 是什么 | 单位/取值 | 在哪个文件 |
| --- | --- | --- | --- |
| `sweep_speed` | 变速档当前目标速度(=发给云台的`target_speed`) | [°/s] | robot_cmd.c |
| `sweep_accel` | 变速档实际积分使用的带符号加速度 | [°/s²] | robot_cmd.c |
| `sweep_dir` | 当前半程是朝限幅点还是返回0 | ±1 | robot_cmd.c |
| `gimbal_sweep_accel_rad` | LD 映射后的正向加速度 | [rad/s²] | robot_cmd.c |
| `gimbal_ld_target_speed` | LD 映射后的云台匀速目标 | [°/s] | robot_cmd.c |
| `chassis_rd_target_speed` | RD 映射后的底盘速度目标 | [m/s] | robot_cmd.c |
| `rc_dbg_ld_percent` / `rc_dbg_rd_percent` | LD/RD 当前行程百分比 | % | robot_cmd.c |
| `gimbal_yaw_speed_setpoint` | 限幅之后**速度环真正拿到**的设定值 | [°/s] | gimbal.c |
| `gimbal_yaw_speed` | 实测角速度(速度环的反馈) | [°/s] | gimbal.c |
| `gimbal_yaw_pid_out` | 速度环输出,即电流指令 | ±16384 | gimbal.c |

两个"设定值"的区别:`sweep_speed` 是**想让它跑多少**;`gimbal_yaw_speed_setpoint` 是**它现在被要求跑多少** ——
后者要先按 `YAW_SPEED_SLEW` 从当前值爬到前者,所以换向的瞬间两者会差一小段(这正是"起步不冲"的来源)。

想调变速档就看这一对:`gimbal_yaw_speed` 应该**贴着** `gimbal_yaw_speed_setpoint` 走;
明显滞后就是 Kp/Ki 偏小或前馈不够,来回摆/发抖就是 Kp 或 LD 给的加速度过大。

底盘有**独立**的一个调试档开关(和云台的`yaw_debug_enable`各管各的):

| 变量 | 作用 |
| --- | --- |
| `chassis_debug_enable` | 1 = 底盘改用下面几个变量模拟遥控器(遥控器那一段代码依然在,只是被跳过) |
| `chassis_dbg_c` | 代替C拨杆:0=下(停), 1=中(摇杆), 2=上(来回5m) |
| `chassis_dbg_stick_v` | 代替右摇杆竖直值(-660~660),>0=往前推 |
| `chassis_dbg_stick_h` | 代替右摇杆水平值(-660~660),>0=往右推 |
| `chassis_stick_speed` / `chassis_turn_speed` | C中档的定速值[m/s],直接写数值就能做阶跃(调速度环用) |
| `chassis_c_source` / `chassis_c_raw` | C从遥控器的哪一路取(默认1=拨轮/CH7)/ 那一路的原始值 |
| `chassis_forward_speed_mps` / `_kmh` | 左右轮平均后的底盘实际前进速度 [m/s] / [km/h] |
| `rc_dbg_sw_left` / `rc_dbg_sw_right` / `rc_dbg_dial` | 遥控器原始数据的镜像,用来确认哪个拨杆占哪一路 |
| `rc_dbg_yaw_gear` | 云台实际档位:`0`=双下急停,`1`=定速,`2`=A上D下卸电,`3`=变速,`0xFF`=A/D有无效值(已强制急停) |
| `rc_dbg_sw_left_raw` / `rc_dbg_sw_right_raw` / `rc_dbg_dial_raw` | A/D/C 所在通道的 SBUS 原始值(0~2047),查通道和方向用 |
| `rc_dbg_yaw_a_up` / `rc_dbg_yaw_d_up` | 云台控制**实际采用**的 A/D 值;模拟模式下分别等于 `yaw_dbg_a_up` / `yaw_dbg_d_up` |
| `rc_dbg_yaw_stick_used` | 竖直摇杆镜像;当前不参与云台目标 |

让底盘以1m/s往前走:`chassis_debug_enable=1`、`chassis_dbg_c=1`、`chassis_dbg_stick_v=660`。

## 消息

云台的映射结果通过`message_center`的`gimbal_cmd`话题发布,由gimbal应用订阅:

```c
typedef struct {
    Yaw_Mode_e mode;          // 0=急停/卸电 1=速度环(定速/变速)
    Gimbal_Profile_e profile; // 0=定速参数 1=变速专用参数
    float target_speed;       // [°/s] 目标角速度(只有定速/变速档有效)
} Gimbal_Ctrl_Cmd_s;
```

> `A上D上`(变速)时 `profile=GIMBAL_PROFILE_SWEEP`,云台使用 `yaw_sweep_kp/ki`;
> 其余档 `profile=GIMBAL_PROFILE_CONSTANT`,继续使用定速档的 PID。

底盘的走`chassis_cmd`话题,由chassis应用订阅:

```c
typedef struct {
    Chassis_Ctrl_Mode_e mode; // 0=急停 1=刹车 2=摇杆 3=来回5m
    float vx;                 // [m/s] 前后线速度(只有摇杆档用)
    float turn_speed;         // [m/s] 转弯速度,>0左转<0右转(只有摇杆档用)
} Chassis_Ctrl_Cmd_s;
```

这样做的好处:遥控器只被解析一次;以后再加发射机构等,各自订阅自己的话题,应用之间不互相包含。

> 摇杆方向:右摇杆向上 = `rocker_r1`为正 = 云台正转;若觉得方向相反,
> 改`robot_def.h`里的`YAW_MOTOR_REVERSE`(**只改这一处**,理由见`gimbal.md`的最后一节)。
