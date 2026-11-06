# chassis(底盘)

两个轮子的差速底盘:C中档用右摇杆竖直控制前后、右摇杆水平原地转弯,C上档自动来回,
C下档停底盘。C中档摇杆回中后会保持底盘角度;C上档主速度为固定 1m/s,
右摇杆水平只在左右方向做小幅修正。

## 硬件

- 两个**M3508**,各配一个**C620电调**,都挂在**CAN1**上,电调上的ID(闪灯次数)分别是 **1(左轮)** 和 **2(右轮)**。
  - 控制报文:`0x200`(ID 1~4 共用一帧);反馈报文:`0x201` / `0x202`(即 `0x200 + ID`)。
  - **注意减速箱**:轮子直接装在**输出轴**上(输出轴和轮子之间没有额外减速),但 M3508 **自带行星减速箱**
    (减速比 `CHASSIS_GEAR_RATIO` = 3591/187 ≈ **19.2**),而编码器装在**转子**上 ——
    所以**电调报出来的角度和转速是转子的**,不是轮子的,换算到轮子要除以这个减速比。
    (框架老代码里也写着:`REDUCTION_RATIO_WHEEL 19.0f`,"编码器量测的是转子的速度而不是输出轴的速度"。)
    这层换算被关在 `chassis.c` 的 `MPS_2_APS` / `APS_2_MPS` 两个宏里,所以**app层的参数全是轮子的物理值**
    (`chassis_stick_speed` 写 1.0 就真的是 1 m/s)。
- 轮胎直径 **0.101m**(`CHASSIS_WHEEL_DIAMETER_M`),周长 = π×直径 ≈ 0.317m。所有"多少m/s"的设定都是**轮子的线速度**,
  模块层要的是角速度(°/s),`chassis.c` 里用 `MPS_2_APS` 换算。
- 底盘没有传感器测真实位移,来回档的"走了多远"是用**两个电机编码器的累计角度**算出来的(见下文)。

## 三个档(C拨杆)

| C拨杆 | 底盘做什么 | 发给chassis的模式 |
| --- | --- | --- |
| 下 | **急停(断电)**:两个电机电流置零,和云台的急停一样(轮子会自由滑行) | `CHASSIS_ZERO_FORCE_CTRL` |
| 中 | 右摇杆**上下**=按 RD 映射速度前后走;右摇杆**左右**=差速原地转弯(幅值跟随 RD);**两轴回中=角度保持** | `CHASSIS_STICK_CTRL` |
| 上 | 以固定 `CHASSIS_SWING_SPEED_MPS=1.0m/s` 在 0～`CHASSIS_SWING_DISTANCE_M` 之间来回;右摇杆水平做最大约 `0.05m/s` 的方向微调 | `CHASSIS_SWING_CTRL` |

> 遥控器的 **A下D下(云台急停档)现在也会把底盘一起断电**(整车急停),由 `robot_def.h` 里的
> `CHASSIS_STOP_ON_AD_ESTOP` 控制(默认 **1 = 也停**)。
>
> 另外还留了 `CHASSIS_BRAKE_CTRL`(速度环目标给0,主动刹停并顶住),现在没用到。
> 如果哪天觉得"急停后轮子还在滑行"不舒服,把 C下档那行改成 `CHASSIS_BRAKE_CTRL` 就是刹停。

### C中档:转弯为什么用差速

摇杆往左推 = 左转:右轮加速、左轮减速;往右推则相反。
纯转弯时左右轮反向旋转,底盘原地转;前进+转弯时平均前进速度仍是 `vx`,
转弯量只由 `turn_speed` 决定,不会再叠加到前进速度上。

代码里在 `ChassisStickKinematics()`:

| 输入 | 左轮 | 右轮 |
| --- | --- | --- |
| 只推前后(`vx`) | `vx` | `vx` |
| 左转(`turn_speed>0`) | `vx - turn_speed` | `vx + turn_speed` |
| 右转(`turn_speed<0`) | `vx - turn_speed` | `vx + turn_speed` |
| 不推/回中 | 角度保持闭环 | 角度保持闭环 |

斜着推(又推前又推左)时,两轮差速保持 `2×turn_speed`,旋转速度不会随前进速度变大。

摇杆回中后,底盘用左右轮编码器的行程差估计偏航量,并给两轮一个反向的小速度把朝向拉回。
如果轮子完全没有相对转动,设定值就是 0;只有底盘被外力或地面扰动转偏时才进行修正。
角度保持现在优先使用 C 板 BMI088 的 `YawTotalAngle` 和 `Gyro[Z]`:

- 航向角误差负责把车头拉回目标方向;
- `Gyro[Z]` 负责阻尼,能在轮胎刚开始打滑、车头刚开始偏转时立即产生反向差速。
- 只有 IMU 不可用时才退回原来基于左右轮编码器行程差的角度保持。

直线前进、松杆停车、自动折返以及折返零速锁角都会使用这套陀螺仪航向控制。

### C上档:来回5m是怎么做的

1. 进档那一刻把两个电机当前的累计角度记下来当**起点**(`swing_angle_origin`),方向设为"前进";
2. 每拍算一次位移:`(当前平均角度 - 起点) × 周长/360 × CHASSIS_SWING_DIRECTION_SIGN`,
   这就是逻辑上的前进里程(`chassis_distance_m`,逻辑向前为正);
3. 走满 `chassis_swing_distance`(当前默认2m)就掉头往回走,退回起点(≤0)再掉头,如此往复。
   `CHASSIS_SWING_DIRECTION_SIGN=-1` 时,第一次给正逻辑速度会先走物理上的“前进”方向。
4. 距离最近端点 `0.4m` 内按距离线性降速,折返后再用同样 `0.4m` 距离线性升速。
   最低速度限制为 `0.05m/s`,避免速度落入死区后无法重新启动。
5. 右摇杆水平超过 `CHASSIS_TURN_DEAD_ZONE` 后,叠加最大
   `CHASSIS_SWING_MICRO_TURN_SPEED_MPS=0.05m/s` 的差速方向修正;不会改变自动折返逻辑。

用编码器而不是"定时多久"的原因:定时只对速度环跟得完美的情况成立,一旦有打滑、电池电压下降、负载变化,
实际距离就会和设的5m差很远;编码器量出来的是真的走了多远。

| 想改什么 | 在哪里改 |
| --- | --- |
| 来回的**主速度** | `robot_def.h` 的 `CHASSIS_SWING_SPEED_MPS`(当前 1.0m/s,也可运行时改 `chassis_swing_speed`) |
| 来回的**方向微调** | `CHASSIS_SWING_MICRO_TURN_SPEED_MPS`(当前 0.05m/s,也可运行时改 `chassis_swing_micro_turn_speed`) |
| 来回的**距离** | `robot_def.h` 的 `CHASSIS_SWING_DISTANCE_M`(也可以运行时改 `chassis_swing_distance`) |
| 端点斜坡距离 | `robot_def.h` 的 `CHASSIS_SWING_RAMP_DISTANCE_M`(也可以运行时改 `chassis_swing_ramp_distance`) |
| 折返最低速度 | `robot_def.h` 的 `CHASSIS_SWING_MIN_SPEED_MPS`(也可以运行时改 `chassis_swing_min_speed`) |

## 和云台的关系

底盘和云台是**两个互不影响的应用**:

- 底盘只认 **C拨杆 + 右摇杆**(C中控制前后/转弯,C上控制方向微调),云台只认 **A/D拨杆 + 旋钮**;
- 它们各自订阅自己的话题(`chassis_cmd` / `gimbal_cmd`),`robot_cmd` 是整车唯一读遥控器的地方;
- "A下D下"(急停)两边都会响应:云台断电、底盘也断电(整车急停);而 C下档只停底盘。

## 工作流程

`ChassisInit()`(实时系统启动前):

1. 注册左(ID=`CHASSIS_MOTOR_ID_LEFT`)、右(ID=`CHASSIS_MOTOR_ID_RIGHT`)两个M3508,只用速度环。
2. 保存两个速度环PID的地址(`chassis_left_speed_pid` / `chassis_right_speed_pid`),方便调试时直接改参数。
3. 上电默认 `DJIMotorStop()`,即不给电流。
4. 订阅 `chassis_cmd` 话题(由robot_cmd发布)。

`ChassisTask()`(1kHz):

1. `SubGetMessage()` 取 `chassis_cmd`;没有新消息就沿用上一次的指令。
2. 刷新调试变量(实测轮速、电流指令、模式…),急停时也能看到反馈。
3. 模式变了就做换档处理(进来回档记起点、离开来回档清里程)。
4. 按 `mode` 分支:急停(断电)→两个轮子一起卸电;来回档→`ChassisSwingUpdate()`;摇杆档→运动学解算后下发。
5. 摇杆档有运动指令时逐轮判断(`ChassisSetWheelSpeedOrRelease()`):某个轮子的设定是0就卸电
   (`DJIMotorStop()` + `PIDReset()`),只有真的要动的那一轮才使能;纯转弯时支点轮是自由的。
   (`ChassisRelease()` 是"两个一起卸电",急停档用。)
   摇杆前后和转弯同时回中时例外:两轮先同时以0速目标闭环制动,实测速度都低于
   `chassis_stop_speed_mps` 并连续保持 `chassis_stop_settle_s` 后,才记录朝向并进入角度保持。
   这样不会在车还在滑行时立即锁住一个已经偏掉的角度。
5. 下发的设定值单位永远是"轮子线速度 m/s",由 `ChassisSetWheelSpeed()` 换算成速度环的 °/s。

## 调试(Ozone)

### 一步一步:用Ozone模拟底盘(没有遥控器也能开)

> 前提:程序已经在跑(Ozone能attach上)、**车架空或确认四周没人和线**(轮子离地最安全)。

**第0步 先记住一件事**:底盘默认从 C/CH7 取控制量(`chassis_c_source=1`),
所以遥控器可以直接控制;如果 C 通道没有分配,它会落在中位,底盘会按"摇杆档"处理,
这时请先拨 C 或检查 CH7 的通道分配。

**第1步 打开调试档**:watch里把 `chassis_debug_enable` 从 0 改成 **1**(双击数值直接写)。
这一步之后底盘不再看遥控器,只看下面这几个变量;遥控器那段代码还在,把1改回0就恢复。

**第2步 选档**:改 `chassis_dbg_c`(就是模拟C拨杆的位置):

| `chassis_dbg_c` | 底盘做什么 |
| --- | --- |
| `0` | 下档:急停(断电),轮子自由,推车能推动 |
| `1` | 中档:按 `chassis_dbg_stick_v/h` 走 |
| `2` | 上档:自己来回走 `chassis_swing_distance` |

**第3步 开车(C中档,`chassis_dbg_c=1`)**

| 变量 | 写成 | 效果 |
| --- | --- | --- |
| `chassis_dbg_stick_v` | `660` / `-660` | 前进 / 后退 |
| `chassis_dbg_stick_h` | `660` / `-660` | 右转 / 左转(两轮差速) |
| 两个都回 `0` | `0` | **卸电**:两个轮子直接断电流、轮子自由(不是"通电顶住") |

> 数值必须**超过 `CHASSIS_STICK_DEAD_ZONE`(=100)** 才算"推了摇杆",写50、80都不会动。
> **"设定是0"的轮子一律卸电**——没有命令时两个轮子都自由;差速转弯时两轮一般都有非零速度,
> 会同时通电。若某个轮子的差速结果正好是 0,它仍然会卸电。

**第4步 调速度**:`chassis_stick_speed`(前后)和 `chassis_turn_speed`(转弯)就是那两个"固定速度",
单位 m/s,在watch里直接改、下一拍(1ms)生效。想做阶跃响应:保持 `chassis_dbg_stick_v=660`,
把 `chassis_stick_speed` 从 0 写到 1.0,再看 `chassis_v_left_ref` / `chassis_v_left_meas` 的跟随。

**第5步 来回档(`chassis_dbg_c=2`)**:速度、单程距离用 `chassis_swing_speed` / `chassis_swing_distance` 在线改;
折返斜坡由 `chassis_swing_ramp_distance` 和 `chassis_swing_min_speed` 控制。到端点后先以0速目标
用同一套角度保持控制,两轮都停到 `chassis_stop_speed_mps` 以下后保持 `chassis_swing_zero_hold_s`,
再开始反向。盯 `chassis_distance_m`、`chassis_swing_dir`、`chassis_swing_zero_hold_active`
和 `chassis_v_left_ref`。
架空时轮子空转、编码器照样计程,所以**架空就能验证"走满5m掉头"的逻辑**。改成别的档会把里程清零。

**第6步 调PID**:`chassis_left_speed_pid->Kp` / `->Ki` / `->Kd`、`chassis_right_speed_pid->...`,
和调云台一样在watch里直接改(下一拍生效)。具体步骤见下面"速度环怎么调"。

**常见卡点**

| 现象 | 检查 |
| --- | --- |
| 改了变量车不动 | `chassis_debug_enable` 是不是1?`chassis_dbg_c` 是不是1或2?看 `chassis_mode`(0急停/1刹车/2摇杆/3来回)对不对 |
| `chassis_mode=2` 但 `chassis_v_left_ref=0` | 摇杆值没超过死区(100) |
| 想换回遥控器 | 把 `chassis_debug_enable` 写回0 |
| 复位后参数全变回去了 | 调试器改的是RAM;调好的值要写回 `robot_def.h` 的宏再编译 |

### C到底从哪一路进来(重要)

C是**独立的第三路三档拨杆**,不是A也不是D。但DBUS遥控器一共只有**两路开关**(代码里叫
`switch_left`/`switch_right`),而云台的4个档把它们占满了("变速"就是A上+D上)。
所以底盘**绝不能**从A/D取C —— 一拨变速底盘就会跟着来回走。

DBUS里除了4个摇杆,只剩**拨轮(dial)**这一路是空的。所以C要么被发射机/接收机分到了"拨轮"那一格,
要么就根本没进这一帧(那就要动发射机的通道分配或接线)。

`chassis_c_source` 决定底盘从哪一路取C,默认 **1 = 拨轮(dial)/CH7`:

| 值 | 含义 |
| --- | --- |
| 0 | 不从遥控器取:底盘一直按"下档=急停(断电)"处理 |
| 1 | 拨轮(dial)/CH7:高于`+CHASSIS_C_DIAL_TH`=上档,低于`-CHASSIS_C_DIAL_TH`=下档,之间=中档(默认) |
| 2 / 3 | 左开关 / 右开关(云台也在用,除非云台腾开这一路,否则不要选) |

**怎么确认C实际落在哪一路**(在Ozone里一次只拨一个拨杆):

| 变量 | 看什么 |
| --- | --- |
| `rc_dbg_sw_left` / `rc_dbg_sw_right` | 左/右开关的原始值:1=上, 2=下, 3=中, 0=遥控器没在线 |
| `rc_dbg_dial` | 拨轮的原始值[-660,660] |
| `rc_dbg_stick_r1` | 右摇杆竖直(底盘前后用) |
| `chassis_c_raw` | 当前`chassis_c_source`选中的那一路的原始值 |

把C(SC)从下拨到中再拨到上:如果只有 `rc_dbg_dial` 跟着变,那就把 `chassis_c_source` 改成 **1**,底盘就接上了。
如果**哪个都不变**,说明SC根本没进这一帧,需要另外想办法(见 `robot_cmd.md` 里那几种方案)。

### 底盘自己的调试变量(都非static,watch里直接敲名字)

| 变量 | 含义 |
| --- | --- |
| `chassis_mode` | 当前模式,对应`Chassis_Ctrl_Mode_e`(0急停/1刹车/2摇杆/3来回) |
| `chassis_vx_cmd` / `chassis_turn_cmd` | 当前下发的"前后/转弯"指令 [m/s];C中角度保持时 `vx=0` |
| `chassis_vx_limited_mps` | C中档经过前后加速度限幅后的速度 [m/s] |
| `chassis_angle_lock_active` / `chassis_heading_error_m` | C中档是否正在角度保持,以及左右轮行程差误差 [m] |
| `chassis_yaw_deg` / `chassis_yaw_rate_dps` | BMI088 航向角 [°] 和 Z 轴角速度 [°/s] |
| `chassis_heading_error_deg` | 陀螺仪航向目标误差 [°] |
| `chassis_stop_settling` | C中档松杆后是否正在两轮闭环刹停 |
| `chassis_swing_zero_hold_active` | 自动折返是否正在0速锁角 |
| `chassis_v_left_ref` / `chassis_v_right_ref` | 经过运动学解算后真正下发给两个轮子的设定 [m/s] |
| `chassis_v_left_meas` / `chassis_v_right_meas` | 两个轮子的**实测**线速度 [m/s](速度环的反馈) |
| `chassis_forward_speed_mps` / `chassis_forward_speed_kmh` | 两轮平均后的底盘实际前进速度 [m/s] / [km/h] |
| `chassis_left_pid_out` / `chassis_right_pid_out` | 两个速度环的输出,即电流指令(满量程±16384) |
| `chassis_distance_m` | 来回档从起点算起的位移 [m](前进为正) |
| `chassis_swing_dir` | 来回档:1=正在前进, 0=正在后退(只有来回档有意义) |
| `chassis_swing_speed` / `chassis_swing_distance` / `chassis_swing_ramp_distance` / `chassis_swing_min_speed` | 来回档速度/单程距离/端点斜坡距离/折返最低速度,可以在watch里在线改 |
| `chassis_angle_lock_kp` / `chassis_angle_lock_kd` / `chassis_angle_lock_max_speed` | C中角度保持的比例、阻尼和最大修正速度,可在线调 |
| `chassis_gyro_heading_kp` / `chassis_gyro_heading_kd` | 陀螺仪航向角误差和角速度阻尼增益,可在线调 |
| `chassis_gyro_heading_max_speed` | 陀螺仪航向控制允许的最大差速修正 [m/s] |
| `chassis_gyro_yaw_reverse` | 实测车头左转时 `chassis_yaw_deg` 反而减小就改成1 |
| `chassis_stop_settle_s` / `chassis_stop_speed_mps` | 中档停稳确认时间和速度阈值,可在线调 |
| `chassis_swing_zero_hold_s` | 自动折返到0速后的锁角停留时间,可在线调 |
| `chassis_stick_accel_mps2` | C中档前后加减速上限,快速正反切换时防止轮胎瞬间打滑 |
| `chassis_motor_left` / `chassis_motor_right` | 电机实例,展开看`measure`(速度/电流/编码器/温度) |
| `chassis_left_speed_pid` / `chassis_right_speed_pid` | 左/右轮的速度环PID实例,直接改 `->Kp` `->Ki` `->Kd`,下一拍就生效(**两个轮子各一套,改的时候两边都要改**,原因见下) |

### 速度环怎么调

被控对象是"电流指令 → 轮子转速",单位上:设定值是 **°/s**,反馈是 `measure.speed_aps`(°)**,输出是电流指令**(±16384对应±20A)。

- `Kp`:每1°/s的误差给多少电流指令。给到"能跟得上设定值、但不过冲发抖"为止;
- `Ki`:消掉摩擦和地面阻力造成的静差。**给小了会掉速**(实测速度低于设定值),给大了容易和机械上的周期性阻力打架而振荡;
- `Kd`:先留0。要加的话记得同时打开 `PID_DerivativeFilter`,否则反馈噪声会被微分放大成电流尖峰。

#### 具体步骤(没有遥控器时全靠Ozone)

换算记三条:

- 轮子走 1 m/s = 轮子 **1134 °/s**(输出轴转速,轮径0.101m);
- 而速度环的误差单位是**转子**的 °/s:**1 m/s ≈ 21780 °/s**(= 1134 × 19.2),
  所以这组 PID 数值要比"按输出轴单位"想出来的**小约19倍**;
- **电流指令 ±16384 ≈ ±20A**(限幅/积分限幅是电流量纲,和上面的换算是两回事)。

1. **架空车**(轮子离地),上电,在Ozone里把 `chassis_debug_enable` 置 **1**、`chassis_dbg_c` 置 **1**(中档)。
   此时底盘完全不看遥控器,只看 `chassis_dbg_stick_v/h` 和 `chassis_stick_speed/turn_speed`。
2. 在watch里把两个速度环的积分和微分清掉:`chassis_left_speed_pid->Ki = 0`、`->Kd = 0`,
   `chassis_right_speed_pid->Ki = 0`、`->Kd = 0`。
3. **先对方向**:`chassis_dbg_stick_v = 660`、`chassis_stick_speed = 0.3`,
   看两个轮子是不是都朝"前进"方向转、`chassis_v_left_meas`/`chassis_v_right_meas` 是不是正的。
   不对就改 `CHASSIS_MOTOR_REVERSE_LEFT/_RIGHT`。
4. **做阶跃调Kp**:把 `chassis_stick_speed` 从 0 写到 1.0(相当于给速度环 21780°/s 的阶跃),
   盯这四条:`chassis_v_left_ref`(设定)、`chassis_v_left_meas`(实测)、`chassis_left_pid_out`(电流指令)、
   `chassis_left_speed_pid->Err`。Kp 从 **5** 开始试:5 → 10 → 20 → 40。
5. **加Ki**:Kp定了以后,`Ki` 从 **2** 起(2/5/10/20)。判据:稳态时实测速度有没有贴在设定值上。
6. **抄回宏**:调好之后把数值写进 `robot_def.h` 的 `CHASSIS_SPEED_KP/KI/KD`,重新编译。
   (调试器改的是RAM,复位就没了。)
7. **再验证**:`chassis_dbg_stick_v = 0`、`chassis_stick_speed` 给 1.0 看定速直行;
   然后用 `chassis_dbg_stick_h = ±660` + `chassis_turn_speed` 试转弯(应该只有一个轮子转);
   最后 `chassis_dbg_c = 2` 进来回档,盯 `chassis_distance_m`(架空时轮子在空转,编码器照样计程,
   所以距离逻辑架空就能验:应该爬到5m掉头,退回0再掉头)。

| 现象 | 原因 | 怎么改 |
| --- | --- | --- |
| 实测速度上不去、差一大截 | Kp 或 Ki 太小 | 加大 Kp;稳态仍有偏差就加大 Ki |
| 电流指令一直顶在 ±16384 | Kp 太大或负载太重 | 减小 Kp;看 `chassis_left_speed_pid->Output` 是不是饱和 |
| 速度上下抖/有啸叫,`chassis_v_left_meas` 毛刺 | Kp 太大(速度反馈的量化噪声被放大) | 减小 Kp;必要时给速度反馈加滤波(模块层 `SPEED_SMOOTH_COEF` **往小**调,它存的是"新样本的权重",1=不滤波,越小越平滑、滞后越大) |
| 低频来回摆(1~2Hz) | Ki 太大 | 减小 Ki 或减小 `CHASSIS_SPEED_I_LIMIT` |
| 稳态速度和设定值差一点,松手就掉 | Ki 太小 | 加大 Ki |
| 两个轮子一个正一个反 | 某个电机方向装反 | 改对应的 `CHASSIS_MOTOR_REVERSE_LEFT/_RIGHT` |

#### 为什么左/右轮的PID是分开的

**不是我特意分开的,是框架的结构**:`DJIMotorInit()` 会给**每一个电机实例**都建一套自己的控制器
(`Motor_Controller_s`,里面有电流/速度/角度三个PID),`gimbal.c` 里的6020也一样。所以"两个电机 = 两套PID"
是module层的设计,app层只能拿到分别指向各自那一套的指针 —— 想给两个轮子用同一个PID对象是不可能的。

**但参数初值是同一个来源**:两个电机的 `Kp/Ki/Kd` 都来自 `robot_def.h` 的 `CHASSIS_SPEED_KP/KI/KD`,
所以编译出来它们**默认完全一样**,不存在"要调两遍才一样"的问题。

分开的**实际差别**是"运行时能不能单独改":

| | 好处 | 坑 |
| --- | --- | --- |
| 两套独立 | 左右电机阻力/装配/线阻有差异时,可以单独补,让**实测转速**对齐(直行更直) | 在watch里改参数要改两个地方;只改一个 → 直行会往一边偏 |

所以调参时的建议:**每次把两边写成同一组值**(或者用Ozone的watch把两个PID对象都列出来,一次改两个);
等两边实测转速都一致了,再考虑要不要给某一个单独补。想让"改一处、两边同时生效",可以再加一个
`ChassisSetSpeedPID(kp, ki, kd)` 的接口(说一声就加)。

## 常改的参数(都在`robot_def.h`)

| 参数 | 作用 |
| --- | --- |
| `CHASSIS_MOTOR_ID_LEFT/RIGHT` | 左右轮电调上的ID(闪灯次数),默认1/2 |
| `CHASSIS_MOTOR_REVERSE_LEFT/RIGHT` | 某个轮子整体装反了就改成1 |
| `CHASSIS_WHEEL_DIAMETER_M` | 轮胎直径[m],默认0.101(实测10.1cm)。换轮胎只改这个 |
| `CHASSIS_GEAR_RATIO` | M3508自带减速箱的减速比(3591/187≈19.2),转子:输出轴。换电机/减速箱才要改 |
| `CHASSIS_SPEED_MPS` | C中档前后走的固定速度[m/s] |
| `CHASSIS_TURN_SPEED_MPS` | C中档转弯时那个轮子的速度[m/s] |
| `CHASSIS_SWING_SPEED_MPS` / `CHASSIS_SWING_DISTANCE_M` | C上档来回的速度[m/s]和单程距离[m] |
| `CHASSIS_SWING_RAMP_DISTANCE_M` / `CHASSIS_SWING_MIN_SPEED_MPS` | 端点斜坡距离[m]和折返最低速度[m/s] |

### 陀螺仪航向修正调快

只有确认 `chassis_gyro_yaw_reverse` 方向正确后再增加增益。先小幅增加
`chassis_gyro_heading_kp`(例如 `0.005 → 0.008 → 0.010`);如果差速输出长期顶在
`chassis_gyro_heading_max_speed`,可把 `chassis_gyro_heading_max_speed` 从 `0.10`
逐步提高到 `0.15`。车头开始左右摆动时,先增大 `chassis_gyro_heading_kd`,仍摆动则减小 Kp。
| `CHASSIS_STICK_DEAD_ZONE` | 摇杆死区(避免手一碰就跑) |
| `CHASSIS_STICK_V_REVERSE` / `CHASSIS_STICK_H_REVERSE` | 前后/左右反了就改成1 |
| `CHASSIS_TURN_DEAD_ZONE` | 右摇杆水平转弯死区(当前180);超过后按行程比例输出 |
| `CHASSIS_SPEED_DEAD_BAND` | 速度环死区[°/s],默认**0**。现在"没命令"和"差速结果为0的轮子"都是卸电,所以用不到死区;只有哪天你要"长期通电顶住某个速度"时才需要给它一个比静止偏置大的值 |
| `CHASSIS_C_DIAL_TH` | C从拨轮那一路进来时,|拨轮值|超过多少算拨到了上/下档(默认300,满量程±660) |
| `CHASSIS_STOP_ON_AD_ESTOP` | A下D下时底盘要不要也急停(默认1=也停,整车急停) |
| `CHASSIS_SPEED_KP/KI/KD/I_LIMIT/MAX_OUT` | 速度环PID初值 |

## 方向不对怎么办

按下面顺序,每次都**只改一处**,然后从小速度试:

1. **给前进设定,两个轮子转向相反**(或单轮往后跑) → 看 `chassis_v_left_meas` / `chassis_v_right_meas`:
   给前进命令时**两个都应该是正的**,哪个是负的就把哪边的 `CHASSIS_MOTOR_REVERSE_LEFT` / `_RIGHT` 改成1。
   **不用重新编译**:在Ozone里直接改 `chassis_left_reverse` / `chassis_right_reverse`(0=正,1=反),
   `ChassisTask()` 每拍把它们同步给模块,改完立刻生效;试对了再写回宏(免得每次上电都要改)。
   (反转做的是"把设定值取反",反馈值不跟着反,所以 `chassis.c` 里的距离计算会用同一个标志自己把符号掰回来。)
2. **单轮方向对,但整体前后相反** → 改 `CHASSIS_STICK_V_REVERSE`。
3. **左右转弯相反** → 改 `CHASSIS_STICK_H_REVERSE`。
4. **C拨杆压根没反应** → 先看 `chassis_c_source` 是不是还停在默认的0,再按上一节的表格确认
   `rc_dbg_dial`(或别的镜像)在拨C时有没有变;如果哪个都不变,说明这一帧里根本没有C这一路。
