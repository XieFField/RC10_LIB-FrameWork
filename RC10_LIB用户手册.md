## RC10_LIB FrameWork用户手册

用户手册？亦或�?�??制菜的一�?.

RC10_LIB将提供大量�?制菜，旨在�?对底层驱动不熟悉的用户也能畅�?��写应用层代码�?
而本用户手册也是预制菜的一�?��旨在让用户可以更�?��手使用RC10_LIB

**attention**: 这份手册很大程度是AI生成的，笔者只负责�?��其中部分，若发现有纰漏，请及时告诉我，万分感谀�?

### 程序�?��前执行的命名规范
1. 在类�?��变量统一�? _ 的后缀，�?`rpm_`
2. 在类�?��成员以小写字母开�?
3. 类名不�?�?��写字母和大写字母
4. RC10_LIB库中的头文件与源文件命名需带分�?��前缀，�?"Motor_","BSP_"

### 开发建�?
1. 多写注释，�?果懒得写，可以像我一样用vscode�?��的ai补全注释
2. 当您在开发没有头�?��候，�?��回顾开发手�?
3. 不�?将非API加入RC10_LIB
4. 禁�?一切动态内存分�?
5. 一切坐标采用右手系，不符合的就变换为右手系
6. 此�?架内的一切涉及�?度�?速度的都不�?直接使用弧度�?
7. 所有关于�?度的，都应当变换为[0,360](匹配PID�?��设�?)

### User
1. 机构控制类放在Control
2. 调试debug/demo类放在debug
3. Setup�?��能初始化、以及功能运行的地方

### RC10_LIB的核心�?计原�?
1. 严格分层，职责单一
   框架分为�?��驱动层、�?备协�?��、算法层和应用层。当你添加新功能时，必须明确其归属�?

   �?��驱动�?��责与物理总线通信�?
   设�?协�?�?��责解析和打包特定设�?的报文�?
   算法�?��粹的数�?工具�?
   应用�?��责下达高层指令�? 原则：一�?��法层不涉及任何硬件�?备、基层只能调用基层�?

2. 信任�?��化调度，分�?计算与打�?
   1. 例�?: fdCANbus 框架提供一�?��频率的中�?��度器，它会自动调用所有注册�?备的 update() �? packCommand()�?

   1. update(): �?��于�?算。执行�?PID等周期性算法，更新内部状态�?
   2. packCommand(): �?��于打包。�?�? update() 的�?算结果，并将其组装成待发送的CAN报文�?
   3. setTarget...(): �?��于接收指令。这�?��的驱动提供给应用层的接口，用于�?�?��级目标�? 
   4. 原则�? 永远不�?�? packCommand() �?��行�?算，也不要在 update() �?��装报文。相信调度器会按正确的顺序调用它�?�?

3. 继承统一接口，利用�?态实现特异�?
   框架通过面向接口编程实现扩展性。所有�?备驱动都必须继承�?���?��同的基类（�? Motor_Base）�?

   统一管理: 调度器只与基类接口交互，它不关心具体�?��么�?备�?
   虚函数实现�?�?: 使用 virtual 函数（�? get_GearRatio()）来让每�?��类提供自己独特的信息或�?为�? 原则�? 你的新�?备驱动必须实现基类的所有纯虚函数，并利用虚函数重写（override）来实现其特定协�?��功能�?

4. 用户使用接口的简�?
   将一切的重�?性工作都在类的封装中实现，使得用户在开发应用层的时候无需写太多冗杂重复的代码，更高效进�?开发�?

### BSP分支
#### FreeRTOS的使�?
在`BSP_RTOS.h`文件�?��封�?了基�?��RTOS使用，目前有基本的任务和队列�?

1.  **RtosTask 任务封�?**
    `RtosTask` 类提供了两�?任务模式，通过构造函数的 `period` 参数区分�?
    *   **周期性任�? (`period > 0`)**: 任务会以 `period` 指定的Tick间隔�?���?��执�? `loop()` 方法。适用于需要固定�?率运行的简单逻辑�?
        ```cpp
        class MyPeriodicTask : public RtosTask {
        public:
            MyPeriodicTask() : RtosTask("MyTask", 1000) {} // 1000ms周期
        protected:
            void loop() override 
            {
                // 这里的代码每1000ms执�?一�?
            }
        };
        ```
    *   **事件驱动任务 (`period = 0`)**: 任务创建后会执�?一�? `run()` 方法。`run()` 方法必须包含一�??�?�� `for(;;)` 和一�?��塞调�?���? `vTaskDelay`, `xSemaphoreTake`），用于等待外部事件。适用于需要�?动触发的复杂任务，例如CAN总线的调度和接收任务�?
        ```cpp
        class MyEventTask : public RtosTask {
        public:
            MyEventTask() : RtosTask("EventTask", 0) {} // 事件驱动
        protected:
            void run() override 
            {
                init(); //会�?执�?
                for(;;) 
                {
                    // 等待信号量或其他事件
                    xSemaphoreTake(mySemaphore, portMAX_DELAY); 
                    // 处理事件...
                }
            }
        };
        ```

2.  **RtosQueue 队列封�?**
    这是一�?��板类，可以方便地创建和使用线程安全的队列�?
    ```cpp
    // 创建一�?��容纳8个int的队�?
    RtosQueue<int> myQueue(8);

    // 在一�?��务中发送数�?
    myQueue.send(123);

    // 在另一�?��务中接收数据
    int received_value;
    if (myQueue.recv(received_value, 100)) { // 等待100ms
        // 成功接收到数�?
    }
    ```

### APP分支

#### APP_tool
工具类，提供�? `constrain`（限幅）等通用函数�?

#### APP_debugTool
提供调试工具，�?串口打印数据�?

#### APP_PID
提供了位�?��和�?量式两�?PID控制器�?

1.  **核心设�?**
    *   **位置式PID**: 采用了�?形积分、微分先行、积分分离等改进算法，适用于大部分需要精�?���?��制的场景�?
    *   **增量式PID**: 加入了微分跟�?��(Track_D)，能有效平滑�?��值的阶跃变化，减少系统震荡，适用于速度控制等场�?�?
    *   **固定采样时间**: PID控制器内部的 `dt` 使用时间戳方式�?算，但它大部分时候的值是�?1ms。这�?���?**核心设�?**，它强依赖于调用 `pid_calc` �? `update()` 方法�?���?���?��1kHz调度�?���? `fdCANbus::schedulerTaskbody`）所调用。后�?��考虑把杨哥那套用编码值�?算时间的代码�?��来，�?��让dt更加精确�?

2.  **用户该�?何使�?��**
    在电机类（�? `M3508`）的 `pid_init` 函数�?��始化PID参数，然后在 `update` 函数�?���? `pid_calc` 即可。用户无需关心 `dt` 的�?算�?
    ```cpp
    // �? M3508::update() �?
    case SPEED_CONTROL:
    {
        // target_rpm_ �? this->rpm_ 都是输出轴转速，尺度统一
        target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
        break;
    }
    ```

    **如果你使用的�?���?��PID**
        位置式PID包含了两种模式：
            1. 线性模式：此模式下，适合�?��式的PID
            2. �?��模式：�?模式下，适合云台式的PID，范围为[0,360];

#### APP_CoordConvert
`APP_CoordConvert` �?���?���? `CMSIS-DSP` 库优化的高性能坐标变换工具，用于�?�?2D�?3D空间�?��平移和旋�?�?

##### 核心特�?
- **高性能**: 所有矩阵运算都�? `arm_math.h` �?��函数完成，充分利用硬件加速�?
- **易于使用**: 提供�? `HomogeneousTransform2D` �? `HomogeneousTransform3D` 两个类，接口清晰直�?�?
- **功能完�?**: �?��设置变换、应用变捀��矩阵乘法（变换叠加）和求逆变捀�?

##### **【重要提示�?**
- **角度单位**: 所有函数的角度参数（�? `theta_rad`, `roll_rad`）都必须使用 **弧度 (radians)** 作为单位�?
- **命名空间**: 所有类和函数都位于 `geometry` 命名空间下�?

##### 2D变换使用示例

假�?有一�?��感器安�?在机器人上，其坐标系相�?于机器人�?��坐标系有如下关系�?
- 沿机器人X轴平移了 `0.2` 米�?
- 沿机器人Y轴平移了 `0.1` 米�?
- 逆时针旋�?�� `45` 度�?

现在，传感器检测到了一�?��其自�?��标系下的�? `(0.5, 0.0)`，我�?��知道这个点在机器人中心坐标系下的位置�?

```cpp
#include "APP_CoordConvert.h"
#include "arm_math.h" // For PI constant

// 使用命名空间
using namespace geometry;

void transform_example_2d()
{
    // 1. 定义一�? Point2D 对象来描述从传感器到机器人中心的位姿
    //    平移 (0.2, 0.1)，旋�? 45 �? (PI/4 弧度)
    Point2D sensor_pose(0.2f, 0.1f, PI / 4.0f);

    // 2. 使用该位姿�?象创建变换矩�?
    HomogeneousTransform2D sensor_to_robot_tf(sensor_pose);

    // 3. 定义在传感器坐标系下的点
    Point2D point_in_sensor(0.5f, 0.0f);

    // 4. 应用变换，得到在机器人坐标系下的�?
    Point2D point_in_robot = sensor_to_robot_tf.apply(point_in_sensor);

    // point_in_robot.x �? point_in_robot.y 就是最终结�?
}
```

##### 3D变换使用示例

假�?相机坐标系相对于世界坐标系平移了 `(1.0, 2.0, 0.5)`，并且绕Z轴旋�?��90度�?

```cpp
#include "APP_CoordConvert.h"
#include "arm_math.h"

using namespace geometry;

void transform_example_3d()
{
    // 1. 定义一�? Point3D 对象来描述从相机到世界坐标系的位�?
    //    平移 (1, 2, 0.5)，绕Z�?(yaw)旋转90�? (PI/2)
    Point3D camera_pose(1.0f, 2.0f, 0.5f, 0.0f, 0.0f, PI / 2.0f);

    // 2. 使用该位姿�?象创建变换矩�?
    HomogeneousTransform3D camera_to_world_tf(camera_pose);

    // 3. 定义在相机坐标系下的一�?��
    Point3D point_in_camera(0.0f, 1.0f, 0.0f);

    // 4. 应用变换，得到在世界坐标系下的点
    Point3D point_in_world = camera_to_world_tf.apply(point_in_camera);

    // 5. 计算逆变�?��从世界坐标系到相机坐标系�?
    HomogeneousTransform3D world_to_camera_tf = camera_to_world_tf.inverse();

    // 6. 使用逆变换将世界坐标系下的点�?��回相机坐标系
    Point3D point_back_in_camera = world_to_camera_tf.apply(point_in_world);
    // 此时 point_back_in_camera 应�?约等�? point_in_camera
}
```

### Module分支
此分�?��要包�?��特定�?��模块相关的代码，例�? `Module_Encoder.cpp`，它负责将编码器的原始值（�?0-8191）转�?��连续的�?度（-�?, +∞）和单圈�?�?0, 360]�?

#### Chassis_Base 底盘基类使用指南

`Chassis_Base` �?���?��于构建各种底盘运动�?模型的强大基类。它采用C++模板和面向�?象的设�?，实现了运动学解算与具体电机驱动的完全解耦�?

##### 核心设�?

- **静态泛型�?�?**: 使用 `template <std::size_t WheelCount>`，你�?��在编译时就确定底盘的�?��数量，所有内存均为静态分配，符合嵌入式系统的高可靠性�?求�?
- **职责分�?**: `Chassis_Base` �?��责运动�?计算。它计算出每�?��子应该达到的�?���?��（RPM），然后通过 `setTargetRPM()` 将这�?��标传递给已注册的电机对象。实际的电机PID�?��控制和CAN报文发送则�? `fdCANbus` 的调度器�?��完成�?
- **坐标系�?�?**: 内置机器人坐标系和世界坐标系的速度管理。你�?��通过 `updateAngleData()` 提供实时的偏�??（yaw），基类就能�?��处理两个坐标系之间的速度�?���?
- **�?��的更新循�?**: `Chassis_Base` �? `update()` 方法**不会**�? `fdCANbus` �?��调用。你需要在�?��的控制任务中，以你期望的频率来调用它�?


GitHub Copilot

以下内�?�?��接粘贴到“RC10_LIB用户手册.md”�?

## Module_Air_joy �?��遥控 PPM 驱动使用指南

AirJoy �?���?���? EXTI �?��与微秒级时间戳的 PPM 解码器。它�? 8 �?��模通道脉�?（约 1000�?2000 us）解析为易用的数值成员，供底盘、机械臂等上层模块直接�?取�?

### 1. 功能与依�?
- 功能：PPM 输入，自动识�?��头，解析 8 �?��道到成员变量：
  - 模拟量：LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y（单位：�??�?950�?2050�?
  - 开关量：SWA, SWB, SWC, SWD（同样是脉�?值，常�?为两/三档�?
- 依赖�?
  - �??级时间戳服务：BSP_TimeStamp（需先初始化�?
  - GPIO 外部�?��（EXTI�?
  - HAL 库（STM32H7�?

### 2. �?���? CubeMX 配置
- �? PPM 信号接入一�?���? EXTI �? GPIO�?3.3V 逻辑，建�??部上�?/下拉按接收器要求配置）�?
- CubeMX 配置步�?�?
  1. 选择用于 PPM �? GPIO 引脚，模式�?�? External Interrupt Mode（上升沿触发即可）�?
  2. 使能�? EXTI �? NVIC �?���?
  3. 选择一�?��时器�? TimeStamp 使用（任意稳定时钟源），保持一直运行�?
  4. �??系统时钟已�?�?���?�?

### 3. 初�?化与回调
- 初�?�? TimeStamp（示例）�?
    ````cpp
    #include "BSP_TimeStamp.h"
    extern TIM_HandleTypeDef htim2;

    void user_setup()
    {
        TimeStamp::getInstance().init(&htim2); // 让微秒�?时开始跑
        // ... 其他初�?�? ...
    }
    ````

- EXTI 回调（库已内�?��例）�?
  - Module_Air_joy.cpp �?��实现
    air_joy.data_update(GPIO_Pin, GPIO_PIN_8);
  - 如你�? PPM 不在 PIN_8，�?把�?二个参数改为实际使用的那�? GPIO_PIN_XX�?
  - 如果你打算自己写回调，可参考：
    ````cpp
    #include "Module_Air_joy.h"

    extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        // 假�? PPM 接在 PB6
        air_joy.data_update(GPIO_Pin, GPIO_PIN_6);
    }
    ````

### 4. 读取数据与归一�?
- AirJoy 成员变量实时更新（在每收到一整帧后写入），单位均为微秒：
  - LEFT_X = PPM_buf[0]
  - LEFT_Y = PPM_buf[1]
  - RIGHT_X = PPM_buf[3]
  - RIGHT_Y = PPM_buf[2]
  - SWA = PPM_buf[4], SWB = PPM_buf[5], SWC = PPM_buf[6], SWD = PPM_buf[7]
- 典型归一化方法（�? 1000�?2000 us 映射�? -1~+1 �? 0~1）：
    ````cpp
    ```cpp
    static inline float ppm_to_norm_pm(const uint16_t us, uint16_t mid=1500, float span=500.0f)
    {
        // [-1, +1]�?1500 为中值，±500us 为满量程
        return (static_cast<float>(us) - static_cast<float>(mid)) / span;
    }

    static inline float ppm_to_norm_01(const uint16_t us, uint16_t min_us=1000, uint16_t max_us=2000)
    {
        float u = (float)(us - min_us) / (float)(max_us - min_us);
        if(u < 0.f) u = 0.f;
        if(u > 1.f) u = 1.f;
        return u;
    }
    ````

    - 示例：将通道映射为底盘速度指令
    ````cpp
    ```cpp
    // 假�?使用全向底盘，单位自定（例�? m/s、rad/s�?
    float vx_cmd  = ppm_to_norm_pm(air_joy.LEFT_Y)  * 1.0f; // �?/�?
    float vy_cmd  = ppm_to_norm_pm(air_joy.RIGHT_X) * 1.0f; // �?/�?
    float yaw_cmd = ppm_to_norm_pm(air_joy.LEFT_X)  * 2.0f; // 旋转
    // 按需限幅后送入 setRobotSpeed �? setWorldSpeed
    ````

### 5. 与任务循�?��关系
- AirJoy 通过 EXTI �?��按边沿采样并计算脉�?，不需要你在任务里专门“更新”�?
- 建�?以固定周期（例�? 10ms）�?取成员变量并做归一化，再下发给底盘/执�?器�?
- 如果需要判�?��数�?��否新鲜”，�?��用户代码�??录上次使用的值或时间，并�? TimeStamp 取差值做超时判定（例�? >50ms 则�?为遥控断联，进入安全模式）�?

### 6. 通道/引脚�?���?
- 最大通道数：默�? 8（MAX_CHANNELS=8）�?
- 通道映射：在 Module_Air_joy.cpp �?��调整 PPM_buf 索引�? LEFT/RIGHT/SW 的映射�?
- EXTI 引脚：修�? HAL_GPIO_EXTI_Callback �?���? data_update �? GPIO_PIN_* 常量即可�?
- 时间阈值：
  - 帧结束阈�? FRAME_END_MIN（默�? 2100 us�?
  - PWM_MIN/PWM_MAX（默�? 950/2050 us�?
  根据你的接收机协�?��当调整�?

### 7. 常�?�??
- 无数�?��新：
  - �?? TimeStamp �? init 且在跑（�??递�?）�?
  - �?? EXTI 配置到�?�?��脚、触发沿、NVIC 已使能�?
  - �?? HAL_GPIO_EXTI_Callback �?��用的 GPIO_PIN_* 与实际一致�?
- 抖动/数值跳变：
  - 线长、干扰、上�?/下拉配置不当都会导致错�?触发�?
  - �?��驱动内�?加简单滤�?��当前实现为“直接采样”，便于低延迟）�?
- �?��部分通道更新�?
  - 检查接收机输出�?���? PPM（不�? SBUS/IBUS）�?
  - 检�? MAX_CHANNELS 与你的接收机通道数是否匹配�?

以上即可�?��把 PPM 遥控接入你的应用。建�?��串口打印四个主通道的原�? us 值，�??范围与中值，再做归一化映射与控制联调�?

##### 如何使用 `Chassis_Base`

###### 1. 创建你的底盘子类 (AI生成，不用尽�?)

首先，你需要创建一�?��承自 `Chassis_Base` 的子类，并实现其�?��函数。以一�?���?��克纳姆轮底盘为例�?

**`Module_MecanumChassis.h`**
```cpp
#include "Module_ChassisBase.h"

class MecanumChassis : public Chassis_Base<4> {
public:
    // 构造函数：传入�?��半径、最�?PM和底盘的几何参数
    MecanumChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y);

protected:
    // 【必须】实现运动�?更新
    void updateKinematics() override;

    // 【必须】实现逆解：从机器人速度计算�?�?
    void inverseKinematics(const Robot_Twist& twist) override;

    // 【必须】实现�?解：从轮速�?算机器人速度
    void forwardKinematics() override;

private:
    // 麦轮底盘的几何参�?
    const float wheel_distance_x_; // �?��在X方向上的半间�?
    const float wheel_distance_y_; // �?��在Y方向上的半间�?
};
```

**`Module_MecanumChassis.cpp`**
```cpp
#include "Module_MecanumChassis.h"

// 构造函�?
MecanumChassis::MecanumChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y)
    : Chassis_Base<4>(wheel_radius, max_wheel_rpm),
      wheel_distance_x_(wheel_distance_x),
      wheel_distance_y_(wheel_distance_y)
{}

// 运动学更新：先逆解，再正解
void MecanumChassis::updateKinematics() {
    inverseKinematics(this->robot_twist_); // 使用经过斜坡处理后的当前速度进�?逆解
    forwardKinematics();                   // 根据实际�?��反馈（如果需要）进�?正解
}

// 逆解实现
void MecanumChassis::inverseKinematics(const Robot_Twist& twist) {
    const float lx_plus_ly = wheel_distance_x_ + wheel_distance_y_;
    const float rad_per_s_to_rpm = 60.0f / (2.0f * PI);

    // 麦克纳�?�?��解�?��
    float wheel_speed_rad_s[4];
    wheel_speed_rad_s[0] = (twist.vx - twist.vy - twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[1] = (twist.vx + twist.vy + twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[2] = (twist.vx + twist.vy - twist.yaw_rate * lx_plus_ly) / wheel_radius_;
    wheel_speed_rad_s[3] = (twist.vx - twist.vy + twist.yaw_rate * lx_plus_ly) / wheel_radius_;

    // 将�?算出的�?速度(rad/s)�?��为RPM，并存入�?��数组
    for (int i = 0; i < 4; ++i) {
        this->wheele_target_rpm_[i] = wheel_speed_rad_s[i] * rad_per_s_to_rpm;
    }
}

// 正解实现 (示例，实际可能需要从电机获取真实速度)
void MecanumChassis::forwardKinematics() {
    // 这里仅为示例，实际应用中你可能需要从 wheels_[i]->getRPM() 获取真实�?��来计算
    // 此�?暂时留空或基于目标速度进�?估算
}
```

###### 2. 在应用层集成

在你�? `user_setup` 和控制任务中，将所有部分组合起来�?

```cpp
/* user_setup.cpp �? main.cpp */
#include "Module_MecanumChassis.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "BSP_IMU.h" // 假�?你有一个IMU模块

// --- 全局对象定义 ---
// 【修改】通过 getInstance 获取 CAN 总线的唯一实例指针
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1, 1);

// 【保持不变】静态创建电机和底盘对象
M3508 wheel_motors[4] = { M3508(1, CAN1_Bus), M3508(2, CAN1_Bus), M3508(3, CAN1_Bus), M3508(4, CAN1_Bus) };
DJI_Group DJI_Group_1(0x200, CAN1_Bus);
MecanumChassis my_chassis(0.076f, 450.0f, 0.2f, 0.25f); // �?���?, 最�?PM, x间距, y间距
IMU_Class my_imu; // 假�?的IMU对象

// --- 初�?化函�? ---
void user_setup() {
    // 1. 初�?化电机和PID
    for (int i = 0; i < 4; ++i) {
        wheel_motors[i].pid_init(/* ... */);
        DJI_Group_1.addMotor(&wheel_motors[i]);
        // 【修改】注册电机时，使�? CAN1_Bus 指针
        CAN1_Bus->registerMotor(&wheel_motors[i]);
    }
    CAN1_Bus->registerMotor(&DJI_Group_1);
    CAN1_Bus->init();

    // 2. 注册�?��到机箱模�?
    // 注意�?��顺序要与你的运动学模型一�?
    my_chassis.registerWheelMotor(0, &wheel_motors[0]); // 右前�?
    my_chassis.registerWheelMotor(1, &wheel_motors[1]); // 左前�?
    my_chassis.registerWheelMotor(2, &wheel_motors[2]); // 左后�?
    my_chassis.registerWheelMotor(3, &wheel_motors[3]); // 右后�?

    // 3. 配置加速度限制 (�?�?)
    my_chassis.reset_AccLimitStatus(true); // �?��
    my_chassis.reset_AccValue(1.0f);       // 1.0 m/s^2
}

// --- 控制任务 ---
class ChassisControlTask : public RtosTask {
public:
    ChassisControlTask() : RtosTask("ChassisTask", 10) {} // 10ms周期, 100Hz

protected:
    void loop() override {
        // 1. 从遥控器或上位机获取�?��速度
        Robot_Twist target_speed;
        target_speed.vx = remote.getChannel(2); // 假�?从遥控器获取前进速度
        target_speed.vy = remote.getChannel(3); // 假�?从遥控器获取平移速度
        target_speed.yaw_rate = remote.getChannel(0); // 假�?从遥控器获取旋转速度

        // 2. 从IMU获取当前姿�?
        Angle_Twist current_angle = my_imu.getAngle();
        my_chassis.updateAngleData(current_angle);

        // 3. 设置�?��速度到机箱模�? (使用世界坐标�?)
        my_chassis.setWorldSpeed(target_speed);

        // 4. 【核心】更新机箱模�?
        // 这会执�?运动学解算，并将�?��RPM设置给电�?
        my_chassis.update();
    }
};
```

通过以上步�?，你就成功地将一�?��克纳姆轮底盘集成到了RC10_LIB框架�?��`Chassis_Base` 负责了�?杂的运动学�?算和坐标变换，�? `fdCANbus` 则在后台默默地保证了所有电机PID的精�?��行。你的控制任务只需要关注“我想�?底盘以什么速度移动”这一高层逻辑�?

---

### fdCANbus如何工作的？

`fdCANbus` �?���?��机控制库的�?经中枀��它负责处理底层的CAN通信，并以精�?��频率�?��调度所有电机控制任务，将用户从繁琐的实时控制和�?��交互�?��放出来�?

#### 核心组件与工作流�?

`fdCANbus` 内部主�?由两�?��行的RTOS任务驱动�?

1.  **接收任务 (`rxTask_`)**:
    *   **工作**: 这是一�?��件驱动的任务，它永久阻�?并等�? `rxQueue_` 队列�?��新消�?�?
    *   **数据�?**:
        1.  当CAN�?��接收到一�?���?��，`HAL_FDCAN_RxFifo0Callback` �?��服务程序（ISR）�?触发�?
        2.  ISR调用 `fdcan_global_rx_isr`，�?函数从硬件缓冲区读取原�?CAN报文�?
        3.  原�?报文�?��装成 `CanFrame` 对象，并�?��即推�? `rxQueue_` 队列�?
        4.  `rxTask_` �?��醒，从队列中取出 `CanFrame`�?
        5.  `rxTask_` 遍历所有已注册的电机（`motorList_`），调用每个电机�? `matchesFrame()` 方法来�?找�?报文的“主人”�?
        6.  一旦找到匹配的电机，就调用�? `updateFeedback()` 方法，将报文交由电机�??解析�?

2.  **调度任务 (`schedulerTask_`)**:
    *   **工作**: 这是一�?��优先级的、由定时器精�?��发的周期性任务，频率�?1kHz�?
    *   **数据�?**:
        1.  一�?1kHz的硬件定时器�?��触发 `fdcan_global_scheduler_tick_isr()`�?
        2.  �?SR释放（Give）一�?���? `schedSem_` 的信号量，然后立即退出�?
        3.  `schedulerTask_` 在启动后就一直阻塞等待（Take）这�?��号量。一旦获取到信号量，它就会�?唤醒�?
        4.  **更新**: 任务首先遍历所有注册的电机（或电机组），并调用它们�? `update()` 方法。这会触发PID计算等控制逻辑�?
        5.  **打包**: 接着，任务再次遍历所有�?象，调用 `packCommand()` 方法来收集需要发送的CAN指令�?,此�?利用`packCommand()`的返回值�?录需要发送几�?AN�?
        6.  **发�?**: 最后，任务将所有收集到的指令帧通过 `sendFrame()` 方法发送出去。`sendFrame` 内部使用互斥�? `tx_mutex_` 来确保�?任务访问CAN�?��的线程安全�?
        7.  完成一�?��度后，`schedulerTask_` 返回�?��的开始，再�?阻�?等待下一次的信号量，从而实现精�?��1ms周期�?

#### 关键设�?决策

*   **�?��服务程序（ISR）最小化**: ISR�?��最少的工作——�?取数�?��将其推入队列。所有耗时的操作（如遍历、匹配、解析）都转移到优先级较低的 `rxTask_` �?��行，这确保了系统的实时响应能力�?
*   **发送与接收分�?**: 接收�?��全异步和事件驱动的，而发送则�?��步和周期性的。这种�?计�?合控制系统的典型模式：持�?��收反馈，并以固定的�?率输出控制指令�?
*   **全局�?���?��**: 通过一�?��局�? `g_fdcan_bus_map` 数组，可以将来自HAL库的、不区分具体总线的C风格�?��回调，精�?���?��到�?应的 `fdCANbus` C++对象实例上。这使得代码�?��轻松�?��多个CAN总线�?
*   **线程安全**: 通过使用RTOS队列（`RtosQueue`）和互斥锁（`tx_mutex_`），`fdCANbus` �?��了在多任务环境下数据交换和硬件�?�?��安全性�?

### 电机库核心�?计与使用指南

�?��南将引�?你完成从�?��初�?化到�? RTOS 任务�?��制电机的完整流程�?

#### 核心设�?思想

1.  **数据�?��前置**: �? `DJI_Motor::updateFeedback` 函数�?��从CAN总线接收到的**电机�?��原�?数据**（转速、编码器值）�?**立即**通过虚函�? `get_GearRatio()` 获取正确的减速比，并�?���?��**减速后的输出轴数据**�?

2.  **内部状态统一**: �?��完成后，所有存储在基类 `Motor_Base` �?��成员变量（`rpm_`, `angle_`, `totalAngle_`）的�?��都统一�?**输出轴的状�?**�?

3.  **控制与反馈尺度统一**: PID控制�?��（在 `update()` 方法�?���?**�?���?**（�? `target_rpm_`）和**反�?�?**（�? `this->rpm_`）都基于**输出轴的尺度**进�?计算，保证了控制的�?�?���?

4.  **调度�?���?**: �? **不需�?** 手动调用 PID 计算�? CAN 发送函数。`fdCANbus` 内部�? `schedulerTask` 会以 1kHz 的�?率自动完成所有已注册电机（或电机组）�? `update()` �? `packCommand()` 调用�?

5.  **用户职责**: 你的工作非常简单，�?��在一�?��立的控制任务�?��根据需要调�? `setTargetRPM()`, `setTargetAngle()` 等函数来设定**输出轴的�?���?**即可�?

#### �?��步：系统初�?�?

所有硬件和对象的初始化都应该在�?�� RTOS 调度�? (`osKernelStart()`) 之前完成。推荐在 `main.cpp` �? `USER CODE BEGIN 2` �? `USER CODE END 2` 之间，或者一�?��门的 `user_setup.cpp` 文件�?��行�?

```cpp
/* main.cpp �? user_setup.cpp */

#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"

// 1. 定义全局对象
// 【修改】不再直接定�? fdCANbus 对象，而是通过 getInstance 获取其唯一实例的指�?
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1, 1);

// 【保持不变】静态创建电机和电机组�?象，并将 CAN1_Bus 指针传递给它们
M3508 m3508_1(1, CAN1_Bus);     // M3508电机, ID�?1
DJI_Group DJI_Group_1(0x200, CAN1_Bus); // DJI电机�?, 发送ID�?0x200

// 2. 创建一�?��始化函数
void user_setup()
{
    // --- PID参数配置 ---（AI生成的，并非通用参数�?
    PID_Param_Config speed_pid_params = 
    {
        .kp = 10.0f, .ki = 0.5f, .kd = 0.0f,
        .I_Outlimit = 5000.0f, .isIOutlimit = true,
        .output_limit = 16000.0f, .deadband = 0.0f
    };
    PID_Param_Config angle_pid_params = 
    {
        .kp = 0.5f, .ki = 0.0f, .kd = 0.0f,
        .I_Outlimit = 100.0f, .isIOutlimit = true,
        .output_limit = 500.0f, .deadband = 0.0f
    };
    m3508_1.pid_init(speed_pid_params, 0.0f, angle_pid_params, 30.0f);

    // --- 注册与配�? ---
    // 将电机添加到电机�?
    DJI_Group_1.addMotor(&m3508_1);
    // 你可以继�?��加更多电机到这个�?...
    // DJI_Group_1.addMotor(&another_motor);

    // 【重要】将电机�?��和电机组都注册到CAN总线
    // 【修改】通过 CAN1_Bus 指针调用 registerMotor
    // 1. 注册电机�?��，使其能接收反�?报文并更新状�?
    CAN1_Bus->registerMotor(&m3508_1);
    // 2. 注册电机组，使其能�?调度器调�? packCommand() 来打包发送电流指�?
    CAN1_Bus->registerMotor(&DJI_Group_1);

    // --- �?��总线 ---
    // 【修改】通过 CAN1_Bus 指针调用 init
    // 这会�?��CAN的接收中�?��1kHz的调度任�?
    CAN1_Bus->init();
}

// �? main() 函数�?���?
int main(void)
{
    // ... HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), MX_FDCAN1_Init() ...
    
    user_setup(); // 调用我们的初始化函数
    
    osKernelInitialize();
    // ... 创建其他用户任务 ...
    osKernelStart();
    
    // ...
}
```

#### 如果你想拓展电机�?
假�?你�?添加一�?��DJI的、有�?���?��CAN协�?的电机，例�? MyMotor�?

1. 创建 `Motor_MyMotor.h`

```cpp
#include "Motor_Base.h"
#include "APP_PID.h" // 如果需要PID

class MyMotor : public Motor_Base {
public:
    // 1. 构造函数：调用基类构造函�?
    MyMotor(uint32_t id, fdCANbus* bus) 
        : Motor_Base(id, false, bus) // 假�?使用标准�?
    {
        // 初�?化�?电机的�?有成�?
    }

    // 2. 【必须】�?�? packCommand
    //    根据 target_current_ 等目标值，打包成�?电机的CAN�?
    //    此�?的返回值务必实现，否则会�?fdCANbus检测总线上CAN帧数量异常，导致发送丢包�?
    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    // 3. 【必须】�?�? updateFeedback
    //    解析收到的CAN帧，更新 rpm_, angle_ 等成员变�?
    void updateFeedback(const CanFrame& cf) override;

    // 4. 【必须】�?�? matchesFrame
    //    判断收到的CAN帧是否属于这�?���?
    bool matchesFrame(const CanFrame& cf) const override;

    // 5. 【必须】�?�? get_GearRatio
    //    返回该电机的真实减速比
    float get_GearRatio() const override { return 27.0f; } // 假�?减速比�?27

    // 6. 实现 update 方法，用于执行PID计算
    void update() override;

    // 7. 实现 setTarget... 等控制接�?
    void setTargetRPM(float rpm_set) override;

private:
    // 该电机的私有成员，�?PID控制�?
    PID_Incremental speed_pid_;
};
```

2. �? `Motor_MyMotor.cpp` �?��现功�?
```cpp
#include "Motor_MyMotor.h"

std::size_t MyMotor::packCommand(CanFrame outFrames[], std::size_t maxFrames) {
    // ... 根据 this->target_current_ 打包CAN�? ...
    // outFrames[0].ID = 0x123;
    // outFrames[0].data[0] = ...;
    return 1; // 返回打包的帧�?
}

void MyMotor::updateFeedback(const CanFrame& cf) {
    // ... 解析 cf.data ...
    // float raw_rpm = ...;
    // this->rpm_ = raw_rpm / get_GearRatio(); // �?��为输出轴�?�?
}

bool MyMotor::matchesFrame(const CanFrame& cf) const {
    // 判断逻辑，例如：
    return (cf.ID == (0x200 + this->motor_id_));
}

void MyMotor::update() {
    // ... 调用PID计算 ...
    // target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);
}

void MyMotor::setTargetRPM(float rpm_set) {
    // ... 设置�?���? ...
    this->target_rpm_ = rpm_set;
}
```
3. 在应用层使用 像使�? `M3508` 一样，创建 `MyMotor` 对象，并将其注册�? `fdCANbus` 即可。调度器会自动�?理后�?��一切�?


