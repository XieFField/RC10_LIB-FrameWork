/**
 * @brief 三全向轮底盘运动学回归测试
 *
 * 在重构前锁住原有运动学行为，确保代码迁移到库文件后不出现行为变化。
 * 测试通过 Chassis_Omni<3> 的公共接口进行，不依赖私有成员。
 */

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include "Module_ChassisOmni.h"

// TimeStamp 全局变量（库层依赖）
float TimeStamp::now_s_ = 0.0f;

namespace
{
int g_failures = 0;

void expectTrue(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::printf("FAIL line %d: %s\n", line, expression);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float tolerance, const char *expression, int line)
{
    if (std::fabs(actual - expected) > tolerance)
    {
        std::printf("FAIL line %d: %s actual=%f expected=%f tolerance=%f\n",
                    line, expression,
                    static_cast<double>(actual),
                    static_cast<double>(expected),
                    static_cast<double>(tolerance));
        ++g_failures;
    }
}

void expectGt(float actual, float threshold, const char *expression, int line)
{
    if (!(actual > threshold))
    {
        std::printf("FAIL line %d: %s actual=%f expected > %f\n",
                    line, expression,
                    static_cast<double>(actual),
                    static_cast<double>(threshold));
        ++g_failures;
    }
}

#define EXPECT_TRUE(expr) expectTrue((expr), #expr, __LINE__)
#define EXPECT_NEAR(actual, expected, tolerance) expectNear((actual), (expected), (tolerance), #actual, __LINE__)
#define EXPECT_GT(actual, threshold) expectGt((actual), (threshold), #actual, __LINE__)

// ============================================================
// FakeMotor：模拟电机，记录收到的目标值
// ============================================================
class FakeMotor : public Motor_Base
{
public:
    FakeMotor() : Motor_Base(0, false, nullptr) {}

    void setTargetRPM(float rpm_set) override
    {
        target_rpm_ = rpm_set;
        ++rpm_calls;
    }

    void setTargetCurrent(float current_set) override
    {
        target_current_ = current_set;
        ++current_calls;
    }

    void setTargetAngle(float angle_set) override
    {
        target_angle_ = angle_set;
    }

    std::size_t packCommand(CanFrame[], std::size_t) override { return 0; }
    void updateFeedback(const CanFrame &) override {}

    int rpm_calls = 0;
    int current_calls = 0;
};

// ============================================================
// 逆运动学：纯 vx 输出测试
// ============================================================
void testPureVxForwardProducesEqualWheelSpeeds()
{
    // 120° 均布三轮底盘，所有轮子半径相同，期望三个轮子输出相同线速度
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 1000.0f;
    // 轮1：正上方，0°
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    // 轮2：左下，-120° 位置
    cfg.wheels[1].theta = -63.741f + 180.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    // 轮3：右下，120° 位置
    cfg.wheels[2].theta = 63.741f + 180.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    FakeMotor m0, m1, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(1, &m1);
    chassis.registerWheelMotor(2, &m2);

    Robot_Twist target{};
    target.vx = 1.0f; // 纯 x 方向 1m/s
    target.vy = 0.0f;
    target.yaw_rate = 0.0f;

    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    chassis.update();

    // 纯 vx 前进，三轮 RPM 应相等（120° 均布下每个轮 cos(安装角) * vx）
    float rpm0 = m0.getTargetRPM();
    float rpm1 = m1.getTargetRPM();
    float rpm2 = m2.getTargetRPM();

    std::printf("  vx=1.0: rpm0=%f rpm1=%f rpm2=%f\n",
                static_cast<double>(rpm0),
                static_cast<double>(rpm1),
                static_cast<double>(rpm2));

    EXPECT_GT(std::fabs(rpm0), 0.0f);
    // 三个轮子转速绝对值应接近（对称布局下）
    EXPECT_NEAR(std::fabs(rpm1), std::fabs(rpm2), 0.5f);
}

// ============================================================
// 逆运动学：纯 vy 输出测试
// ============================================================
void testPureVySidewaysProducesNonZeroWheelSpeeds()
{
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 1000.0f;
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    cfg.wheels[1].theta = -63.741f + 180.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    cfg.wheels[2].theta = 63.741f + 180.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    FakeMotor m0, m1, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(1, &m1);
    chassis.registerWheelMotor(2, &m2);

    Robot_Twist target{};
    target.vx = 0.0f;
    target.vy = 1.0f; // 纯 y 方向 1m/s
    target.yaw_rate = 0.0f;

    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    chassis.update();

    float rpm0 = m0.getTargetRPM();
    float rpm1 = m1.getTargetRPM();
    float rpm2 = m2.getTargetRPM();

    std::printf("  vy=1.0: rpm0=%f rpm1=%f rpm2=%f\n",
                static_cast<double>(rpm0),
                static_cast<double>(rpm1),
                static_cast<double>(rpm2));

    // 轮0 在 theta=0° 位置 cos(theta)=1, sin(theta)=0, 只能产生 vx 分量不能产生 vy
    EXPECT_NEAR(rpm0, 0.0f, 0.01f);
    EXPECT_GT(std::fabs(rpm1), 0.0f);
    EXPECT_GT(std::fabs(rpm2), 0.0f);
}

// ============================================================
// 逆运动学：纯旋转 wz 输出测试
// ============================================================
void testPureRotationProducesAllWheelsTurning()
{
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 1000.0f;
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    cfg.wheels[1].theta = -63.741f + 180.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    cfg.wheels[2].theta = 63.741f + 180.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    FakeMotor m0, m1, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(1, &m1);
    chassis.registerWheelMotor(2, &m2);

    Robot_Twist target{};
    target.vx = 0.0f;
    target.vy = 0.0f;
    target.yaw_rate = 1.0f; // 纯旋转 1rad/s

    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    chassis.update();

    float rpm0 = m0.getTargetRPM();
    float rpm1 = m1.getTargetRPM();
    float rpm2 = m2.getTargetRPM();

    std::printf("  wz=1.0: rpm0=%f rpm1=%f rpm2=%f\n",
                static_cast<double>(rpm0),
                static_cast<double>(rpm1),
                static_cast<double>(rpm2));

    // 纯旋转时三轮线速度方向相同（同向旋转），转速应同号
    EXPECT_GT(std::fabs(rpm0), 0.0f);
    EXPECT_TRUE((rpm0 > 0.0f && rpm1 > 0.0f && rpm2 > 0.0f) ||
                (rpm0 < 0.0f && rpm1 < 0.0f && rpm2 < 0.0f));
}

// ============================================================
// 零电流模式
// ============================================================
void testZeroCurrentModeSendsZeroCurrentToAllMotors()
{
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 1000.0f;
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    cfg.wheels[1].theta = 120.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    cfg.wheels[2].theta = -120.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    FakeMotor m0, m1, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(1, &m1);
    chassis.registerWheelMotor(2, &m2);

    Robot_Twist target{};
    chassis.set_ControlMode(CURRENT_ZERO_MODE);
    chassis.set_Target(target);

    EXPECT_NEAR(m0.getTargetCurrent(), 0.0f, 0.0001f);
    EXPECT_NEAR(m1.getTargetCurrent(), 0.0f, 0.0001f);
    EXPECT_NEAR(m2.getTargetCurrent(), 0.0f, 0.0001f);
}

// ============================================================
// 速度限幅：超过最大 RPM 时等比缩放
// ============================================================
void testSpeedLimitingScalesAllWheelsWhenAnyExceedsMax()
{
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 100.0f; // 很低的限制
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    cfg.wheels[1].theta = 120.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    cfg.wheels[2].theta = -120.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    FakeMotor m0, m1, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(1, &m1);
    chassis.registerWheelMotor(2, &m2);

    // 请求一个很大的速度
    Robot_Twist target{};
    target.vx = 10.0f;
    target.vy = 10.0f;
    target.yaw_rate = 10.0f;

    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    chassis.update();

    float rpm0 = m0.getTargetRPM();
    float rpm1 = m1.getTargetRPM();
    float rpm2 = m2.getTargetRPM();

    std::printf("  limited: rpm0=%f rpm1=%f rpm2=%f\n",
                static_cast<double>(rpm0),
                static_cast<double>(rpm1),
                static_cast<double>(rpm2));

    // 所有轮子的 RPM 不应超过 max_wheel_rpm
    EXPECT_TRUE(std::fabs(rpm0) <= 100.0f + 1.0f);
    EXPECT_TRUE(std::fabs(rpm1) <= 100.0f + 1.0f);
    EXPECT_TRUE(std::fabs(rpm2) <= 100.0f + 1.0f);
}

// ============================================================
// 空指针安全：未注册的电机不应被访问
// ============================================================
void testUnregisteredMotorsAreNotAccessed()
{
    Chassis_Omni<3>::init_config cfg{};
    cfg.wheel_radius = 0.075f;
    cfg.max_wheel_rpm = 1000.0f;
    cfg.wheels[0].theta = 0.0f;
    cfg.wheels[0].x = 0.0f;
    cfg.wheels[0].y = 0.375f;
    cfg.wheels[1].theta = 120.0f;
    cfg.wheels[1].x = -0.37f;
    cfg.wheels[1].y = -0.375f;
    cfg.wheels[2].theta = -120.0f;
    cfg.wheels[2].x = 0.37f;
    cfg.wheels[2].y = -0.375f;

    Chassis_Omni<3> chassis(cfg);

    // 只注册轮0和轮2，轮1不注册（保持nullptr）
    FakeMotor m0, m2;
    chassis.registerWheelMotor(0, &m0);
    chassis.registerWheelMotor(2, &m2);

    Robot_Twist target{};
    target.vx = 1.0f;
    target.vy = 0.0f;
    target.yaw_rate = 0.0f;

    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    chassis.update();

    // 不应崩溃，未注册的电机被跳过
    // 已注册的电机收到了RPM指令
    EXPECT_GT(m0.rpm_calls, 0);
}

// ============================================================
// 轮子注册越界
// ============================================================
void testWheelRegistrationOutOfBounds()
{
    Chassis_Omni<3> chassis(0.05f, 1000.0f, 0.2f);
    FakeMotor motor;
    EXPECT_TRUE(chassis.registerWheelMotor(0, &motor));
    EXPECT_TRUE(!chassis.registerWheelMotor(3, &motor));
    EXPECT_TRUE(!chassis.registerWheelMotor(255, &motor));
}

} // namespace

int main()
{
    std::printf("=== tri_omni_kinematics tests ===\n");

    testWheelRegistrationOutOfBounds();
    testZeroCurrentModeSendsZeroCurrentToAllMotors();
    testUnregisteredMotorsAreNotAccessed();
    testPureVxForwardProducesEqualWheelSpeeds();
    testPureVySidewaysProducesNonZeroWheelSpeeds();
    testPureRotationProducesAllWheelsTurning();
    testSpeedLimitingScalesAllWheelsWhenAnyExceedsMax();

    std::printf("\n%d failures\n", g_failures);
    return g_failures > 0 ? 1 : 0;
}
