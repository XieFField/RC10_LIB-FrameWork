#ifndef __MODULE_CHASSISSWERVE_H
#define __MODULE_CHASSISSWERVE_H

#include <cstddef>

namespace jia
{
namespace swerve
{
constexpr std::size_t kModuleCount = 4;
constexpr float kPi = 3.14159265358979323846f;

enum class IdlePostureMode
{
    kHoldLast,
    kXPark,
};

enum class HomingState
{
    kIdle,
    kSearch,
    kEdgeDetected,
    kOffsetApply,
    kContinuousAngleReady,
    kReady,
    kFault,
};

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;
};

struct WheelGeometry
{
    float pos_x_m = 0.0f;
    float pos_y_m = 0.0f;
    float theta_oa_to_owi_rad = 0.0f;
    float steer_motor_sign = 1.0f;
    float drive_motor_sign = 1.0f;
};

struct HomingConfig
{
    bool enabled = true;
    bool sensor_active_high = true;
    bool simulate_bounce = false;
    float bounce_duration_s = 0.0f;
    float search_speed_rad_s = 1.0f;
    float trigger_angle_local_rad = 0.0f;
    float trigger_window_rad = 2.0f * kPi / 180.0f;
    float zero_offset_local_rad = 0.0f;
    float timeout_s = 5.0f;
};

struct SharedLimits
{
    float wheel_radius_m = 0.075f;
    float max_drive_omega_rad_s = 30.0f;
    float max_drive_alpha_rad_s2 = 120.0f;
    float max_steer_rate_rad_s = 8.0f;
    float max_steer_alpha_rad_s2 = 60.0f;
    float stationary_speed_epsilon_m_s = 0.01f;
    bool enable_cosine_compensation = true;
    bool enable_actuator_dynamics = true;
};

struct SwerveModuleConfig
{
    WheelGeometry geometry;
    HomingConfig homing;
};

struct SwerveConfig
{
    SharedLimits shared;
    IdlePostureMode idle_posture_mode = IdlePostureMode::kHoldLast;
    SwerveModuleConfig modules[kModuleCount];
};

struct ChassisCommand
{
    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
};

struct ModuleFeedback
{
    float steer_motor_total_angle_rad = 0.0f;
    float drive_omega_rad_s = 0.0f;
};

struct ModuleCommand
{
    float wheel_velocity_oa_x_m_s = 0.0f;
    float wheel_velocity_oa_y_m_s = 0.0f;
    float raw_target_steer_angle_oa_rad = 0.0f;
    float alt_target_steer_angle_oa_rad = 0.0f;
    float selected_target_steer_oa_total_angle_rad = 0.0f;
    float selected_target_steer_motor_total_angle_rad = 0.0f;
    float selected_target_steer_motor_mod_angle_rad = 0.0f;
    float selected_target_drive_omega_rad_s = 0.0f;
    float steer_target_velocity_rad_s = 0.0f;
    float cosine_scale = 1.0f;
    bool flipped_drive_direction = false;
};

struct HomingSensorSample
{
    bool active = false;
    bool start_request = false;
};

struct HomingTracker
{
    HomingState state = HomingState::kIdle;
    bool last_sensor_active = false;
    bool zero_offset_valid = false;
    float elapsed_s = 0.0f;
    float continuous_zero_offset_rad = 0.0f;
    float last_edge_measured_angle_rad = 0.0f;
};

struct ModuleSnapshot
{
    ModuleFeedback feedback;
    ModuleCommand command;
    HomingTracker homing;
    bool sensor_active = false;
};

struct SimulationStepRecord
{
    float time_s = 0.0f;
    ChassisCommand input_command;
    ChassisCommand estimated_body_motion;
    bool homing_all_ready = false;
    ModuleSnapshot modules[kModuleCount];
};

float wrapToPi(float angle_rad);
float wrapTo2Pi(float angle_rad);
float clampValue(float value, float min_value, float max_value);
float shortestAngularDistance(float from_rad, float to_rad);
float nearestEquivalentAngle(float current_rad, float target_mod_rad);
float magnitude(const Vec2& value);
float makeXParkAngle(const WheelGeometry& geometry);

bool estimateChassisMotion(const SwerveConfig& config,
                           const ModuleFeedback feedback[kModuleCount],
                           ChassisCommand* out_motion);

const char* toString(HomingState state);
bool isHomingReady(const HomingTracker& tracker);
void resetHomingTracker(HomingTracker* tracker);
float applyHomingCorrection(const HomingTracker& tracker, float raw_local_angle_rad);
float updateHomingTracker(HomingTracker* tracker,
                          const HomingConfig& config,
                          const HomingSensorSample& sensor,
                          float raw_local_angle_rad,
                          float dt_s);

class SwerveController
{
public:
    explicit SwerveController(const SwerveConfig& config);

    void configure(const SwerveConfig& config);
    void reset();
    void step(const ChassisCommand& command,
              const ModuleFeedback feedback[kModuleCount],
              float dt_s,
              ModuleCommand out_commands[kModuleCount]);

    const SwerveConfig& config() const { return config_; }

private:
    float limitPositionSecondOrder(float current_value,
                                   float current_rate,
                                   float target_value,
                                   float max_rate,
                                   float max_accel,
                                   float dt_s,
                                   float* next_rate) const;

    float limitValueWithAcceleration(float current_value,
                                     float target_value,
                                     float max_accel,
                                     float dt_s) const;

    SwerveConfig config_;
    float last_steer_rate_cmd_rad_s_[kModuleCount] = {0.0f};
    float last_drive_omega_cmd_rad_s_[kModuleCount] = {0.0f};
};

} // namespace swerve
} // namespace jia

#endif // __MODULE_CHASSISSWERVE_H
