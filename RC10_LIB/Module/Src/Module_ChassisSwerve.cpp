#include "Module_ChassisSwerve.h"

#include <cmath>

namespace jia
{
namespace swerve
{
namespace
{
float safeDt(float dt_s)
{
    if (dt_s <= 1.0e-6f)
    {
        return 1.0e-3f;
    }
    return dt_s;
}

bool solve3x3(float matrix[3][4], float* x0, float* x1, float* x2)
{
    for (int pivot = 0; pivot < 3; ++pivot)
    {
        int best_row = pivot;
        float best_abs = std::fabs(matrix[pivot][pivot]);
        for (int row = pivot + 1; row < 3; ++row)
        {
            const float abs_value = std::fabs(matrix[row][pivot]);
            if (abs_value > best_abs)
            {
                best_abs = abs_value;
                best_row = row;
            }
        }

        if (best_abs <= 1.0e-6f)
        {
            return false;
        }

        if (best_row != pivot)
        {
            for (int column = pivot; column < 4; ++column)
            {
                const float temp = matrix[pivot][column];
                matrix[pivot][column] = matrix[best_row][column];
                matrix[best_row][column] = temp;
            }
        }

        const float diagonal = matrix[pivot][pivot];
        for (int column = pivot; column < 4; ++column)
        {
            matrix[pivot][column] /= diagonal;
        }

        for (int row = 0; row < 3; ++row)
        {
            if (row == pivot)
            {
                continue;
            }

            const float factor = matrix[row][pivot];
            if (std::fabs(factor) <= 1.0e-8f)
            {
                continue;
            }

            for (int column = pivot; column < 4; ++column)
            {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
        }
    }

    *x0 = matrix[0][3];
    *x1 = matrix[1][3];
    *x2 = matrix[2][3];
    return true;
}
} // namespace

float wrapToPi(float angle_rad)
{
    while (angle_rad >= kPi)
    {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi)
    {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

float wrapTo2Pi(float angle_rad)
{
    while (angle_rad >= 2.0f * kPi)
    {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < 0.0f)
    {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

float clampValue(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

float shortestAngularDistance(float from_rad, float to_rad)
{
    return wrapToPi(to_rad - from_rad);
}

float nearestEquivalentAngle(float current_rad, float target_mod_rad)
{
    return current_rad + shortestAngularDistance(current_rad, target_mod_rad);
}

float magnitude(const Vec2& value)
{
    return std::sqrt(value.x * value.x + value.y * value.y);
}

float makeXParkAngle(const WheelGeometry& geometry)
{
    return std::atan2(geometry.pos_y_m, geometry.pos_x_m);
}

bool estimateChassisMotion(const SwerveConfig& config,
                           const ModuleFeedback feedback[kModuleCount],
                           ChassisCommand* out_motion)
{
    if (out_motion == nullptr)
    {
        return false;
    }

    float normal[3][3] = {};
    float rhs[3] = {};

    for (std::size_t module_index = 0; module_index < kModuleCount; ++module_index)
    {
        const WheelGeometry& geometry = config.modules[module_index].geometry;
        const float steer_angle_oa_rad = feedback[module_index].steer_motor_total_angle_rad +
                                         geometry.theta_oa_to_owi_rad;
        const float cos_theta = std::cos(steer_angle_oa_rad);
        const float sin_theta = std::sin(steer_angle_oa_rad);
        const float drive_linear_m_s = feedback[module_index].drive_omega_rad_s *
                                       config.shared.wheel_radius_m;

        const float rows[2][3] = {
            {cos_theta, sin_theta, -geometry.pos_y_m * cos_theta + geometry.pos_x_m * sin_theta},
            {-sin_theta, cos_theta, geometry.pos_y_m * sin_theta + geometry.pos_x_m * cos_theta},
        };
        const float measurements[2] = {drive_linear_m_s, 0.0f};

        for (int row = 0; row < 2; ++row)
        {
            for (int i = 0; i < 3; ++i)
            {
                rhs[i] += rows[row][i] * measurements[row];
                for (int j = 0; j < 3; ++j)
                {
                    normal[i][j] += rows[row][i] * rows[row][j];
                }
            }
        }
    }

    float augmented[3][4] = {
        {normal[0][0], normal[0][1], normal[0][2], rhs[0]},
        {normal[1][0], normal[1][1], normal[1][2], rhs[1]},
        {normal[2][0], normal[2][1], normal[2][2], rhs[2]},
    };

    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
    if (!solve3x3(augmented, &vx_m_s, &vy_m_s, &wz_rad_s))
    {
        *out_motion = {};
        return false;
    }

    out_motion->vx_m_s = vx_m_s;
    out_motion->vy_m_s = vy_m_s;
    out_motion->wz_rad_s = wz_rad_s;
    return true;
}

const char* toString(HomingState state)
{
    switch (state)
    {
    case HomingState::kIdle:
        return "Idle";
    case HomingState::kSearch:
        return "Search";
    case HomingState::kEdgeDetected:
        return "EdgeDetected";
    case HomingState::kOffsetApply:
        return "OffsetApply";
    case HomingState::kContinuousAngleReady:
        return "ContinuousAngleReady";
    case HomingState::kReady:
        return "Ready";
    case HomingState::kFault:
        return "Fault";
    default:
        return "Unknown";
    }
}

bool isHomingReady(const HomingTracker& tracker)
{
    return tracker.state == HomingState::kReady;
}

void resetHomingTracker(HomingTracker* tracker)
{
    if (tracker != nullptr)
    {
        *tracker = {};
    }
}

float applyHomingCorrection(const HomingTracker& tracker, float raw_local_angle_rad)
{
    if (tracker.zero_offset_valid)
    {
        return raw_local_angle_rad + tracker.continuous_zero_offset_rad;
    }
    return raw_local_angle_rad;
}

float updateHomingTracker(HomingTracker* tracker,
                          const HomingConfig& config,
                          const HomingSensorSample& sensor,
                          float raw_local_angle_rad,
                          float dt_s)
{
    if (tracker == nullptr)
    {
        return 0.0f;
    }

    if (!config.enabled)
    {
        tracker->state = HomingState::kReady;
        tracker->zero_offset_valid = true;
        tracker->continuous_zero_offset_rad = 0.0f;
        tracker->last_sensor_active = sensor.active;
        return 0.0f;
    }

    if (tracker->state == HomingState::kIdle)
    {
        if (sensor.start_request)
        {
            tracker->state = HomingState::kSearch;
            tracker->elapsed_s = 0.0f;
        }
        tracker->last_sensor_active = sensor.active;
        return 0.0f;
    }

    if (tracker->state == HomingState::kSearch)
    {
        tracker->elapsed_s += dt_s;
        if (tracker->elapsed_s > config.timeout_s)
        {
            tracker->state = HomingState::kFault;
            tracker->last_sensor_active = sensor.active;
            return 0.0f;
        }

        bool detected = false;
        if (sensor.active != tracker->last_sensor_active)
        {
            detected = true;
        }
        if (sensor.active && tracker->elapsed_s <= dt_s + 1.0e-6f)
        {
            detected = true;
        }

        if (detected)
        {
            tracker->state = HomingState::kEdgeDetected;
            tracker->last_edge_measured_angle_rad = raw_local_angle_rad;
            tracker->continuous_zero_offset_rad = config.zero_offset_local_rad - raw_local_angle_rad;
            tracker->zero_offset_valid = true;
            tracker->last_sensor_active = sensor.active;
            return 0.0f;
        }

        tracker->last_sensor_active = sensor.active;
        return config.search_speed_rad_s;
    }

    if (tracker->state == HomingState::kEdgeDetected)
    {
        tracker->state = HomingState::kOffsetApply;
        tracker->last_sensor_active = sensor.active;
        return 0.0f;
    }

    if (tracker->state == HomingState::kOffsetApply)
    {
        tracker->state = HomingState::kContinuousAngleReady;
        tracker->last_sensor_active = sensor.active;
        return 0.0f;
    }

    if (tracker->state == HomingState::kContinuousAngleReady)
    {
        tracker->state = HomingState::kReady;
        tracker->last_sensor_active = sensor.active;
        return 0.0f;
    }

    tracker->last_sensor_active = sensor.active;
    return 0.0f;
}

SwerveController::SwerveController(const SwerveConfig& config)
    : config_(config)
{
}

void SwerveController::configure(const SwerveConfig& config)
{
    config_ = config;
    reset();
}

void SwerveController::reset()
{
    for (std::size_t module_index = 0; module_index < kModuleCount; ++module_index)
    {
        last_steer_rate_cmd_rad_s_[module_index] = 0.0f;
        last_drive_omega_cmd_rad_s_[module_index] = 0.0f;
    }
}

float SwerveController::limitPositionSecondOrder(float current_value,
                                                 float current_rate,
                                                 float target_value,
                                                 float max_rate,
                                                 float max_accel,
                                                 float dt_s,
                                                 float* next_rate) const
{
    const float fixed_dt_s = safeDt(dt_s);
    const float delta_value = target_value - current_value;
    const float desired_rate = clampValue(delta_value / fixed_dt_s, -max_rate, max_rate);
    const float rate_delta_limit = max_accel * fixed_dt_s;
    float limited_rate = current_rate;

    if (desired_rate > limited_rate + rate_delta_limit)
    {
        limited_rate += rate_delta_limit;
    }
    else if (desired_rate < limited_rate - rate_delta_limit)
    {
        limited_rate -= rate_delta_limit;
    }
    else
    {
        limited_rate = desired_rate;
    }

    limited_rate = clampValue(limited_rate, -max_rate, max_rate);

    float step_value = limited_rate * fixed_dt_s;
    if (std::fabs(step_value) > std::fabs(delta_value))
    {
        step_value = delta_value;
        limited_rate = step_value / fixed_dt_s;
    }

    if (next_rate != nullptr)
    {
        *next_rate = limited_rate;
    }
    return current_value + step_value;
}

float SwerveController::limitValueWithAcceleration(float current_value,
                                                   float target_value,
                                                   float max_accel,
                                                   float dt_s) const
{
    const float fixed_dt_s = safeDt(dt_s);
    const float delta_limit = max_accel * fixed_dt_s;
    const float delta_value = target_value - current_value;

    if (delta_value > delta_limit)
    {
        return current_value + delta_limit;
    }
    if (delta_value < -delta_limit)
    {
        return current_value - delta_limit;
    }
    return target_value;
}

void SwerveController::step(const ChassisCommand& command,
                            const ModuleFeedback feedback[kModuleCount],
                            float dt_s,
                            ModuleCommand out_commands[kModuleCount])
{
    if (feedback == nullptr || out_commands == nullptr)
    {
        return;
    }

    const float fixed_dt_s = safeDt(dt_s);

    for (std::size_t module_index = 0; module_index < kModuleCount; ++module_index)
    {
        const SwerveModuleConfig& module_config = config_.modules[module_index];
        const WheelGeometry& geometry = module_config.geometry;

        const float current_steer_motor_total_rad = feedback[module_index].steer_motor_total_angle_rad;
        const float current_steer_oa_total_rad = current_steer_motor_total_rad + geometry.theta_oa_to_owi_rad;

        ModuleCommand command_out = {};
        command_out.wheel_velocity_oa_x_m_s = command.vx_m_s - command.wz_rad_s * geometry.pos_y_m;
        command_out.wheel_velocity_oa_y_m_s = command.vy_m_s + command.wz_rad_s * geometry.pos_x_m;

        const Vec2 wheel_velocity{
            command_out.wheel_velocity_oa_x_m_s,
            command_out.wheel_velocity_oa_y_m_s,
        };
        const float wheel_speed_m_s = magnitude(wheel_velocity);

        float desired_steer_oa_mod_rad = 0.0f;
        float desired_drive_omega_rad_s = 0.0f;

        if (wheel_speed_m_s <= config_.shared.stationary_speed_epsilon_m_s)
        {
            if (config_.idle_posture_mode == IdlePostureMode::kXPark)
            {
                desired_steer_oa_mod_rad = wrapTo2Pi(makeXParkAngle(geometry));
            }
            else
            {
                desired_steer_oa_mod_rad = wrapTo2Pi(current_steer_oa_total_rad);
            }
            desired_drive_omega_rad_s = 0.0f;
        }
        else
        {
            desired_steer_oa_mod_rad = wrapTo2Pi(std::atan2(command_out.wheel_velocity_oa_y_m_s,
                                                            command_out.wheel_velocity_oa_x_m_s));
            desired_drive_omega_rad_s = wheel_speed_m_s / config_.shared.wheel_radius_m;
        }

        command_out.raw_target_steer_angle_oa_rad = desired_steer_oa_mod_rad;
        command_out.alt_target_steer_angle_oa_rad = wrapTo2Pi(desired_steer_oa_mod_rad + kPi);

        const float candidate_a_oa_total_rad = nearestEquivalentAngle(current_steer_oa_total_rad,
                                                                      command_out.raw_target_steer_angle_oa_rad);
        const float candidate_b_oa_total_rad = nearestEquivalentAngle(current_steer_oa_total_rad,
                                                                      command_out.alt_target_steer_angle_oa_rad);

        const float delta_a_rad = candidate_a_oa_total_rad - current_steer_oa_total_rad;
        const float delta_b_rad = candidate_b_oa_total_rad - current_steer_oa_total_rad;

        float selected_target_oa_total_rad = candidate_a_oa_total_rad;
        float selected_drive_omega_rad_s = desired_drive_omega_rad_s;
        bool flipped_drive = false;

        if (std::fabs(delta_b_rad) < std::fabs(delta_a_rad))
        {
            selected_target_oa_total_rad = candidate_b_oa_total_rad;
            selected_drive_omega_rad_s = -desired_drive_omega_rad_s;
            flipped_drive = true;
        }

        const float steering_error_rad = shortestAngularDistance(current_steer_oa_total_rad,
                                                                 selected_target_oa_total_rad);
        float cosine_scale = 1.0f;
        if (config_.shared.enable_cosine_compensation)
        {
            cosine_scale = std::cos(std::fabs(steering_error_rad));
            if (cosine_scale < 0.0f)
            {
                cosine_scale = 0.0f;
            }
        }

        selected_drive_omega_rad_s *= cosine_scale;
        selected_drive_omega_rad_s = clampValue(selected_drive_omega_rad_s,
                                                -config_.shared.max_drive_omega_rad_s,
                                                config_.shared.max_drive_omega_rad_s);

        const float selected_target_motor_total_rad = selected_target_oa_total_rad - geometry.theta_oa_to_owi_rad;

        float next_steer_rate_rad_s = 0.0f;
        float next_target_motor_total_rad = selected_target_motor_total_rad;
        float next_drive_omega_rad_s = selected_drive_omega_rad_s;

        if (config_.shared.enable_actuator_dynamics)
        {
            next_target_motor_total_rad = limitPositionSecondOrder(
                current_steer_motor_total_rad,
                last_steer_rate_cmd_rad_s_[module_index],
                selected_target_motor_total_rad,
                config_.shared.max_steer_rate_rad_s,
                config_.shared.max_steer_alpha_rad_s2,
                fixed_dt_s,
                &next_steer_rate_rad_s);

            next_drive_omega_rad_s = clampValue(
                limitValueWithAcceleration(last_drive_omega_cmd_rad_s_[module_index],
                                           selected_drive_omega_rad_s,
                                           config_.shared.max_drive_alpha_rad_s2,
                                           fixed_dt_s),
                -config_.shared.max_drive_omega_rad_s,
                config_.shared.max_drive_omega_rad_s);
        }
        else
        {
            next_steer_rate_rad_s = (next_target_motor_total_rad - current_steer_motor_total_rad) / fixed_dt_s;
        }

        command_out.selected_target_steer_motor_total_angle_rad = next_target_motor_total_rad;
        command_out.selected_target_steer_oa_total_angle_rad = next_target_motor_total_rad +
                                                               geometry.theta_oa_to_owi_rad;
        command_out.selected_target_steer_motor_mod_angle_rad = wrapTo2Pi(next_target_motor_total_rad);
        command_out.selected_target_drive_omega_rad_s = next_drive_omega_rad_s;
        command_out.steer_target_velocity_rad_s = next_steer_rate_rad_s;
        command_out.cosine_scale = cosine_scale;
        command_out.flipped_drive_direction = flipped_drive;

        last_steer_rate_cmd_rad_s_[module_index] = next_steer_rate_rad_s;
        last_drive_omega_cmd_rad_s_[module_index] = next_drive_omega_rad_s;

        out_commands[module_index] = command_out;
    }
}

} // namespace swerve
} // namespace jia
