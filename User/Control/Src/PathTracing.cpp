#include "PathTracing.h"
#include <cmath>




// Ĭ�Ϲ��캯�� - ��ʼ�����г�Ա����
PathTracing::PathTracing() {
    waypoints_ = nullptr;
    max_waypoints_ = 0;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    // ��ʼ��������״̬Ϊ��
    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;  // ��ʼ����һʱ�̽��ٶ�

    // Ĭ�����ò��� - ������С���ƶ�������
    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;     // 5�����ݲ�
    config_.lookahead_distance = 0.3f;  // 30����ǰ�Ӿ���
}

// �����·�������Ĺ��캯��
PathTracing::PathTracing(unsigned int max_points) {
    waypoints_ = nullptr;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// ʹ���ⲿ�������Ĺ��캯�� - ���⶯̬�ڴ����
PathTracing::PathTracing(Waypoint* buffer, unsigned int max_points) {
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// �������� - ������ָ�룬���ͷ��ⲿ�����Ļ�����
PathTracing::~PathTracing() {
    waypoints_ = nullptr;
}

// ��������֮���ŷ����þ���
float PathTracing::calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

// �Ƕȹ�һ�� - ���Ƕ�������[-��, ��]��Χ��
float PathTracing::normalizeAngle(float angle) {
    while (angle > PT_PI) angle -= 2.0f * PT_PI;
    while (angle < -PT_PI) angle += 2.0f * PT_PI;
    return angle;
}

// ����ӵ�ǰλ�õ�Ŀ����ȫ�ֽǶ�
float PathTracing::calculateAngleToTarget(float target_x, float target_y) {
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    return atan2f(dy, dx);  // �����������������ϵ�ĽǶ�
}

// ����Ƿ񵽴�Ŀ��� - ���ھ����ݲ��ж�
bool PathTracing::isGoalReached(float target_x, float target_y) {
    float distance = calculateDistance(robot_state_.current_x, robot_state_.current_y,
                                      target_x, target_y);
    return distance <= config_.goal_tolerance;
}

// Pure Pursuit�����㷨���� - �������Ŀ�������Ľ��ٶ�
void PathTracing::purePursuitControl(float target_x, float target_y) {
    // ���㵽Ŀ��������
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    // ����Ŀ�����ȫ������ϵ�еĽǶ�
    float target_global_angle = atan2f(dy, dx);
    // ����Ƕ�����ǰ������Ŀ�귽��֮��Ĳ�ֵ��
    float alpha = normalizeAngle(target_global_angle - robot_state_.current_theta);

    if (distance_to_target > 0.001f) {
        // Pure Pursuit���Ĺ�ʽ������ = 2*sin(��)/L
        // ���Ц��ǽǶ���L�ǵ�Ŀ���ľ���
        float curvature = 2.0f * sinf(alpha) / fmaxf(distance_to_target, 1e-6f);
        float desired_omega = robot_state_.linear_velocity * curvature;

        // ���ٶ��޷���ȷ�������������ٶ�
        if (desired_omega > config_.max_angular_velocity) desired_omega = config_.max_angular_velocity;
        if (desired_omega < -config_.max_angular_velocity) desired_omega = -config_.max_angular_velocity;
        robot_state_.angular_velocity = desired_omega;
    } else {
        robot_state_.angular_velocity = 0.0f;  // �ǳ��ӽ�Ŀ���ʱֹͣת��
    }
}

// ����·���㵽·��������
bool PathTracing::addWaypoint(float x, float y, float theta) {
    if (waypoints_ == nullptr || max_waypoints_ == 0) return false;  // ������δ��ʼ��
    if (current_waypoint_count_ >= max_waypoints_) return false;     // ·��������

    // �洢·��������
    waypoints_[current_waypoint_count_].x = x;
    waypoints_[current_waypoint_count_].y = y;
    waypoints_[current_waypoint_count_].theta = theta;
    current_waypoint_count_++;
    return true;
}

// �������·���㣬���ø���״̬
bool PathTracing::clearWaypoints() {
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}

// ��ȡ��ǰ·��������
unsigned int PathTracing::getWaypointCount() {
    return current_waypoint_count_;
}

// ����·���������ò���
void PathTracing::setConfig(float max_linear_vel, float max_angular_vel,
                            float linear_accel, float angular_accel,
                            float tolerance, float lookahead) {
    config_.max_linear_velocity = max_linear_vel;
    config_.max_angular_velocity = max_angular_vel;
    config_.linear_acceleration = linear_accel;
    config_.angular_acceleration = angular_accel;
    config_.goal_tolerance = tolerance;
    config_.lookahead_distance = lookahead;
}

// ��ȡ��ǰ����
PathTracingConfig PathTracing::getConfig() {
    return config_;
}

// ���û����˵�ǰ״̬
void PathTracing::setRobotState(float x, float y, float theta) {
    robot_state_.current_x = x;
    robot_state_.current_y = y;
    robot_state_.current_theta = normalizeAngle(theta);  // ��һ���Ƕ�
}

// ��ȡ�����˵�ǰ״̬
RobotState PathTracing::getRobotState() {
    return robot_state_;
}

// "�滮"·�� - ʵ���ǳ�ʼ������״̬���ӵ�һ��·���㿪ʼ����
bool PathTracing::planPath() {
    if (current_waypoint_count_ == 0) return false;  // û��·�������
    current_target_index_ = 0;  // �ӵ�һ��·���㿪ʼ����
    return true;
}

// ִ��һ��·�����ٿ��� - ���Ŀ���ѭ��
void PathTracing::executeOneStep(float dt_seconds) {
    // ����Ƿ���·������Ƿ����������·����
    if (current_waypoint_count_ == 0 || current_target_index_ >= current_waypoint_count_) return;

    Waypoint current_target = waypoints_[current_target_index_];
    
    // ����Ƿ񵽴ﵱǰĿ��㣬����������л�����һ��Ŀ���
    if (isGoalReached(current_target.x, current_target.y)) {
        current_target_index_++;
        // ����Ƿ��������·����
        if (current_target_index_ >= current_waypoint_count_) {
            robot_state_.linear_velocity = 0.0f;
            robot_state_.angular_velocity = 0.0f;
            return;  // ·����ɣ�ֹͣ�˶�
        }
        current_target = waypoints_[current_target_index_];  // ���µ�ǰĿ���
    }

    if (dt_seconds <= 0.0f) return;  // ��Чʱ�䲽��

    // ǰ�ӵ���㣺��·���ӵ�ǰĿ�����ǰѰ�Ҿ����ǰ�Ӿ���ĵ�
    float lookahead = config_.lookahead_distance;
    float accum = 0.0f;
    uint32_t idx = current_target_index_;
    float lx = waypoints_[idx].x;
    float ly = waypoints_[idx].y;
    
    // ����·���㣬�ۼӾ���ֱ���ﵽǰ�Ӿ���
    while (idx + 1 < current_waypoint_count_ && accum < lookahead) {
        float seg = calculateDistance(waypoints_[idx].x, waypoints_[idx].y, 
                                     waypoints_[idx+1].x, waypoints_[idx+1].y);
        accum += seg;
        idx++;
        lx = waypoints_[idx].x;
        ly = waypoints_[idx].y;
    }

    // ���ھ�������Ŀ��Ľӽ��̶ȵ����������ٶȣ��ӽ�ʱ���٣�
    float dist_to_goal = calculateDistance(robot_state_.current_x, robot_state_.current_y, 
                                          waypoints_[current_waypoint_count_-1].x, 
                                          waypoints_[current_waypoint_count_-1].y);
    float desired_linear_vel = config_.max_linear_velocity;
    
    // �ӽ�����Ŀ��ʱ����
    if (dist_to_goal < config_.goal_tolerance * 10.0f) {
        desired_linear_vel = config_.max_linear_velocity * 0.3f;
    }

    // ���ݵ�ǰ���ٶ��������ٶȵı�����һ���������ٶȣ�ת��ʱ���٣�
    if (fabsf(robot_state_.angular_velocity) > config_.max_angular_velocity * 0.5f) {
        desired_linear_vel *= 0.7f;
    }

    // ���ٶȼ��ٶ����� - ȷ���ٶ�ƽ���仯
    if (desired_linear_vel > robot_state_.linear_velocity) 
    {
        robot_state_.linear_velocity += config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity > desired_linear_vel) robot_state_.linear_velocity = desired_linear_vel;
    } else if (desired_linear_vel < robot_state_.linear_velocity) {
        robot_state_.linear_velocity -= config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity < desired_linear_vel) robot_state_.linear_velocity = desired_linear_vel;
    }

    // ʹ��ǰ�ӵ����Pure Pursuit���ٶ�
    purePursuitControl(lx, ly);

    // ���ٶȱ仯�ʣ����ٶȣ����� - ȷ�����ٶ�ƽ���仯�����⼱ת
    {
        float max_omega_delta = config_.angular_acceleration * dt_seconds;
        float target_omega = robot_state_.angular_velocity;  // Pure Pursuit�����Ŀ����ٶ�
        float delta = target_omega - last_angular_velocity_; // ���ٶȱ仯��
        
        // ���ƽ��ٶȱ仯��
        if (delta > max_omega_delta) delta = max_omega_delta;
        if (delta < -max_omega_delta) delta = -max_omega_delta;
        
        robot_state_.angular_velocity = last_angular_velocity_ + delta;  // Ӧ�����ƺ�Ľ��ٶ�
        last_angular_velocity_ = robot_state_.angular_velocity;          // ������һʱ�̽��ٶ�
    }
}

// ���·���Ƿ�����ɸ���
bool PathTracing::isPathCompleted() {
    return current_target_index_ >= current_waypoint_count_;
}

// ��ȡ��ǰ�˶���������
void PathTracing::calculateMotionCommands(float* linear_vel, float* angular_vel) {
    if (linear_vel) *linear_vel = robot_state_.linear_velocity;
    if (angular_vel) *angular_vel = robot_state_.angular_velocity;
}

// ��ȡ��ǰ���ٵ�Ŀ���
Waypoint PathTracing::getCurrentTarget() {
    if (current_target_index_ < current_waypoint_count_)
        return waypoints_[current_target_index_];
    else if (current_waypoint_count_ > 0)
        return waypoints_[current_waypoint_count_ - 1];  // �������һ����
    return Waypoint{0, 0, 0};  // Ĭ�Ͽյ�
}

// ����·���ܳ��� - �ۼ�����·���γ���
float PathTracing::getPathLength() {
    float total = 0.0f;
    for (unsigned int i = 1; i < current_waypoint_count_; ++i)
        total += calculateDistance(waypoints_[i - 1].x, waypoints_[i - 1].y,
                                   waypoints_[i].x, waypoints_[i].y);
    return total;
}

// ��ʼ��·�������� - �����ⲿ�ṩ��·���㻺����
bool PathTracing::init(Waypoint* buffer, unsigned int max_points) {
    if (!buffer || !max_points) return false;  // ��������Ч
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}