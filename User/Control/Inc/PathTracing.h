/**
 * @file PathTracing.h
 * @author Zhang Hongli
 * @brief ·��������ͷ�ļ�������Pure Pursuit�㷨ʵ��
 * @version 1.0
 */


#ifndef PATH_TRACING_H
#define PATH_TRACING_H

#include <math.h>
// ��ѧ��������
static const float PT_PI = 3.14159265358979323846f;
// ·����ṹ - �洢��������ϵ�е�·������Ϣ
typedef struct {
    float x;        // ��������ϵx���꣨�ף�
    float y;        // ��������ϵy���꣨�ף�
    float theta;    // ��������Ƕȣ����ȣ�
} Waypoint;

// ������״̬ - �洢�����˵�ǰ״̬��Ϣ
typedef struct {
    float current_x;        // ��ǰx���꣨�ף�
    float current_y;        // ��ǰy���꣨�ף�
    float current_theta;    // ��ǰ����Ƕȣ����ȣ�
    float linear_velocity;  // ��ǰ���ٶȣ���/�룩
    float angular_velocity; // ��ǰ���ٶȣ�����/�룩
} RobotState;

// ·���������ò���
typedef struct {
    float max_linear_velocity;     // ������ٶȣ���/�룩
    float max_angular_velocity;    // �����ٶȣ�����/�룩
    float linear_acceleration;     // �߼��ٶȣ���/��?��
    float angular_acceleration;    // �Ǽ��ٶȣ�����/��?��
    float goal_tolerance;          // Ŀ����ݲ�ף�
    float lookahead_distance;      // ǰ�Ӿ��루�ף�
} PathTracingConfig;

class PathTracing {
private:
    Waypoint* waypoints_;                  // ·��������
    unsigned int max_waypoints_;           // ���·������
    unsigned int current_waypoint_count_;  // ��ǰ·������
    unsigned int current_target_index_;    // ��ǰĿ�������

    RobotState robot_state_;               // �����˵�ǰ״̬
    PathTracingConfig config_;             // �������ò���
    // ��һʱ�̽��ٶȣ����ڽ��ٶȼ��ٶ����ƣ�ÿ��ʵ��һ�ݣ�
    float last_angular_velocity_;          // �洢��һ�������ڵĽ��ٶ�

    // ˽�з���
    float calculateDistance(float x1, float y1, float x2, float y2); // ������������
    float normalizeAngle(float angle);     // �Ƕȹ�һ����[-��, ��]
    float calculateAngleToTarget(float target_x, float target_y); // ���㵽Ŀ���ĽǶ�
    bool isGoalReached(float target_x, float target_y);           // ����Ƿ񵽴�Ŀ���
    void purePursuitControl(float target_x, float target_y);      // Pure Pursuit�����㷨

public:
    PathTracing();  // Ĭ�Ϲ��캯��
    explicit PathTracing(unsigned int max_points); // ָ�����·�������Ĺ��캯��
    PathTracing(Waypoint* buffer, unsigned int max_points); // ʹ���ⲿ�������Ĺ��캯��
    ~PathTracing(); // ��������

    bool init(Waypoint* buffer, unsigned int max_points); // ��ʼ��·��������

    // ·������
    bool addWaypoint(float x, float y, float theta); // ����·����
    bool clearWaypoints();                           // �������·����
    unsigned int getWaypointCount();                 // ��ȡ·��������

    // ���ù���
    void setConfig(float max_linear_vel, float max_angular_vel,
                   float linear_accel, float angular_accel,
                   float tolerance, float lookahead); // �������ò���
    PathTracingConfig getConfig();                    // ��ȡ��ǰ����

    // ״̬����
    void setRobotState(float x, float y, float theta); // ���û�����״̬
    RobotState getRobotState();                        // ��ȡ������״̬

    // ·�����ٿ���
    bool planPath();                       // "�滮"·����ʵ���ǳ�ʼ������״̬��
    void executeOneStep(float dt_seconds); // ִ��һ�����ٿ���
    bool isPathCompleted();                // ���·���Ƿ����

    // �˶��������
    void calculateMotionCommands(float* linear_vel, float* angular_vel); // ��ȡ�˶���������
    Waypoint getCurrentTarget();          // ��ȡ��ǰĿ���
    float getPathLength();                // ����·���ܳ���
};

#endif // PATH_TRACING_H