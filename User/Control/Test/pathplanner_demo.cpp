#include <stdio.h>
#include "PathPlanner.h"

// 简单 demo: 在主循环中用固定 dt 调用 PathPlanner
int main()
{
    // 静态缓冲区（嵌入式友好）
    static Waypoint buffer[16];

    // 使用带缓冲区构造器
    PathPlanner planner(buffer, 16);

    // 添加一些 waypoint
    planner.addWaypoint(0.0f, 0.0f, 0.0f);
    planner.addWaypoint(1.0f, 0.0f, 0.0f);
    planner.addWaypoint(1.0f, 1.0f, 0.0f);

    // 进行规划
    planner.planPath();

    // 模拟循环
    const float dt = 0.02f; // 20 ms
    for (int i = 0; i < 500; ++i) {
        // 在真实系统中，应从定时器/RTOS获取真实 dt
        planner.executeOneStep(dt);

        RobotState st = planner.getRobotState();
        Waypoint t = planner.getCurrentTarget();
        printf("step %d: pos=(%.3f, %.3f) theta=%.3f lin_vel=%.3f ang_vel=%.3f target=(%.3f,%.3f)\n",
               i, st.current_x, st.current_y, st.current_theta, st.linear_velocity, st.angular_velocity, t.x, t.y);

        if (planner.isPathCompleted()) {
            printf("Path completed at step %d\n", i);
            break;
        }
    }

    return 0;
}
