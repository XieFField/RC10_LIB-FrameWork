#include "yakabin_demo_.h"
#include <cstdio>

int main() 
{
    // 基座在原点，L0 = 0.5 m，伸缩范围 [0, 0.4] m，抬升范围 [-0.1, 0.8] m
    Kinematics kin(0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.4f, -0.1f, 0.8f);

    // 目标点（x,y,z）
    Pose target; target.x = 1.2f; target.y = 0.0f; target.z = 0.2f;

    JointState q;
    bool exact = kin.inverse(target, q);
    if (!exact) {
        printf("target out of range, used saturated joints\n");
    }
    printf("Inverse results: h=%.3f m, theta=%.3f rad, s=%.3f m\n", q.h, q.theta, q.s);

    Pose p = kin.forward(q);
    printf("Forward check: x=%.3f y=%.3f z=%.3f\n", p.x, p.y, p.z);

    // 计算雅可比
    float J[3][3];
    kin.jacobian(q, J);
    printf("J = [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ]\n",
           J[0][0], J[0][1], J[0][2], J[1][0], J[1][1], J[1][2], J[2][0], J[2][1], J[2][2]);
}