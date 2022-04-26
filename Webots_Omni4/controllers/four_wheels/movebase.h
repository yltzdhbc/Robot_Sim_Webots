#ifndef __MOVEBASE__H
#define __MOVEBASE__H

//轮子到中心距离
#define WHEEL_DISTANCE_TO_CENTER 0.32f

void Output_Wheel(float vel, float direction, float omega);
void chassis_vector_to_wheel_speed(float vx_set, float vy_set, float wz_set);
#endif