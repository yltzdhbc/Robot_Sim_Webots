#include <webots/robot.h>
#include <webots/motor.h>
// #include <webots/inertial_unit.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "movebase.h"

//*****************      m/s          m/s         rad/s
void Output_Wheel(float vel_x, float vel_y, float omega)
{
  chassis_vector_to_wheel_speed(vel_x, vel_y, omega);
}

void chassis_vector_to_wheel_speed(float vx_set, float vy_set, float wz_set)
{
  //将三角函数计算定义为静态变量写入内存中减少计算量
  static float cos_45 = 0.707106781, sin_45 = 0.707106781;
  float wheel_speed[4] = {0.00f};

  //根据运动关系进行解算
  wheel_speed[0] = +vx_set * cos_45 + vy_set * sin_45 + wz_set * WHEEL_DISTANCE_TO_CENTER;
  wheel_speed[1] = -vx_set * sin_45 + vy_set * cos_45 + wz_set * WHEEL_DISTANCE_TO_CENTER;
  wheel_speed[2] = -vx_set * sin_45 - vy_set * cos_45 + wz_set * WHEEL_DISTANCE_TO_CENTER;
  wheel_speed[3] = +vx_set * cos_45 - vy_set * sin_45 + wz_set * WHEEL_DISTANCE_TO_CENTER;

  WbDeviceTag wheel1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel2 = wb_robot_get_device("wheel2");
  WbDeviceTag wheel3 = wb_robot_get_device("wheel3");
  WbDeviceTag wheel4 = wb_robot_get_device("wheel4");
  wb_motor_set_velocity(wheel1, wheel_speed[0]);
  wb_motor_set_velocity(wheel2, wheel_speed[1]);
  wb_motor_set_velocity(wheel3, wheel_speed[2]);
  wb_motor_set_velocity(wheel4, wheel_speed[3]);
}