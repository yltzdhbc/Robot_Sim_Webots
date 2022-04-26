#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/inertial_unit.h>
#include <webots/receiver.h>
#include <webots/lidar.h>
#include <time.h>

#include "movebase.h"

#define TIME_STEP 20

int main()
{
  /* 初始化 */
  wb_robot_init();

  /* 加载电机驱动器 */
  WbDeviceTag wheel1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel2 = wb_robot_get_device("wheel2");
  WbDeviceTag wheel3 = wb_robot_get_device("wheel3");
  WbDeviceTag wheel4 = wb_robot_get_device("wheel4");
  wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_position(wheel2, INFINITY);
  wb_motor_set_position(wheel3, INFINITY);
  wb_motor_set_position(wheel4, INFINITY);
  wb_motor_set_velocity(wheel1, 0);
  wb_motor_set_velocity(wheel2, 0);
  wb_motor_set_velocity(wheel3, 0);
  wb_motor_set_velocity(wheel4, 0);

  /* 加载惯性单元 获取 YAW 轴数据 */
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(inertial_unit, 1); // 0,1,2 YAW Y轴

  /* 加载GPS 获取 x,y 数据 */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    Output_Wheel(0.8, 0.0, 0.0); //输出速度到轮子
  }

  /* 结束时释放webots内存 */
  wb_robot_cleanup();

  return 0;
}
