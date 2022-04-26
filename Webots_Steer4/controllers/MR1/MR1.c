#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/inertial_unit.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "movebase.h"

#define TIME_STEP 20

int main(int argc, char **argv)
{
  /* 初始化 */
  wb_robot_init();

  /* 加载惯性单元 获取 YAW 轴数据 */
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(inertial_unit, 1); //0,1,2 YAW Y轴

  /* 加载GPS 获取 x,y 数据 */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    Output_Wheel(0.8, 0.8, -0.001); //输出速度到轮子
  };

  /* 结束时释放webots内存 */
  wb_robot_cleanup();

  return 0;
}
