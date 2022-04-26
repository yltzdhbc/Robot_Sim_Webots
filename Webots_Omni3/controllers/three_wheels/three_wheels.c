
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32

int main()
{
  /* 初始化 */
  wb_robot_init();
  /* 加载电机驱动器 */
  WbDeviceTag wheel1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel2 = wb_robot_get_device("wheel2");
  WbDeviceTag wheel3 = wb_robot_get_device("wheel3");
  wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_position(wheel2, INFINITY);
  wb_motor_set_position(wheel3, INFINITY);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    wb_motor_set_velocity(wheel1,0.0);
    wb_motor_set_velocity(wheel2,-1.0);
    wb_motor_set_velocity(wheel3,1.0);
  }
  /* 结束时释放webots内存 */
  wb_robot_cleanup();

  return 0;
}
