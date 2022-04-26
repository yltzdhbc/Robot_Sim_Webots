// #include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "movebase.h"

//*****************      m/s          m/s         rad/s
void Output_Wheel(float vel_x, float vel_y, float omega)
{
	wheel_t output_wheel;
	//计算各个轮子速度方向
	output_wheel.leftFront = CalcWheel(vel_x, vel_y, omega, LEFT_FRONT_VERTICAL_ANG, GetAngle());
	output_wheel.rightFront = CalcWheel(vel_x, vel_y, omega, RIGHT_FRONT_VERTICAL_ANG, GetAngle());
	output_wheel.leftRear = CalcWheel(vel_x, vel_y, omega, LEFT_REAR_VERTICAL_ANG, GetAngle());
	output_wheel.rightRear = CalcWheel(vel_x, vel_y, omega, RIGHT_REAR_VERTICAL_ANG, GetAngle());
	//输出
	Wheel_Control(output_wheel);
}

// ******************       m/s          m/s         rad/s           度             度
wheelVel_t CalcWheel(float vel_x, float vel_y, float omega, float angleN, float postureAngle)
{
	wheelVel_t sumVel = {0.0f};
	float velX, velY = 0.0f;
	float velN, velNDirection = 0.0f;
	float sumVelX, sumVelY = 0.0f;

	//计算平移速度的X，Y分量
	velX = vel_x;
	velY = vel_y;
	//计算旋转的线速度
	velN = omega * MOVEBASE_RADIUS;

	velNDirection = angleN + postureAngle;
	AngleLimit(&velNDirection);

	//计算和速度大小和方向
	sumVelX = velX + velN * cosf(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * sinf(ANGLE2RAD(velNDirection));

	sumVel.vel = sqrt(sumVelX * sumVelX + sumVelY * sumVelY);

	//计算合成速度方向时 未将0向量单独处理
	sumVel.direction = RAD2ANGLE(atan2f(sumVelY, sumVelX));

	return sumVel;
}

void Wheel_Control(wheel_t wheelVel)
{
	static float leftFrontAng = 0.0f, rightFrontAng = 0.0f, leftRearAng = 0.0f, rightRearAng = 0.0f;
	//将定位系统坐标系下角度转换为机器人坐标系下角度 direction-=GetAngle()
	Transform2RobotCoodinate(&wheelVel);
	//将机器人坐标系下角度转换为和电机一致 direction = 90.0f - direction
	Transform2WheelCoodinate(&wheelVel);
	//判断是否需要将轮速反向
	JudgeVelDirection(&wheelVel.leftFront, leftFrontAng);
	JudgeVelDirection(&wheelVel.rightFront, rightFrontAng);
	JudgeVelDirection(&wheelVel.leftRear, leftRearAng);
	JudgeVelDirection(&wheelVel.rightRear, rightRearAng);
	//保证旋转为劣弧
	leftFrontAng = TurnInferiorArc(wheelVel.leftFront.direction, leftFrontAng);
	rightFrontAng = TurnInferiorArc(wheelVel.rightFront.direction, rightFrontAng);
	leftRearAng = TurnInferiorArc(wheelVel.leftRear.direction, leftRearAng);
	rightRearAng = TurnInferiorArc(wheelVel.rightRear.direction, rightRearAng);

	WbDeviceTag lf_dir_motor = wb_robot_get_device("lf_dir_motor");
	WbDeviceTag lf_vel_motor = wb_robot_get_device("lf_vel_motor");
	WbDeviceTag lb_dir_motor = wb_robot_get_device("lb_dir_motor");
	WbDeviceTag lb_vel_motor = wb_robot_get_device("lb_vel_motor");
	WbDeviceTag rf_dir_motor = wb_robot_get_device("rf_dir_motor");
	WbDeviceTag rf_vel_motor = wb_robot_get_device("rf_vel_motor");
	WbDeviceTag rb_dir_motor = wb_robot_get_device("rb_dir_motor");
	WbDeviceTag rb_vel_motor = wb_robot_get_device("rb_vel_motor");

	wb_motor_set_position(lf_vel_motor, INFINITY);
	wb_motor_set_position(lb_vel_motor, INFINITY);
	wb_motor_set_position(rf_vel_motor, INFINITY);
	wb_motor_set_position(rb_vel_motor, INFINITY);

	wb_motor_set_velocity(lf_vel_motor, wheelVel.leftFront.vel);
	wb_motor_set_velocity(rf_vel_motor, wheelVel.rightFront.vel);
	wb_motor_set_velocity(lb_vel_motor, wheelVel.leftRear.vel);
	wb_motor_set_velocity(rb_vel_motor, wheelVel.rightRear.vel);

	wb_motor_set_position(lf_dir_motor, ANGLE2RAD(leftFrontAng));
	wb_motor_set_position(rf_dir_motor, ANGLE2RAD(rightFrontAng));
	wb_motor_set_position(rb_dir_motor, ANGLE2RAD(rightRearAng));
	wb_motor_set_position(lb_dir_motor, ANGLE2RAD(leftRearAng));

	//printf("leftFrontAng = %f wheelVel = %f dir %f\r\n", leftFrontAng, wheelVel.leftFront.vel, wheelVel.leftFront.direction);
}

/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle)
{
	static unsigned char recursiveTimes = 0;

	recursiveTimes++;

	if (recursiveTimes < 100)
	{
		if (*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if (*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}

	recursiveTimes--;
}
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle)
{
	static unsigned char recursiveTimes = 0;

	recursiveTimes++;

	if (recursiveTimes < 100)
	{
		if (angle > 180.0f)
		{
			angle = ReturnLimitAngle(angle - 360.0f);
		}
		else if (angle < -180.0f)
		{
			angle = ReturnLimitAngle(angle + 360.0f);
		}
	}

	recursiveTimes--;

	return angle;
}
float TurnInferiorArc(float targetAngle, float actualAngle)
{
	if (targetAngle - actualAngle > 180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if (targetAngle - actualAngle < -180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}
}

/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子正方向角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel, float actualAngle)
{
	int n = 0;
	float angleErr = 0.0f;

	//将目标角度和当前实际角度转换到一个360度周期中
	n = (int)(actualAngle / 180.0f) - (int)(actualAngle / 360.0f);

	targetVel->direction = n * 360.0f + targetVel->direction;

	//计算目标角度和实际角度的误差
	angleErr = targetVel->direction - actualAngle;

	//将误差限制在-180度到180度
	AngleLimit(&angleErr);

	//如果角度误差大于90度则将速度反向并将目标角度加180度
	if (fabs(angleErr) > 90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;

		//保证处理后的目标角度和当前实际角度在一个周期中
		if (targetVel->direction > (n * 360.0f + 180.0f))
		{
			targetVel->direction -= 360.0f;
		}
		else if (targetVel->direction < (n * 360.0f - 180.0f))
		{
			targetVel->direction += 360.0f;
		}
	}
}
void Transform2WheelCoodinate(wheel_t *wheelVel)
{
	//将机器人坐标系下轮子朝向转换为轮子坐标系下角度
	wheelVel->leftFront.direction = wheelVel->leftFront.direction - 90.0f;
	wheelVel->rightFront.direction = wheelVel->rightFront.direction - 90.0f;
	wheelVel->leftRear.direction = wheelVel->leftRear.direction - 90.0f;
	wheelVel->rightRear.direction = wheelVel->rightRear.direction - 90.0f;

	//将角度限制在-180°到180°
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);
}

void Transform2RobotCoodinate(wheel_t *wheelVel)
{
	//将定位系统坐标系下角度转换为机器人坐标系下角度
	wheelVel->leftFront.direction -= GetAngle();
	wheelVel->rightFront.direction -= GetAngle();
	wheelVel->leftRear.direction -= GetAngle();
	wheelVel->rightRear.direction -= GetAngle();

	//将角度限制在180度到-180度范围内
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);
}

float GetAngle(void)
{
	float angle = 0.00;
	WbDeviceTag inertial_unit = wb_robot_get_device("inertial_unit");
	wb_inertial_unit_enable(inertial_unit, 1);
	angle = RAD2ANGLE(wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[2]);
	return angle;
}