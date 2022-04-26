#ifndef __MOVEBASE_H
#define __MOVEBASE_H


//角度制转化为弧度制
#define ANGLE2RAD(x) (x / 180.0f * PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x / PI * 180.0f)
//底盘旋转半径
#define MOVEBASE_RADIUS (424.2f)
//左前轮与中心连线切线方向
#define LEFT_FRONT_VERTICAL_ANG (-135.0f) //-135.0f
//右前轮与中心连线切线方向
#define RIGHT_FRONT_VERTICAL_ANG (135.0f) //135.0f
//左后轮与中心连线切线方向
#define LEFT_REAR_VERTICAL_ANG (-45.0f) //-45.0f
//右后轮与中心连线切线方向
#define RIGHT_REAR_VERTICAL_ANG (45.0f) //45.0f

#define PI 3.1415926f
typedef struct
{
	//轮子速度大小
	float vel;
	//轮子速度方向
	float direction;
} wheelVel_t;

typedef struct
{
	//左前轮
	wheelVel_t leftFront;
	//右前轮
	wheelVel_t rightFront;
	//左后轮
	wheelVel_t leftRear;
	//右后轮
	wheelVel_t rightRear;
} wheel_t;

void Output_Wheel(float vel, float direction, float omega);
wheelVel_t CalcWheel(float vel, float direction, float omega, float angleN, float postureAngle);
// void RadLimit(float *angle);
void AngleLimit(float *angle);
float ReturnLimitAngle(float angle);
void Wheel_Control(wheel_t wheelVel);
void motor_fetch();
void JudgeVelDirection(wheelVel_t *targetVel, float actualAngle);
void Transform2WheelCoodinate(wheel_t *wheelVel);
void Transform2RobotCoodinate(wheel_t *wheelVel);
float GetAngle(void);

float TurnInferiorArc(float targetAngle, float actualAngle);

#endif
