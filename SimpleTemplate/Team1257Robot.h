#include <WPILib.h>
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define LEFT_MOTOR1 1
#define RIGHT_MOTOR1 2
#define LEFT_STICK 1 
#define RIGHT_STICK 2
#define AREA_MINIMUM 500
#define LIGHT 4
#define GYRO 1
#define INIT_ANGLE 42 // NOTE: IT WILL BE 37 based on nothing
#define FINAL_ANGLE 22.55 // Deliberate - I swear


/*class lineinfo
{
public:
	int changenum;
	int avgpos;
	int y;
	int likely;
	lineinfo::lineinfo();
	lineinfo::lineinfo(int cn, int ap, int py, int lk);
	void lineinfo::addline(lineinfo &ln);
	void lineinfo::addlineavgy(lineinfo &ln);
};

class CTeam1257Robot : public SimpleRobot
{
public:
	CTeam1257Robot();
	void Autonomous(void);
	void OperatorControl(void);
	void Test(void);
	void balanceRobot();
	bool aimRobot(AxisCamera& camera);
	void drive(double left, double right);
	void drive();
	double dAbs(double num);
	float reduceAngle(float angle);
	double toMM(double voltage);
	
public:

	Victor left1;
	Victor right1;
	
	RobotDrive team1257Robot;
	DriverStationLCD* team1257LCD;
	Joystick leftStick;
	Joystick rightStick;
	Relay epilepsy;
	Gyro gyro;
	double sPos;
	double sf;
};
