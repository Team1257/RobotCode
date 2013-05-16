#include <WPILib.h>
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define LEFT_MOTOR1 1 // Left motor is 1, moron
#define RIGHT_MOTOR1 2 // Right motor is 2
#define LEFT_MOTOR2 3
#define RIGHT_MOTOR2 4
#define CARRIAGE	5
#define SHOOTER		 6
#define SERVO_FIRE	 7
#define LEFT_STICK 1 //Testing github!!!
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

void quicksort(lineinfo *arr, unsigned long long start, unsigned long long end);
unsigned long long partition(lineinfo *arr, unsigned long long start, unsigned long long end);
void quicksortlk(vector<lineinfo> &arr, unsigned long long start, unsigned long long end);
unsigned long long partitionlk(vector<lineinfo> &arr, unsigned long long start, unsigned long long end);
inline bool lessthan(int n1, int n2); //for comparing likely, to sort such that sorted = {2,2,2,2,1,1,3,4,5,6,7,8,8,9,111,113}
inline bool roughlyequal(int n1, int n2, int tolerence);*/

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
	void shoot();
	void adjustArm(bool auton);
	
public:
	//Jaguar shoulder1;
	Victor arm;
	Victor shooter;
	Victor left1;
	Victor left2;
	Victor right1;
	Victor right2;
	Servo  fire;
	
	//Jaguar left;
	//Jaguar right;
	RobotDrive team1257Robot;
	DriverStationLCD* team1257LCD;
	Joystick leftStick;
	Joystick rightStick;
	Relay epilepsy;
	Gyro headingGyro;
	//Gyro armGyro;
	//DigitalInput wall;
	DigitalInput armStopBottom;
	DigitalInput armStopTop;
	//Relay hook1;
	//Relay hook2;
	//AnalogChannel walf; // Potentiometer
	AnalogChannel superUltra;
	//Jaguar arm;
	//Servo sThingy;
	double sPos;
	//Encoder testEncoder;
	double sf;
	//double speedValue;
	//Jaguar jag1;
	//Jaguar jag2;
};
