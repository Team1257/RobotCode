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

template <typename Type>

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
/*
void quicksort(lineinfo *arr, unsigned long long start, unsigned long long end);
unsigned long long partition(lineinfo *arr, unsigned long long start, unsigned long long end);
void quicksortlk(vector<lineinfo> &arr, unsigned long long start, unsigned long long end);
unsigned long long partitionlk(vector<lineinfo> &arr, unsigned long long start, unsigned long long end);
inline bool lessthan(int n1, int n2); //for comparing likely, to sort such that sorted = {2,2,2,2,1,1,3,4,5,6,7,8,8,9,111,113}
inline bool roughlyequal(int n1, int n2, int tolerence);*/

These functions seem pointless.
I will comment them out for now, but I see no reason not to delete them.
There do not even seem to be full definitions, just prototypes.
*/
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
	Type tAbs(Type num);
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
