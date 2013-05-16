#include "Team1257Robot.h"

struct rgb
{	
	unsigned char blue;
	unsigned char green;
	unsigned char red;
	unsigned char alpha;
};

struct target
{
	target()
	{
		posX = 0;
		posY = 0;
		width = 0;
		timesFound = 0;
	}
	
	target(int x, int y)
	{
		posX = x;
		posY = y;
		width = 0;
		timesFound = 0;
	}
	int posX;
	int posY;
	int width;
	int timesFound;
};


CTeam1257Robot::CTeam1257Robot(void):
	/*leftDrive1(LEFT_MOTOR1),
	rightDrive1(RIGHT_MOTOR1),
	leftDrive2(LEFT_MOTOR2),
	rightDrive2(RIGHT_MOTOR2),*/
	//left(1),
	//right(2),
	//leftDrive(11),
	//rightDrive(12),
	//shoulder1(3),
	//shoulder2(14),
	//arm(6),
	//arm2(16),
	arm(CARRIAGE),
	shooter(SHOOTER),
	left1(LEFT_MOTOR1),
	left2(LEFT_MOTOR2),
	right1(RIGHT_MOTOR1),
	right2(RIGHT_MOTOR2),
	fire(SERVO_FIRE),
	team1257Robot(left1,left2,right1,right2),
	leftStick(LEFT_STICK),
	rightStick(RIGHT_STICK),
	epilepsy(LIGHT),
	headingGyro(GYRO),
	//armGyro(4),
	//wall(2),
	armStopBottom(2),
	armStopTop(1),
	//hook1(2),
	//hook2(3),
	//walf(4),
	superUltra(3)
	//arm(3),
	// sThingy(6)
{
	//team1257LCD->Clear();
	team1257LCD = DriverStationLCD::GetInstance();
	team1257LCD->Clear();
	team1257Robot.SetExpiration(0.1); // Feed the watchdog this often or doom shall ensue
	sf = 0.7;
	//speedValue = 0;
	//epilepsy.Set(epilepsy.kForwardOnly);
}

void CTeam1257Robot::Test()
{
	/*while(IsEnabled())
	{
		drive(0.5, 0.5);
	}*/
	
	Timer move;
	move.Start();
	
	while(move.Get() < 1)
	{
		drive(0.3, 0.3);
	}
	drive(0, 0);
	while(move.Get() < 2)
	{
		drive(-0.3, -0.3);
	}
	drive(0, 0);
	while(move.Get() < 3)
	{
		drive(0.5, -0.5); // Turn right
		team1257LCD->Printf(DriverStationLCD::kUser_Line3, 1, "%f", reduceAngle(headingGyro.GetAngle()));
		team1257LCD->UpdateLCD();
	}
	drive(0, 0);
	while(move.Get() < 4)
	{
		drive(-0.5, 0.5); // Turn left
		team1257LCD->Printf(DriverStationLCD::kUser_Line3, 1, "%f", reduceAngle(headingGyro.GetAngle()));
		team1257LCD->UpdateLCD();
	}
	/*while(IsEnabled())
	{
		epilepsy.Set(epilepsy.kForward);
		Wait(1);
		epilepsy.Set(epilepsy.kOff);
		team1257LCD->Printf(DriverStationLCD::kUser_Line3, 1, "%f, %f", reduceAngle(headingGyro.GetAngle()), toMM(superUltra.GetValue()));
		team1257LCD->UpdateLCD();
	}*/
}

void CTeam1257Robot::Autonomous()
{	
	AxisCamera& camera = AxisCamera::GetInstance("10.12.57.11");	// Declare axis camera
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(30);
	camera.WriteWhiteBalance(AxisCamera::kWhiteBalance_FixedIndoor);
	
	team1257LCD->Clear();
	team1257LCD->UpdateLCD();
	
	team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Autonomous Engaged!");
	team1257LCD->UpdateLCD();
		
	team1257LCD->Printf(DriverStationLCD::kUser_Line2, 1, "Attempting to get good data...");
	team1257LCD->UpdateLCD();
	
	while(!aimRobot(camera) && !leftStick.GetRawButton(2) && !rightStick.GetRawButton(2)); // Epic
	
	team1257LCD->Printf(DriverStationLCD::kUser_Line2, 1, "Data retreived!                ");
	team1257LCD->UpdateLCD();
	
	Timer move;
	move.Start();
	bool good = true;
	while(good && IsEnabled())
	{
		aimRobot(camera);
		//drive(0, 0);
		move.Reset();
		while(move.Get() < 1.5 && IsEnabled())
		{
			drive(0.4, 0.4);
			team1257LCD->Printf(DriverStationLCD::kUser_Line3, 1, "Distance (mm): %f", toMM(superUltra.GetVoltage()));
			team1257LCD->UpdateLCD();
			if(toMM(superUltra.GetVoltage()) < 508 + 792)
			{
				good = false;
				drive(0, 0);
				break;
			}
			if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
				break;
		}
		drive(0, 0);
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
	}
	drive(0, 0);
	
	team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Autonomous Completed! Firing");
	team1257LCD->UpdateLCD();
	
	/*speedValue = 0;
	Timer speedTimer;
	speedTimer.Start();
	while(reduceAngle(armGyro.GetAngle()) < FINAL_ANGLE - INIT_ANGLE)
	{
		shoulder1.Set(speedValue);
		shoulder2.Set(speedValue);
		if(speedTimer.Get() >= 0.1)
		{
			speedValue += 0.03;
			speedTimer.Reset();
		}
		
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
	}
	bool called = false;
	while(IsAutonomous() && IsEnabled())
	{
		shoulder1.Set(speedValue);
		shoulder2.Set(speedValue);
		if(!called)
		{
			shoot();
			called = true;
		}
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
	}*/
}

void CTeam1257Robot::OperatorControl()
{
	AxisCamera& camera = AxisCamera::GetInstance("10.12.57.11");	// Declare axis camera
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(30);
	camera.WriteWhiteBalance(AxisCamera::kWhiteBalance_FixedIndoor);
		
	team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Teleop Engaged.  Drive well, Peter.");
	team1257LCD->UpdateLCD();
	
	Timer shootTimer;
	int shootiterations=0;
	//Timer armTimer;
	//Timer hookTimer;
	//hookTimer.Start();
	//sPos = // sThingy.GetAngle();
	
	while(IsOperatorControl() && IsEnabled())
	{
		/*team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "%f", (walf.GetVoltage() / 4.15) * 270);
		team1257LCD->UpdateLCD();*/
		/*team1257LCD->Printf(DriverStationLCD::kUser_Line2, 1, "%f", rightStick.GetRawAxis(5));
		team1257LCD->UpdateLCD();
		team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "%i", armStopBottom.Get());
		team1257LCD->UpdateLCD();*/
		fire.SetAngle(sPos);
		team1257LCD->Printf(DriverStationLCD::kUser_Line2, 1, "%f", shootTimer.Get()); // sThingy.GetAngle());
		team1257LCD->UpdateLCD();
		/*team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "%f, %f", sPos, // sThingy.GetAngle());
		team1257LCD->UpdateLCD();*/
		
		/*if(rightStick.GetRawButton(7))
		{
			shoot();
			Wait(1);
		}
		if(rightStick.GetRawButton(8)) // Loading
		{
			
		}*/
		//drive(0.5, 0.5);		
		
		if((leftStick.GetRawAxis(5) < 0 && leftStick.GetY()) ||	leftStick.GetRawAxis(5) > 0 && leftStick.GetY() < 0)
		{
			sf = 1;
		}
		else
		{
			sf = 0.7;
		}
		
		if(leftStick.GetRawButton(5) && leftStick.GetRawButton(6))
		{
			drive();
			arm.Set(leftStick.GetRawAxis(3));
			//team1257Robot.SetLeftRightMotorOutputs(leftStick.GetY() * sf, leftStick.GetRawAxis(axis) * sf);
		}
		else
		{
			drive(0, 0);
			arm.Set(0);
		}
		
		if(!armStopTop.Get() && !armStopBottom.Get()) // Default state is false
		{
			arm.Set(leftStick.GetRawAxis(3));
			//arm2.Set(leftStick.GetRawAxis(3));
		}
		else if(armStopTop.Get()) // Triggered
		{
			team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Top tripped");
			team1257LCD->UpdateLCD();
			if(leftStick.GetRawAxis(3) < 0) // Negative, so down
			{
				team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Moving down");
				team1257LCD->UpdateLCD();
				arm.Set(leftStick.GetRawAxis(3));
				//arm2.Set(leftStick.GetRawAxis(3));
			}
			else
			{
				arm.Set(0);
				//arm2.Set(0);
			}
		}
		else if(armStopBottom.Get()) 
		{
			team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Bottom tripped");
			team1257LCD->UpdateLCD();
			if(leftStick.GetRawAxis(3) > 0) // Positive, so up
			{
				team1257LCD->Printf(DriverStationLCD::kUser_Line1, 1, "Moving up");
				team1257LCD->UpdateLCD();
				arm.Set(leftStick.GetRawAxis(3));
				//arm2.Set(leftStick.GetRawAxis(3));
			}
			else
			{
				arm.Set(0);
				//arm2.Set(0);
			}
		}
		
		if(leftStick.GetRawButton(1) && shootTimer.Get()==0)
		{	
			while(!aimRobot(camera));
			shootTimer.Start();
			shooter.Set(1.0);
		}
		else if(shootTimer.Get()>0)
			shooter.Set(1.0);
		
		if(shootTimer.Get()>3 && fire.GetAngle()>=180)
			sPos=0;
		else if(shootTimer.Get()>3 && fire.GetAngle()<=1 && shootiterations<5)
		{
			sPos=180;
			shootiterations++;
		}
		else if(shootiterations==5)
		{
			shootTimer.Stop();
			shootTimer.Reset();
			shootiterations=0;
			sPos=0;
		}
	}
}

bool CTeam1257Robot::aimRobot(AxisCamera& camera)
{
	ColorImage* colorImage1 = new RGBImage;
	camera.GetImage(colorImage1);
	ImageInfo info1;
	Image* imaqImage1 = colorImage1->GetImaqImage();
	imaqGetImageInfo(imaqImage1, &info1);
	rgb* pixelImage1 = (rgb*)info1.imageStart;
	
	Wait(0.25);
	
	// Light code
	epilepsy.Set(epilepsy.kForward);
	
	Wait(0.25);
	
	ColorImage* colorImage2 = new RGBImage;
	camera.GetImage(colorImage2);
	ImageInfo info2;
	Image* imaqImage2 = colorImage2->GetImaqImage();
	imaqGetImageInfo(imaqImage2, &info2);
	rgb* pixelImage2 = (rgb*)info2.imageStart;
	
	int prevX = -500;
	
	target final[15];
	
	if(info1.xRes < 191)
		return false;
	
	for(int y = 0; y < info1.yRes; y++)
	{
		prevX = -500;
		vector<target> targets;
		for(int x = 0; x < info1.xRes; x++)
		{				
			// Do the processing
			if((int)pixelImage2->green - (int)pixelImage1->green > 100) 
			{
				if(x - prevX > 7) // Big jump - new target
				{
					target newTarget(x, y);
					newTarget.width++;
					targets.push_back(newTarget);
				}
				else // Still the same target
				{
					int index = targets.size() - 1;
					targets[index].posX += x;
					targets[index].width++;
				}
				prevX = x;
			}
			pixelImage1++;
			pixelImage2++;
			
			if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
				break;
		}
		
		for(int i = 0; i < (int)targets.size(); i++)
		{
			int leastValue = 5000;
			int leastIndex = 0;
			for(int j = 0; j < 15; j++)
			{
				if(final[j].width < leastValue)
				{
					leastValue = final[j].width;
					leastIndex = j;
				}
				if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
					break;
			}
			
			if(targets[i].width > leastValue)
			{
				final[leastIndex] = targets[i];
			}
			
			if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
				break;
		}
		
		pixelImage1 += info1.pixelsPerLine - info1.xRes;
		pixelImage2 += info2.pixelsPerLine - info2.xRes;
	}
	
	for(int i = 0; i < 15; i++)
	{
		final[i].posX /= final[i].width;
	}
	
	vector<target> finalTargets;
	for(int i = 0; i < 15; i++)
	{
		bool found = false;
		for(int j = 0; j < (int)finalTargets.size(); j++)
		{
			if((abs(finalTargets[j].posX / finalTargets[j].timesFound - final[i].posX) < 7) &&
				(abs(finalTargets[j].width / finalTargets[j].timesFound - final[i].width) < 7))
			{
				finalTargets[j].posX += final[i].posX;
				finalTargets[j].posY += final[i].posY;
				finalTargets[j].width += final[i].width;
				finalTargets[j].timesFound++;
				found = true;          
			}
			if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
				break;
		}
		if(!found)
		{
			finalTargets.push_back(final[i]);
			finalTargets[finalTargets.size() - 1].timesFound++;
		}
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
	}
	
	team1257LCD->Clear();
	team1257LCD->UpdateLCD();
	
	/*if(finalTargets.size() > 0)
	{
		team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "x: %i, y: %i, width: %i", finalTargets[0].posX / finalTargets[0].timesFound, finalTargets[0].posY / finalTargets[0].timesFound, finalTargets[0].width / finalTargets[0].timesFound);
		team1257LCD->UpdateLCD();
	}*/
/*#define LINE(val) team1257LCD->Printf(DriverStationLCD::kUser_Line##val##, 1, "%i, %i, %i", finalTargets[ ## val ## ].posX, finalTargets[## val ##].posY, finalTargets[## val ##].width);
	for(int i=0; i<3; i++)
	{
		if(i>=finalTargets.size())
			break;
		LINE(i);
	}*/
	
	/*if(finalTargets.size() > 1)
	{
		team1257LCD->Printf(DriverStationLCD::kUser_Line3, 1, "%i, %i, %i", finalTargets[1].posX / finalTargets[1].timesFound, finalTargets[1].posY / finalTargets[1].timesFound, finalTargets[1].width / finalTargets[1].timesFound);
		team1257LCD->UpdateLCD();
	}
	if(finalTargets.size() > 2)
	{
		team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "%i, %i, %i", finalTargets[2].posX / finalTargets[2].timesFound, finalTargets[2].posY / finalTargets[2].timesFound, finalTargets[2].width / finalTargets[2].timesFound);
		team1257LCD->UpdateLCD();
	}
	if(finalTargets.size() > 3)
	{
		team1257LCD->Printf(DriverStationLCD::kUser_Line5, 1, "%i, %i, %i", finalTargets[3].posX / finalTargets[3].timesFound, finalTargets[3].posY / finalTargets[3].timesFound, finalTargets[3].width / finalTargets[3].timesFound);
		team1257LCD->UpdateLCD();
	}*/
	team1257LCD->Printf(DriverStationLCD::kUser_Line4, 1, "Targets Found: %i", finalTargets.size());
	team1257LCD->UpdateLCD();
	
	target theTarget;
	theTarget.posY = 5000;
	// Calculate target positions
	for(int i = 0; i < (int)finalTargets.size(); i++)
	{
		if(finalTargets[i].timesFound > 2)
		{
			if(finalTargets[i].posY / finalTargets[i].timesFound > theTarget.posY)
			{
				theTarget.posY = finalTargets[i].posY / finalTargets[i].timesFound;
				theTarget.posX = finalTargets[i].posX / finalTargets[i].timesFound;
				theTarget.width = finalTargets[i].width / finalTargets[i].timesFound;
			}
		}
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
	}
	
	team1257LCD->Printf(DriverStationLCD::kUser_Line5, 1, "x: %i, y: %i, width: %i", theTarget.posX, theTarget.posY, theTarget.width);
	team1257LCD->UpdateLCD();
	// Clean up
	delete colorImage1;
	delete colorImage2;
	imaqDispose(imaqImage1); //I'm pretty sure, check - guess not
	imaqDispose(imaqImage2);
	
	double averagePosX = (double)theTarget.posX;
	
	double angle  = ((averagePosX - 160) / 160) * 22.5;
		headingGyro.Reset();
		
	while(dAbs(reduceAngle(headingGyro.GetAngle()) - angle) > 0.6 && IsEnabled())
	{
		if(reduceAngle(headingGyro.GetAngle()) - angle < -0.6) // Turn left
		{
			drive(0.7, -0.7);
		}
		if(reduceAngle(headingGyro.GetAngle()) - angle > 0.6) // Turn right
		{
			drive(-0.7, 0.7);
		}
		
		if(leftStick.GetRawButton(2) || rightStick.GetRawButton(2))
			break;
		
		team1257LCD->Printf(DriverStationLCD::kUser_Line6, 1, "Angles: %f, %f", reduceAngle(headingGyro.GetAngle()), angle);
		team1257LCD->UpdateLCD();
	}
	
	drive(0, 0);
	epilepsy.Set(epilepsy.kOff);
	Wait(0.25);
	
	return true;
}

/*void CTeam1257Robot::shoot(bool start)
{
	if(start==true)
		shootTimer.Start();
	shooter.Set(1.0);
}*/

void CTeam1257Robot::drive(double left, double right)
{
	team1257Robot.SetLeftRightMotorOutputs(left, right); // Note - no negatives on real robot!!! probably
	//leftDrive2.Set(-left);
	//rightDrive2.Set(right);
}

void CTeam1257Robot::drive()
{
	team1257Robot.SetLeftRightMotorOutputs(-leftStick.GetRawAxis(5) * sf, -leftStick.GetY() * sf);
	//leftDrive2.Set(leftStick.GetY() * sf);
	//rightDrive2.Set(-leftStick.GetTwist() * sf);
}

double CTeam1257Robot::dAbs(double num)
{
	if(num < 0)
		num *= - 1;
	
	return num;
}

float CTeam1257Robot::reduceAngle(float angle)
{
	int intAngle = (int)angle % 360;
	int intCompleteAngle = (int)angle;
	float decimalPart = angle - (float)intCompleteAngle;
	angle = (float)intAngle + decimalPart;
	
	return angle;
}

double CTeam1257Robot::toMM(double voltage)
{
	return (voltage / 2.5) * 6450;//* 253.937; // 0-2.5 volts converted to MM (max distnace of 6,450mm)
}

START_ROBOT_CLASS(CTeam1257Robot);
