#ifndef __MANOWAR_H_
#define __MANOWAR_H_

int sgn(double num) {
	return num == 0 ? 0 : num / fabs(num);
}

float sym_sigmoid(float x, float a, float b, float c, float d) {
	return d + ((a - d)/(1 + pow(x / c, b)));
}

class ManOWar : public SampleRobot
{
public:
	RobotDrive *robotDrive;
	Joystick *joystick;

	Talon *intakeTalon;
	CANTalon *topFireCanTalon;
	CANTalon *botFireCanTalon;

	Relay *ledRelay;

	AnalogGyro *gyro;

	SerialPort *jetsonSerialPort;
	NetworkTable *jetsonNetworkTable;

	ManOWar();
	~ManOWar() {};

	void RobotInit();
	void Disabled() {};
	void Autonomous();
	void OperatorControl();
	void Test() {};
	// Don't override
	//void RobotMain() {};
	//void StartCompetition() {};
};

#endif
