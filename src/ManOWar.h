#ifndef __MANOWAR_H_
#define __MANOWAR_H_

int sgn(double num) {
	return num == 0 ? 0 : num / fabs(num);
}

class ManOWar : public SampleRobot
{
public:
	RobotDrive *robotDrive;
	Joystick *joystick;

	SendableChooser *sendableChooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";

	Talon *intakeTalon;
	CANTalon *topFireCanTalon;
	CANTalon *botFireCanTalon;

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
