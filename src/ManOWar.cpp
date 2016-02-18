#include <WPILib.h>
#include <Config.h>
#include <ManOWar.h>

ManOWar::ManOWar() {
	this->robotDrive = new RobotDrive(LEFT_TANK_TALON, RIGHT_TANK_TALON);
	this->joystick = new Joystick(0);

	this->intakeTalon = new Talon(INTAKE_TALON);
	this->topFireCanTalon = new CANTalon(TOP_FIRE_CAN_TALON);
	this->botFireCanTalon = new CANTalon(BOT_FIRE_CAN_TALON);

	this->ledRelay = new Relay(LED_RELAY);

	this->jetsonSerialPort = new SerialPort(38400, SerialPort::Port::kMXP, 8,
			SerialPort::Parity::kParity_None,
			SerialPort::StopBits::kStopBits_One);
	this->jetsonNetworkTable = NULL;
}

void ManOWar::RobotInit() {
	this->topFireCanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->topFireCanTalon->ConfigEncoderCodesPerRev(1024);
	this->topFireCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->topFireCanTalon->SetControlMode(CANTalon::ControlMode::kSpeed);
	this->topFireCanTalon->SetSensorDirection(true);
	this->topFireCanTalon->SetClosedLoopOutputDirection(true);

	this->botFireCanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->botFireCanTalon->ConfigEncoderCodesPerRev(1024);
	this->botFireCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->botFireCanTalon->SetControlMode(CANTalon::ControlMode::kSpeed);
	this->botFireCanTalon->SetSensorDirection(true);
	this->botFireCanTalon->SetClosedLoopOutputDirection(true);

	SmartDashboard::PutString("DB/String 0", "2550");
	SmartDashboard::PutString("DB/String 1", "2550");
	SmartDashboard::PutString("DB/String 2", "1");
	SmartDashboard::PutString("DB/String 3", "0");

	this->jetsonSerialPort->DisableTermination();
	//jetsonSerialPort.SetFlowControl(SerialPort::FlowControl::kFlowControl_XonXoff);
	//NetworkTable::SetTeam(2410);
	NetworkTable::SetPort(5000);
	NetworkTable::SetServerMode();
	//NetworkTable::Initialize();
	this->jetsonNetworkTable = NetworkTable::GetTable("Jetson").get();
}

void ManOWar::Autonomous() {
	int mode = std::stof(SmartDashboard::GetString("DB/String 3", "0"));
	DriverStation::ReportError("Running autonomous " + mode);
}

void ManOWar::OperatorControl() {
	bool *serialThreadQuit = new bool(false);
	std::thread *serialThread;
	float *angle = new float(0);
	float *distance = new float(0);
	long *reads = new long(0);

	serialThread = new std::thread([this, serialThreadQuit, angle, distance, reads] () -> void {
		char buffer[8];
		memset(buffer, 0, 8);
		while(!(*serialThreadQuit)) {
			SmartDashboard::PutString("DB/String 4", std::to_string(*reads));
			if (this->jetsonSerialPort->GetBytesReceived() >= 8) {
				this->jetsonSerialPort->Read(buffer, 8);
				this->jetsonSerialPort->Reset();
				memcpy(angle, buffer, sizeof(float));
				memcpy(distance, buffer + 4, sizeof(float));
				SmartDashboard::PutString("DB/String 8", std::to_string(*angle));
				SmartDashboard::PutString("DB/String 9", std::to_string(*distance));
				*reads += 1;
			}
		}
	});
	serialThread->detach();

	this->robotDrive->SetSafetyEnabled(false);
	SmartDashboard::PutString("DB/String 7", "CEASE_FIRE");

	bool intakeOverride = false;

	double topFireRpm = 0;
	double botFireRpm = 0;

	float topFireTargetRpm;
	float botFireTargetRpm;
	float intakeRpm;

	bool intake;
	bool spin;
	bool reverseIntake;
	bool reverseSpin;
	bool autoFire;
	bool autoAlign;

	bool fire;

	while (RobotBase::IsEnabled()) {
		// Grab values from dashboard
		//topFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 0", "0"));
		//botFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 1", "0"));
		intakeRpm = std::stof(SmartDashboard::GetString("DB/String 2", "0"));

		// Calculate power
		if (*distance <= 0 || *distance >= 250.f || isnan(*distance)) {
			// Correct for serial error
			*distance = 120.f;
		}
		topFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
		botFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
		SmartDashboard::PutString("DB/String 0", std::to_string(topFireTargetRpm));
		SmartDashboard::PutString("DB/String 1", std::to_string(botFireTargetRpm));

		// Get buttons
		intake = this->joystick->GetRawButton(JOY_BTN_LBM);
		spin = this->joystick->GetRawButton(JOY_BTN_LTG);
		reverseIntake = this->joystick->GetRawButton(JOY_BTN_RBM);
		reverseSpin = this->joystick->GetRawButton(JOY_BTN_RTG);
		autoFire = this->joystick->GetRawButton(JOY_BTN_A);
		autoAlign = this->joystick->GetRawButton(JOY_BTN_Y);

		// Drive
		if (autoAlign) {
			this->robotDrive->ArcadeDrive(0, -sgn(*angle) * ALIGN_ROTATE_POWER, false);
		}
		else {
			this->robotDrive->ArcadeDrive(this->joystick->GetRawAxis(JOY_AXIS_LY), this->joystick->GetRawAxis(JOY_AXIS_RX) * 0.75f);
		}

		// Get encoder RPMs
		topFireRpm = this->topFireCanTalon->GetSpeed();
		botFireRpm = this->botFireCanTalon->GetSpeed();
		SmartDashboard::PutString("DB/String 5", std::to_string(topFireRpm));
		SmartDashboard::PutString("DB/String 6", std::to_string(botFireRpm));

		fire = topFireRpm > topFireTargetRpm * 0.975f && topFireRpm < topFireTargetRpm * 1.025f && (botFireRpm > botFireTargetRpm * 0.975f && botFireRpm < botFireTargetRpm * 1.025f);

		// Spin fire motors
		if (spin) {
			this->topFireCanTalon->Set(topFireTargetRpm);
			this->botFireCanTalon->Set(botFireTargetRpm);
		}
		else if (reverseSpin) {
			this->topFireCanTalon->Set(-topFireTargetRpm);
			this->botFireCanTalon->Set(-botFireTargetRpm);
		}
		else {
			this->topFireCanTalon->Set(0.f);
			this->botFireCanTalon->Set(0.f);
		}

		if (fire) {
			SmartDashboard::PutString("DB/String 7", "FIRE");
			if (spin && autoFire) {
				this->intakeTalon->Set(intakeRpm);
				intakeOverride = true;
			}
			else {
				intakeOverride = false;
			}
		}
		else {
			SmartDashboard::PutString("DB/String 7", "CEASE_FIRE");
			intakeOverride = false;
		}

		// Intake motors
		if (intake) {
			this->intakeTalon->Set(intakeRpm);
		}
		else if (reverseIntake) {
			this->intakeTalon->Set(-intakeRpm / 2.f);
		}
		else if (!intakeOverride) {
			this->intakeTalon->Set(0.f);
		}
	}

	*serialThreadQuit = true;
	Wait(0.1f);
	delete serialThreadQuit;
	delete serialThread;
	delete angle;
	delete distance;
	delete reads;
}

START_ROBOT_CLASS(ManOWar);
