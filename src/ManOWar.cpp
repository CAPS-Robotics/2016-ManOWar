#include <WPILib.h>
#include <Config.h>
#include <ManOWar.h>

ManOWar::ManOWar() {
	DriverStation::ReportError("Running pre-init...");
	this->robotDrive = new RobotDrive(LEFT_TANK_TALON, RIGHT_TANK_TALON);
	this->joystick = new Joystick(0);

	this->intakeTalon = new Talon(INTAKE_TALON);
	this->topFireCanTalon = new CANTalon(TOP_FIRE_CAN_TALON);
	this->botFireCanTalon = new CANTalon(BOT_FIRE_CAN_TALON);
	this->armACanTalon = new CANTalon(ARM_A_CAN_TALON);
	this->armBCanTalon = new CANTalon(ARM_B_CAN_TALON);

	this->ledRelay = new Relay(LED_RELAY);

	this->gyro = new AnalogGyro(GYRO_ANALOG);
	this->photoSensor = new DigitalInput(PHOTO_DIGITAL);

	this->jetsonSerialPort = new SerialPort(9600, SerialPort::Port::kMXP, 8,
			SerialPort::Parity::kParity_None,
			SerialPort::StopBits::kStopBits_One);
	this->jetsonNetworkTable = NULL;

	this->armMode = new bool(false);
}

void ManOWar::RobotInit() {
	DriverStation::ReportError("Running RobotInit()...");
	this->topFireCanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->topFireCanTalon->ConfigEncoderCodesPerRev(1024);
	this->topFireCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->topFireCanTalon->SetControlMode(CANTalon::ControlMode::kSpeed);
	this->topFireCanTalon->SetSensorDirection(false);
	this->topFireCanTalon->SetClosedLoopOutputDirection(false);

	this->botFireCanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->botFireCanTalon->ConfigEncoderCodesPerRev(1024);
	this->botFireCanTalon->SetPID(0.05f, 0.000096f, 0.8f, 0.f);
	this->botFireCanTalon->SetControlMode(CANTalon::ControlMode::kSpeed);
	this->botFireCanTalon->SetSensorDirection(false);
	this->botFireCanTalon->SetClosedLoopOutputDirection(false);

	//TEST CODE
	if (SmartDashboard::GetBoolean("DB/Button 0", false)) {
		this->armACanTalon = new CANTalon(TOP_FIRE_CAN_TALON);
		this->armMode = new bool(true);
	}

	this->armACanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->armACanTalon->ConfigEncoderCodesPerRev(497);
	this->armACanTalon->SetPID(1.8f, 0.00006f, 0.00008f, 0.f);
	this->armACanTalon->SetControlMode(CANTalon::ControlMode::kPosition);
	this->armACanTalon->SetSensorDirection(false);
	this->armACanTalon->SetClosedLoopOutputDirection(false);

	this->armBCanTalon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	this->armBCanTalon->ConfigEncoderCodesPerRev(497);
	this->armBCanTalon->SetPID(1.8f, 0.00006f, 0.00008f, 0.f);
	this->armBCanTalon->SetControlMode(CANTalon::ControlMode::kPosition);
	this->armBCanTalon->SetSensorDirection(false);
	this->armBCanTalon->SetClosedLoopOutputDirection(false);

	DriverStation::ReportError("[RobotInit()] Gyro calibrating...");
	this->gyro->InitGyro();
	this->gyro->Calibrate();
	DriverStation::ReportError("[RobotInit()] Gyro calibrated.");

	SmartDashboard::PutString("DB/String 0", "2550");
	SmartDashboard::PutString("DB/String 1", "2550");
	SmartDashboard::PutString("DB/String 2", "1");
	SmartDashboard::PutString("DB/String 3", "0");
	SmartDashboard::PutString("DB/String 4", "0");

	this->jetsonSerialPort->DisableTermination();
	NetworkTable::SetServerMode();
	NetworkTable::SetTeam(2410);
	NetworkTable::SetPort(1735);
	NetworkTable::Initialize();
	this->jetsonNetworkTable = NetworkTable::GetTable("Jetson");
	this->jetsonNetworkTable->PutNumber("angle", 0.f);
	this->jetsonNetworkTable->PutNumber("distance", 120.f);

	CameraServer::GetInstance()->SetQuality(50);
	std::shared_ptr<USBCamera> camera(new USBCamera("cam0", false));
	camera->OpenCamera();
	camera->SetExposureManual(0);
	camera->SetBrightness(80);
	CameraServer::GetInstance()->StartAutomaticCapture(camera);
	this->robotDrive->SetSafetyEnabled(false);
}

void ManOWar::Autonomous() {
	int mode = std::stof(SmartDashboard::GetString("DB/String 3", "0"));
	DriverStation::ReportError("Running Autonomous() " + std::to_string(mode));
	bool *serialThreadQuit = new bool(false);
	std::thread *serialThread;
	float *angle = new float(0);
	float *distance = new float(0);

	serialThread = new std::thread([this, serialThreadQuit, angle, distance] () -> void {
		char buffer[8];
		memset(buffer, 0, 8);
		while(!(*serialThreadQuit)) {
			//SmartDashboard::PutString("DB/String 4", "RIP_SERIAL");
			*angle = this->jetsonNetworkTable->GetNumber("angle", 0.f);
			*distance = this->jetsonNetworkTable->GetNumber("distance", 120.f);
			SmartDashboard::PutString("DB/String 8", std::to_string(*angle));
			SmartDashboard::PutString("DB/String 9", std::to_string(*distance));
		}
	});
	serialThread->detach();

	float topFireTargetRpm;
	float botFireTargetRpm;
	double topFireRpm = 0;
	double botFireRpm = 0;
	bool fire;

	bool aligning = false;
	bool aligned = false;
	float refAngle = 0;

	this->gyro->Reset();

	this->robotDrive->ArcadeDrive(-0.85f, 0, false);
	Wait(2.3f);
	this->robotDrive->ArcadeDrive(0, 0, false);
	Wait(1.f);
	while (fabs(this->gyro->GetAngle()) > 0.25) {
		this->robotDrive->ArcadeDrive(0, sgn(-this->gyro->GetAngle()) * ALIGN_ROTATE_POWER, false);
	}
	this->robotDrive->ArcadeDrive(0, 0, false);
	Wait(2.5f);

	while (!aligned) {
		if (aligning && !aligned) {
			DriverStation::ReportError(std::to_string(this->gyro->GetAngle()));
			if (fabs(refAngle - this->gyro->GetAngle()) > 0.25) {
				this->robotDrive->ArcadeDrive(0, sgn(refAngle) * ALIGN_ROTATE_POWER, false);
			}
			else {
				DriverStation::ReportError("Aligned.");
				this->robotDrive->ArcadeDrive(0, 0, false);
				aligning = false;
				aligned = true;
			}
		}
		else {
			aligning = true;
			refAngle = *angle;
			this->gyro->Reset();
			DriverStation::ReportError("Starting alignment...");
		}
	}

	Wait(0.2f);
	// Get encoder RPMs
	topFireRpm = this->topFireCanTalon->GetSpeed();
	botFireRpm = this->botFireCanTalon->GetSpeed();
	SmartDashboard::PutString("DB/String 5", std::to_string(topFireRpm));
	SmartDashboard::PutString("DB/String 6", std::to_string(botFireRpm));
	topFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
	botFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
	SmartDashboard::PutString("DB/String 0", std::to_string(topFireTargetRpm));
	SmartDashboard::PutString("DB/String 1", std::to_string(botFireTargetRpm));

	// Spin fire motors
	this->topFireCanTalon->Set(topFireTargetRpm);
	this->botFireCanTalon->Set(botFireTargetRpm);

	do {
		topFireRpm = this->topFireCanTalon->GetSpeed();
		botFireRpm = this->botFireCanTalon->GetSpeed();
		fire = topFireRpm > topFireTargetRpm * 0.975f && topFireRpm < topFireTargetRpm * 1.025f && (botFireRpm > botFireTargetRpm * 0.975f && botFireRpm < botFireTargetRpm * 1.025f);
	}
	while (!fire);
	this->intakeTalon->Set(1.f);
	Wait(1.0f);
	this->intakeTalon->Set(0.f);
	this->topFireCanTalon->Set(0.f);
	this->botFireCanTalon->Set(0.f);

	// Kill serial thread
	*serialThreadQuit = true;
	Wait(0.1f);
	delete serialThreadQuit;
	delete serialThread;
	delete angle;
	delete distance;
}

void ManOWar::OperatorControl() {
	DriverStation::ReportError("Running OperatorControl()...");
	bool *serialThreadQuit = new bool(false);
	std::thread *serialThread;
	float *angle = new float(0);
	float *distance = new float(0);

	serialThread = new std::thread([this, serialThreadQuit, angle, distance] () -> void {
		char buffer[8];
		memset(buffer, 0, 8);
		while(!(*serialThreadQuit)) {
			//SmartDashboard::PutString("DB/String 4", "RIP_SERIAL");
			*angle = this->jetsonNetworkTable->GetNumber("angle", 0.f);
			*distance = this->jetsonNetworkTable->GetNumber("distance", 120.f);
			SmartDashboard::PutString("DB/String 8", std::to_string(*angle));
			SmartDashboard::PutString("DB/String 9", std::to_string(*distance));
		}
	});
	serialThread->detach();

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
	bool armSet;
	bool port;
	bool bridge;
	bool sally;
	bool frise;
	bool reset;

	bool aligning = false;
	bool aligned = false;
	float refAngle = 0;

	bool fire;

	float Kp = 0.044000;
	float Ki = 0.000001;
	float Kd = 0.000001;
	float xPIDError = 0;
	float xPIDIntegral = 0;
	float xCurrentSpeed = 0;
	float yPIDError = 0;
	float yPIDIntegral = 0;
	float yCurrentSpeed = 0;
	double oldtime = GetTime();

	while (RobotBase::IsEnabled()) {
		// Grab values from dashboard
		topFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 0", "0"));
		botFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 1", "0"));
		intakeRpm = std::stof(SmartDashboard::GetString("DB/String 2", "0"));

		// Calculate power
		if (*distance <= 0 || *distance >= 250.f || isnan(*distance)) {
			// Fallback on error
			*distance = 120.f;
		}
		/*topFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
		botFireTargetRpm = sym_sigmoid(*distance, POWER_FUNC_A, POWER_FUNC_B, POWER_FUNC_C, POWER_FUNC_D);
		SmartDashboard::PutString("DB/String 0", std::to_string(topFireTargetRpm));
		SmartDashboard::PutString("DB/String 1", std::to_string(botFireTargetRpm));*/

		// Get buttons
		intake = this->joystick->GetRawButton(JOY_BTN_LBM);
		spin = this->joystick->GetRawButton(JOY_BTN_LTG);
		reverseIntake = this->joystick->GetRawButton(JOY_BTN_RBM);
		reverseSpin = this->joystick->GetRawButton(JOY_BTN_RTG);
		autoFire = this->joystick->GetRawButton(JOY_BTN_A);
		autoAlign = this->joystick->GetRawButton(JOY_BTN_Y);
		armSet = this->joystick->GetRawButton(JOY_BTN_B);

		sally = this->joystick->GetPOV() == 0;
		port = this->joystick->GetPOV() == 90;
		bridge = this->joystick->GetPOV() == 180;
		reset = this->joystick->GetPOV() == -1;

		// Drive
		double curTime = GetTime();
		// X PID loop
		float xCurrentError = joystick->GetRawAxis(JOY_AXIS_RX) - xCurrentSpeed;
		xPIDIntegral += xPIDError * (curTime - oldtime);
		float xPIDderivative = (xCurrentError - xPIDError) / (curTime - oldtime);
		xCurrentSpeed += (Kp * xCurrentError) + (Ki * xPIDIntegral) + (Kd * xPIDderivative);
		xPIDError = xCurrentError;
		// Y PID loop
		float yCurrentError = joystick->GetRawAxis(JOY_AXIS_LY) - yCurrentSpeed;
		yPIDIntegral += yPIDError * (curTime - oldtime);
		float yPIDderivative = (yCurrentError - yPIDError) / (curTime - oldtime);
		yCurrentSpeed += (Kp * yCurrentError) + (Ki * yPIDIntegral) + (Kd * yPIDderivative);
		yPIDError = yCurrentError;
		oldtime = curTime;
		if (autoAlign) {
			if (aligning && !aligned) {
				DriverStation::ReportError(std::to_string(this->gyro->GetAngle()));
				if (fabs(refAngle - this->gyro->GetAngle()) > 0.35) {
					this->robotDrive->ArcadeDrive(0, sgn(refAngle) * ALIGN_ROTATE_POWER, false);
				}
				else {
					DriverStation::ReportError("Aligned.");
					this->robotDrive->ArcadeDrive(0, 0, false);
					aligning = false;
					aligned = true;
				}
			}
			else {
				aligning = true;
				refAngle = *angle;
				this->gyro->Reset();
				DriverStation::ReportError("Starting alignment...");
			}
		}
		else {
			this->robotDrive->ArcadeDrive(yCurrentSpeed, xCurrentSpeed * 0.85f);
			if (armSet) {
				//float armA = std::stof(SmartDashboard::GetString("DB/String 3", "0"));
				//float armB = std::stof(SmartDashboard::GetString("DB/String 4", "0"));
				if (reset) {
					this->armACanTalon->Set(0);
					this->armBCanTalon->Set(0);
				}
				else if (sally) {
					if (autoFire) {
						this->armACanTalon->Set(SALLY_ARM_A_A);
						this->armBCanTalon->Set(SALLY_ARM_B_A);
					}
					else {
						this->armACanTalon->Set(SALLY_ARM_A_B);
						this->armBCanTalon->Set(SALLY_ARM_B_B);
					}
				}
			}
			aligning = false;
			aligned = false;
		}

		// Get encoder RPMs
		topFireRpm = this->topFireCanTalon->GetSpeed();
		botFireRpm = this->botFireCanTalon->GetSpeed();
		SmartDashboard::PutString("DB/String 5", std::to_string(topFireRpm));
		SmartDashboard::PutString("DB/String 6", std::to_string(botFireRpm));

		fire = topFireRpm > topFireTargetRpm * 0.975f && topFireRpm < topFireTargetRpm * 1.025f && (botFireRpm > botFireTargetRpm * 0.975f && botFireRpm < botFireTargetRpm * 1.025f);

		// Spin fire motors
		if (spin && !this->armMode) {
			this->topFireCanTalon->Set(topFireTargetRpm);
			this->botFireCanTalon->Set(botFireTargetRpm);
		}
		else if (reverseSpin && !this->armMode) {
			this->topFireCanTalon->Set(-topFireTargetRpm);
			this->botFireCanTalon->Set(-botFireTargetRpm);
		}
		else if (!this->armMode) {
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
		this->jetsonNetworkTable->PutBoolean("photo_bool", this->photoSensor->Get());
		if (intake && this->photoSensor->Get()) {
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
	DriverStation::ReportError("Exiting OperatorControl()...");
}

START_ROBOT_CLASS(ManOWar);
