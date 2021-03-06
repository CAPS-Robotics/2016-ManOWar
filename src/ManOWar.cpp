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

	this->ledRelay = new Relay(LED_RELAY);

	this->gyro = new AnalogGyro(GYRO_ANALOG);
	this->photoSensor = new DigitalInput(PHOTO_DIGITAL);
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

	DriverStation::ReportError("[RobotInit()] Gyro calibrating...");
	this->gyro->InitGyro();
	this->gyro->Calibrate();
	DriverStation::ReportError("[RobotInit()] Gyro calibrated.");

	SmartDashboard::PutString("DB/String 0", "2550");
	SmartDashboard::PutString("DB/String 1", "2550");
	SmartDashboard::PutString("DB/String 2", "1");
	SmartDashboard::PutString("DB/String 3", "1");
	SmartDashboard::PutString("DB/String 4", "0");
	SmartDashboard::PutNumber("DB/Slider 0", 2.5);

	int cameraBrightness = SmartDashboard::GetNumber("DB/Slider 0", 2.5) * 20;

	CameraServer::GetInstance()->SetQuality(50);
	camera.reset(new USBCamera("cam0", false));
	camera->OpenCamera();
	camera->SetExposureManual(0);
	camera->SetBrightness(cameraBrightness);
	CameraServer::GetInstance()->StartAutomaticCapture(camera);

	this->robotDrive->SetSafetyEnabled(false);
}

void ManOWar::Autonomous() {
	int mode = std::stoi(SmartDashboard::GetString("DB/String 3", "0"));
	DriverStation::ReportError("Running Autonomous() " + std::to_string(mode));
	float *angle = new float(0);
	float *distance = new float(0);

	float topFireTargetRpm;
	float botFireTargetRpm;
	double topFireRpm = 0;
	double botFireRpm = 0;
	bool fire;

	bool aligning = false;
	bool aligned = false;
	float refAngle = std::stof(SmartDashboard::GetString("DB/String 4", "0"));

	if (mode == 0) {
		this->gyro->Reset();

		this->robotDrive->ArcadeDrive(-0.85f, sgn(-this->gyro->GetAngle()) * ALIGN_ROTATE_POWER, false);
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
		Wait(0.1f);
		delete angle;
		delete distance;
	} else if (mode == 1) {
		this->gyro->Reset();

		this->robotDrive->ArcadeDrive(-1.f, 0, false);
		Wait(3.0f);
		this->robotDrive->ArcadeDrive(0, 0, false);
		Wait(1.f);
	} else if (mode == 2) {

	} else if (mode == 3) {
		topFireTargetRpm = 2550;
		botFireTargetRpm = 2550;

		this->robotDrive->ArcadeDrive(-0.5f, -0.05f, false);
		Wait(0.5f);
		this->robotDrive->ArcadeDrive(0, 0, false);
		Wait(1.f);

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

	}
}

void ManOWar::OperatorControl() {
	DriverStation::ReportError("Running OperatorControl()...");
	float *angle = new float(0);
	float *distance = new float(120.f);

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
	bool autoDrop;
//	bool autoAlign;
	//bool armSet;
	//bool port;
	//bool bridge;
	//bool sally;
	//bool frise;
	//bool reset;

//	bool aligning = false;
//	bool aligned = false;
//	float refAngle = 0;

	bool fire;

	float Kp = 0.044000;
	float Ki = 0.000001;
	float Kd = 0.000001;
	float lPIDError = 0;
	float lPIDIntegral = 0;
	float lCurrentSpeed = 0;
	float rPIDError = 0;
	float rPIDIntegral = 0;
	float rCurrentSpeed = 0;
	float xPIDError = 0;
	float xPIDIntegral = 0;
	float xCurrentSpeed = 0;
	float yPIDError = 0;
	float yPIDIntegral = 0;
	float yCurrentSpeed = 0;
	double oldtime = GetTime();

	bool isArcadeDrive = true;
	bool dropped = false;

	while (RobotBase::IsEnabled()) {
		// Grab values from dashboard
		topFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 0", "0"));
		botFireTargetRpm = std::stof(SmartDashboard::GetString("DB/String 1", "0"));
		intakeRpm = std::stof(SmartDashboard::GetString("DB/String 2", "0"));

		camera->SetBrightness(SmartDashboard::GetNumber("DB/Slider 0", 2.5) * 20);

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
		autoDrop = this->joystick->GetRawButton(JOY_BTN_B);

		if (this->joystick->GetPOV() == 0) {
			isArcadeDrive = true;
		} else if (this->joystick->GetPOV() == 180) {
			isArcadeDrive = false;
		}

		if (isArcadeDrive) {
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

			 if (fabs(xCurrentSpeed) >= 0.05) {
				this->robotDrive->ArcadeDrive(yCurrentSpeed, xCurrentSpeed * 0.85f);
			 } else {
				this->robotDrive->ArcadeDrive(yCurrentSpeed, sgn(-this->gyro->GetAngle()) * ALIGN_ROTATE_POWER);
			 }

		} else {
			// Drive
			double curTime = GetTime();
			// L PID loop
			float lCurrentError = joystick->GetRawAxis(JOY_AXIS_LY) - lCurrentSpeed;
			lPIDIntegral += lPIDError * (curTime - oldtime);
			float lPIDderivative = (lCurrentError - lPIDError) / (curTime - oldtime);
			lCurrentSpeed += (Kp * lCurrentError) + (Ki * lPIDIntegral) + (Kd * lPIDderivative);
			lPIDError = lCurrentError;
			// R PID loop
			float rCurrentError = joystick->GetRawAxis(JOY_AXIS_RY) - rCurrentSpeed;
			rPIDIntegral += rPIDError * (curTime - oldtime);
			float rPIDderivative = (rCurrentError - rPIDError) / (curTime - oldtime);
			rCurrentSpeed += (Kp * rCurrentError) + (Ki * rPIDIntegral) + (Kd * rPIDderivative);
			rPIDError = rCurrentError;
			oldtime = curTime;

			this->robotDrive->TankDrive(lCurrentSpeed, rCurrentSpeed);
		}


		// Get encoder RPMs
		topFireRpm = this->topFireCanTalon->GetSpeed();
		botFireRpm = this->botFireCanTalon->GetSpeed();
		SmartDashboard::PutString("DB/String 5", std::to_string(topFireRpm));
		SmartDashboard::PutString("DB/String 6", std::to_string(botFireRpm));

		fire = topFireRpm > topFireTargetRpm * 1.f && topFireRpm < topFireTargetRpm * 1.025f && (botFireRpm > botFireTargetRpm * 1.f && botFireRpm < botFireTargetRpm * 1.025f);

		if (spin && !dropped) {
			autoDrop = true;
			spin = false;
		}

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
			SmartDashboard::PutBoolean("DB/LED 1", true);
			if (spin && autoFire) {
				this->intakeTalon->Set(intakeRpm);
				intakeOverride = true;
				SmartDashboard::PutBoolean("DB/LED 0", false);
			}
			else {
				intakeOverride = false;
			}
		}
		else {
			SmartDashboard::PutString("DB/String 7", "CEASE_FIRE");
			intakeOverride = false;
			SmartDashboard::PutBoolean("DB/LED 1", false);
		}

		if (autoDrop && !this->photoSensor->Get()) {
			SmartDashboard::PutString("DB/String 8", "Dropping");
			this->intakeTalon->Set(-intakeRpm / 1.5f);
		} else if (autoDrop && this->photoSensor->Get()) {
			SmartDashboard::PutString("DB/String 8", "Dropped");
			SmartDashboard::PutBoolean("DB/LED 0", true);
			this->intakeTalon->Set(0.f);
			dropped = true;
		}
		// Intake motors
		else if (intake && this->photoSensor->Get()) {
			// If spinning ball in
			this->intakeTalon->Set(intakeRpm);
		} else if (intake && !this->photoSensor->Get()) {
			// If balled is picked up
			SmartDashboard::PutString("DB/String 8", "Picked Up");
			this->intakeTalon->Set(0.f);
		} else if (reverseIntake) {
			// Reverse intake if ball loaded
			this->intakeTalon->Set(-intakeRpm);
		} else if (!intakeOverride) {
			// Otherwise, just don't run it
			this->intakeTalon->Set(0.f);
		}
	}

	Wait(0.1f);
	delete angle;
	delete distance;
	DriverStation::ReportError("Exiting OperatorControl()...");
}

START_ROBOT_CLASS(ManOWar);
