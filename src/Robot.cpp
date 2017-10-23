#include <Joystick.h>
#include <RobotDrive.h>
#include <SampleRobot.h>
#include <Timer.h>

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
class Robot: public frc::SampleRobot {
public:
	Robot() {
		robotDrive.SetExpiration(0.1);

		// Invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);

		// You may need to change or remove this to match your robot
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl() override {
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled()) {
			/* Use the joystick X axis for lateral movement, Y axis for forward
			 * movement, and Z axis for rotation. This sample does not use
			 * field-oriented drive, so the gyro input is set to zero.
			 */
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(),
					stick.GetZ());

			frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

private:
	// Channels for the wheels
	static constexpr int kFrontLeftChannel = 2;
	static constexpr int kRearLeftChannel = 3;
	static constexpr int kFrontRightChannel = 1;
	static constexpr int kRearRightChannel = 0;

	static constexpr int kJoystickChannel = 0;

	// Robot drive system
	frc::RobotDrive robotDrive { kFrontLeftChannel, kRearLeftChannel,
			kFrontRightChannel, kRearRightChannel };
	// Only joystick
	frc::Joystick stick { kJoystickChannel };
};

START_ROBOT_CLASS(Robot)


#include <iostream>
#include <memory>
#include <string>
#include "AHRS.h"
#include "WPILib.h"
#include "math.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>


class Robot: public frc::IterativeRobot {

	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooseAutonSelector, chooseDriveEncoder,
			chooseDeflector, chooseKicker, chooseShooter, chooseDeflectorLimit;
	const std::string AutonNameSwitch = "Use Switch";
	const std::string autonNameOFF = "0 OFF";
	const std::string autonNameBlue1 = "Blue 1";
	const std::string autonNameBlue2 = "Blue 2";
	const std::string autonNameBlue3 = "Blue 3";
	const std::string autonNameRed1 = "Red 1";
	const std::string autonNameRed2 = "Red 2";
	const std::string autonNameRed3 = "Red 3";
	const std::string RH_Encoder = "RH_Encoder";
	const std::string LH_Encoder = "LH_Encoder";
	const std::string chooserClosedLoop = "Closed Loop";
	const std::string chooserOpenLoop = "Open Loop";
	const std::string Disable = "Disable";
	const std::string Enable = "Enable";
	std::string autoSelected, encoderSelected;
	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	Timer AutonTimer;
	Timer ShooterDelay;
	Encoder EncoderLeft;
	Encoder EncoderRight;
	std::shared_ptr<NetworkTable> table;
	AHRS *ahrs;
//tells us what state we are in in each auto mode
	int modeState;bool AutonOverride, AutoSw1, AutoSw2, AutoSw3;
	DigitalInput DiIn9, DiIn8, DiIn7;
	int AutoVal, AutoVal0, AutoVal1, AutoVal2;
	float OutputX, OutputY;
	std::shared_ptr<NetworkTable> GRIPTable;
	int isWaiting = 0;			/////***** Divide this into 2 variables.

	Solenoid *driveSolenoid = new Solenoid(0);
	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//manipulator
	VictorSP Winch0, Winch1;
	VictorSP Shooter0, Shooter1;
	VictorSP Conveyor;
	VictorSP Agitator0, Agitator1;
	VictorSP FloorIntakeRoller;
	VictorSP KickerWheel;
	VictorSP DeflectorMotor;
	Solenoid *FloorIntakeArm = new Solenoid(2);
	Solenoid *GearIn = new Solenoid(3);
	Solenoid *GearOut = new Solenoid(1);
	Encoder EncoderKicker;
	Encoder EncoderShoot;
	DigitalInput WinchStop;
	AnalogPotentiometer DeflectorAnglePOT;
	double DeflectorTarget, IntakeCommandPWM, AgitatorCommandPWM,
			ConvCommandPWM, ShootCommandRPM, ShootCommandPWM, DeflectAngle,
			KickerCommandRPM, KickerCommandPWM, ShootKP, ShootKI, ShootKD,
			ShootKF;
	DigitalInput DeflectorHighLimit, DeflectorLowLimit;

	bool useRightEncoder;bool DeflectorClosedLoop;bool KickerClosedLoop;bool ShooterClosedLoop;bool DeflectorLimitEnabled;

	PIDController DeflectorPID, KickerPID, ShooterPID, DrivePID;

	bool driveRightTriggerPrev = false;bool driveButtonYPrev = false;bool operatorRightTriggerPrev =
	false;bool intakeDeployed = false;

	double autoBackupDistance;
	double deflectorTargetMemory;
//MultiSpeedController *Shooter;


public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), EncoderLeft(0, 1), EncoderRight(2, 3), table(
			NULL), ahrs(NULL), modeState(0), DiIn9(9), DiIn8(8), DiIn7(7), Winch0(
					8), Winch1(9), Shooter0(12), Shooter1(7), Conveyor(13), Agitator0(
					6), Agitator1(15), FloorIntakeRoller(14), KickerWheel(11), DeflectorMotor(
					10), EncoderKicker(20, 21), EncoderShoot(4, 5), WinchStop(
					6), DeflectorAnglePOT(0, 270, 0), DeflectorTarget(0), ConvCommandPWM(
					0.1), ShootCommandPWM(0.75), DeflectAngle(145), DeflectorHighLimit(
					22), DeflectorLowLimit(23), DeflectorPID(-0.03, 0.0, 0.0,
					&DeflectorAnglePOT, &DeflectorMotor), KickerPID(0.03, 0.0,
					0.0, &EncoderKicker, &KickerWheel), ShooterPID(0.0, 0.0,
					-0.003, 0.0, &EncoderShoot, &Shooter0, 0.010), DrivePID(0.0, 0.0, 0.0,
					0.0, &EncoderRight, &DriveRight0) {

		//GRIPTable = NetworkTable::GetTable("GRIP/myContuorsReport");
		//Shooter = new MultiSpeedController();

	}

	void RobotInit() {
		//setup smartDashboard choosers
		chooseAutonSelector.AddDefault(AutonNameSwitch, AutonNameSwitch);
		chooseAutonSelector.AddObject(autonNameOFF, autonNameOFF);
		chooseAutonSelector.AddObject(autonNameRed1, autonNameRed1);
		chooseAutonSelector.AddObject(autonNameRed2, autonNameRed2);
		chooseAutonSelector.AddObject(autonNameRed3, autonNameRed3);
		chooseAutonSelector.AddObject(autonNameBlue1, autonNameBlue1);
		chooseAutonSelector.AddObject(autonNameBlue2, autonNameBlue2);
		chooseAutonSelector.AddObject(autonNameBlue3, autonNameBlue3);
		frc::SmartDashboard::PutData("Auto Modes", &chooseAutonSelector);

		chooseDriveEncoder.AddDefault(RH_Encoder, RH_Encoder);
		chooseDriveEncoder.AddObject(LH_Encoder, LH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseDriveEncoder);

		chooseDeflector.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseDeflector.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Deflector", &chooseDeflector);

		chooseKicker.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseKicker.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Kicker", &chooseKicker);

		chooseShooter.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseShooter.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Shooter", &chooseShooter);

		chooseDeflectorLimit.AddDefault(Enable, Enable);
		chooseDeflectorLimit.AddObject(Disable, Disable);
		frc::SmartDashboard::PutData("Deflector Limits", &chooseDeflectorLimit);

		// Inialize settings from Smart Dashboard
		ShootCommandPWM = 0.8;
		ShootCommandRPM = 2800;
		ShootKP = -0.003;
		ShootKI = 0.0;
		ShootKD = 0.0;
		ShootKF = -(1.0 / 3200.0);  // 1 / (MAX  RPM with full power) (UPDATE fOR COMP BOT)
 		KickerCommandPWM = 0.75;
		KickerCommandRPM = 500;
		DeflectorTarget = 170;
		ConvCommandPWM = 0.75;
		AgitatorCommandPWM = 0.75;
		IntakeCommandPWM = 0.75;
		autoBackupDistance = -7.0;

		SmartDashboard::PutNumber("IN: Shooter CMD (PWM)", ShootCommandPWM);
		SmartDashboard::PutNumber("IN: Shooter CMD (RPM)", ShootCommandRPM);
		SmartDashboard::PutNumber("IN: ShootKP", ShootKP);
		SmartDashboard::PutNumber("IN: ShootKI", ShootKI);
		SmartDashboard::PutNumber("IN: ShootKD", ShootKD);
		SmartDashboard::PutNumber("IN: ShootKF", ShootKF);
		SmartDashboard::PutNumber("IN: Kicker CMD (PWM)", KickerCommandPWM);
		SmartDashboard::PutNumber("IN: Kicker CMD (RPM)", KickerCommandRPM);
		SmartDashboard::PutNumber("IN: Deflector CMD (DEG)", DeflectorTarget);
		SmartDashboard::PutNumber("IN: Conveyor CMD (PWM)", ConvCommandPWM);
		SmartDashboard::PutNumber("IN: Agitator CMD (PWM)", AgitatorCommandPWM);
		SmartDashboard::PutNumber("IN: Intake CMD (PWM)", IntakeCommandPWM);
		SmartDashboard::PutNumber("IN: Auto Backup Distance (Inch)",
				autoBackupDistance);

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs ADLAI - Is this something we should be doing?
		Adrive.SetSafetyEnabled(false);

		//changes these original negative values to positive values
		EncoderLeft.SetReverseDirection(true);
		EncoderRight.SetReverseDirection(false);
		EncoderShoot.SetReverseDirection(true);
		EncoderKicker.SetReverseDirection(true);

		//calibrations for encoders
		EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderShoot.SetDistancePerPulse(1.0 / 3328.0 * 4.0);
		EncoderKicker.SetDistancePerPulse(1.0 / 4122.0 * 4.0);

		EncoderShoot.SetSamplesToAverage(8);

		//configure PIDs
		DeflectorPID.SetOutputRange(-0.1, 0.1);

		//drive command averaging filter ADLAI - Does this just make sure joysticks are reading 0? Should it be both here and in teleopinit?
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading for autonomous mode
		useRightEncoder = true;

		// Turn off the the sensors/reset
		//from NAVX mxp data monitor example
		try { /////***** Let's do this differently.  We want Auton to fail gracefully, not just abort. Remember Ariane 5
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP, 200);
			ahrs->Reset();
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		// This gives the NAVX time to reset.
		// It takes about 0.5 seconds for the reset to complete.
		// RobotInit runs well before the autonomous mode starts,
		//		so there is plenty of time.
		Wait(1);
	}

	void AutonomousInit() override {
		modeState = 1;
		isWaiting = 0;							/////***** Rename this.

		AutonTimer.Reset();
		AutonTimer.Start();
		// Encoder based auton
		EncoderLeft.Reset();
		EncoderRight.Reset();
		// Turn off drive motors
		DriveLeft0.Set(0);
		DriveLeft1.Set(0);
		DriveLeft2.Set(0);
		DriveRight0.Set(0);
		DriveRight1.Set(0);
		DriveRight2.Set(0);
		//zeros the navX
		if (ahrs) {
			ahrs->ZeroYaw();
		}

		DeflectorPID.Reset();

		//forces robot into low gear
		driveSolenoid->Set(false);

		//makes sure gear doesn't eject
		GearOut->Set(false);

	}

	void TeleopInit() {

		OutputX = 0, OutputY = 0;


	}

	void RobotPeriodic() {
		//links multiple motors together
		Winch1.Set(-Winch0.Get());
		Shooter1.Set(-Shooter0.Get());
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());
		Agitator1.Set(Agitator0.Get());

		//displays sensor and motor info to smartDashboard
		SmartDashboardUpdate();
	}
	void DisabledPeriodic() {
		//SmartDashboardUpdate();
	}
	void AutonomousPeriodic() {
	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DriveCreepSpeed = 0.5;

		//high gear & low gear controls
		if (Drivestick.GetRawButton(6))
			driveSolenoid->Set(true);			// High gear press RH bumper
		if (Drivestick.GetRawButton(5))
			driveSolenoid->Set(false);			// Low gear press LH bumper

		// Temporary high gear when right trigger pushed
		if (Drivestick.GetRawAxis(3) > Deadband) {
			driveSolenoid->Set(true);
			driveRightTriggerPrev = true;
		} else if (driveRightTriggerPrev) {
			driveSolenoid->Set(false);
			driveRightTriggerPrev = false;
		}

		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//  Read climber motor current from PDP and display on drivers station
		double climberCurrentLeft = pdp->GetCurrent(3);
		double climberCurrentRight = pdp->GetCurrent(12);
		SmartDashboard::PutNumber("DBG climber current", climberCurrentLeft);

		// rumble if current to high
		double LHClimb = 0.0;		// Define value for rumble
		double climberMaxCurrent = 30.0;	//needs to be changed... 30.0 is a guess
		if (climberCurrentLeft > climberMaxCurrent)	// Rumble Left if greater than climberMaxCurrent
			LHClimb = 0.5;
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		OperatorStick.SetRumble(Vibrate, LHClimb); // Set Left Rumble to LH Trigger

		double RHClimb = 0.0;		// Define value for rumble
		if (climberCurrentRight > climberMaxCurrent)	// Rumble Right if greater than climberMaxCurrent
			RHClimb = 0.5;
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		OperatorStick.SetRumble(Vibrate, RHClimb);// Set Right Rumble to RH Trigger

		//drive controls ADLAI - I know this works, I just don't understand how returning only negative values works for this.
		double SpeedLinear = Drivestick.GetRawAxis(1) * -1; // get Yaxis value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * -1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Deadband)
			SpeedRotate = 0.0;

		//Reduce turn speed when left trigger is pressed
		if (Drivestick.GetRawAxis(2) > Deadband) {
			SpeedLinear = SpeedLinear * DriveCreepSpeed;  // Reduce turn speed
			SpeedRotate = SpeedRotate * DriveCreepSpeed;  // Reduce drive speed
		}

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		//drive
		if (Drivestick.GetRawButton(4)) {
			//boiler auto back up when y button pushed
			if (!driveButtonYPrev) {
				EncoderRight.Reset();
				EncoderLeft.Reset();
				ahrs->ZeroYaw();
				driveButtonYPrev = true;
			}
		} else {
			//manual control
			driveButtonYPrev = false;
			Adrive.ArcadeDrive(OutputY, OutputX, true);
		}
	}

	void SmartDashboardUpdate() {

		// Auto State
		SmartDashboard::PutNumber("Auto Switch (#)", AutoVal);
		SmartDashboard::PutString("Auto Program", autoSelected);
		SmartDashboard::PutNumber("Auto State (#)", modeState);
		SmartDashboard::PutNumber("Auto Timer (s)", AutonTimer.Get());

		// Drive Encoders
		SmartDashboard::PutNumber("Drive Encoder Left (RAW)",
				EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Left (Inches)",
				EncoderLeft.GetDistance());

		SmartDashboard::PutNumber("Drive Encoder Right (RAW)",
				EncoderRight.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Right (Inch)",
				EncoderRight.GetDistance());

		encoderSelected = chooseDriveEncoder.GetSelected();
		useRightEncoder = (encoderSelected == RH_Encoder);

		autoBackupDistance = SmartDashboard::GetNumber(
				"IN: Auto Backup Distance (Inch)", autoBackupDistance);
		SmartDashboard::PutNumber("Auto Backup Distance", autoBackupDistance);

		// Gyro
		if (ahrs) {
			double gyroAngle = ahrs->GetAngle();
			SmartDashboard::PutNumber("Gyro Angle", gyroAngle);
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999);
		}

		/*
		 * Manipulator
		 */
		// PWM displays
		SmartDashboard::PutNumber("Drive L0 Output", DriveLeft0.Get());
		SmartDashboard::PutNumber("Drive L1 Output", DriveLeft1.Get());
		SmartDashboard::PutNumber("Drive L2 Output", DriveLeft2.Get());
		SmartDashboard::PutNumber("Drive R0 Output", DriveRight0.Get());
		SmartDashboard::PutNumber("Drive R1 Output", DriveRight1.Get());
		SmartDashboard::PutNumber("Drive R2 Output", DriveRight2.Get());

		SmartDashboard::PutNumber("Shooter Motor0 Output", Shooter0.Get());
		SmartDashboard::PutNumber("Shooter Motor1 Output", Shooter1.Get());
		SmartDashboard::PutNumber("Winch Motor0 Output", Winch0.Get());
		SmartDashboard::PutNumber("Winch Motor1 Output", Winch1.Get());
		SmartDashboard::PutNumber("Kicker Motor Output", KickerWheel.Get());
		SmartDashboard::PutNumber("Agitator Motor0 Output", Agitator0.Get());
		SmartDashboard::PutNumber("Agitator Motor1 Output", Agitator1.Get());
		SmartDashboard::PutNumber("Floor Intake Motor Output",
				FloorIntakeRoller.Get());
		SmartDashboard::PutNumber("Conveyor Motor  Output", Conveyor.Get());
		SmartDashboard::PutNumber("Deflector Motor Output",
				DeflectorMotor.Get());

		//chooser code for manip in open/closed loop

		//**TEST
		SmartDashboard::PutNumber("DBG POV", OperatorStick.GetPOV(0));


	}

	void motorSpeed(double leftMotor, double rightMotor) { // ADLAI - Possibly redundant with the previous drive motor setup in robotperiodic
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
	}

	int timedGyroDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			double driveCommandRotation = ahrs->GetAngle() * KP_ROTATION;
			motorSpeed(-leftMotorSpeed + driveCommandRotation,
					-rightMotorSpeed - driveCommandRotation);
		} else {
			stopMotors();
			return 1;
		}
		return 0;
	}

	void TestPeriodic() {
		lw->Run();
	}

START_ROBOT_CLASS(Robot)

