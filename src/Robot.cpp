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

// Harambe Mecanum Code  2017
//RJ-Libraries
//#include "MultiSpeedController.h"

class Robot: public frc::IterativeRobot {

	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();

	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeftFwd;
	VictorSP DriveLeftRear;
	VictorSP DriveRightFwd;
	VictorSP DriveRightRear;
	VictorSP Climb1;
	VictorSP Gear1, Gear2;
	AHRS *ahrs;

	DigitalInput DiIn9, DiIn8, DiIn7;
	float OutputX, OutputY, OutputZ;

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//manipulator



public:
	Robot() :
			Adrive(DriveLeftFwd, DriveLeftRear, DriveRightFwd,DriveRightRear ),
			Drivestick(0), OperatorStick(1), DriveLeftFwd(0), DriveLeftRear(1),
			DriveRightFwd(2), DriveRightRear(3), Climb1(4), Gear1(5), Gear2(6), ahrs(NULL),
			DiIn9(9), DiIn8(8), DiIn7(7), OutputX(0), OutputY(0), OutputZ(0)  {


	}

	void RobotInit() {
		// invert the left side motors
		// you may need to change or remove this to match your robot
		Adrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, false);
		Adrive.SetInvertedMotor(RobotDrive::kRearRightMotor, false);
		Adrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		Adrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);



		//drive command averaging filter
		OutputX = 0, OutputY = 0, OutputZ = 0;



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


	}

	void TeleopInit() {

		OutputX = 0, OutputY = 0, OutputZ = 0;
		if (ahrs) {
			ahrs->ZeroYaw();
		}


	}

	void RobotPeriodic() {

		//displays sensor and motor info to smartDashboard
		SmartDashboardUpdate();
	}
	void DisabledPeriodic() {

	}
	void AutonomousPeriodic() {

	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DriveCreepSpeed = 0.5;
		double GearSpeed = 0.15;
		double ClimbSpeed = 1.0;


		//high gear & low gear controls
		if (Drivestick.GetRawButton(6))
						// High gear press RH bumper
		if (Drivestick.GetRawButton(5))
					// Low gear press LH bumper

		// Temporary high gear when right trigger pushed
		if (Drivestick.GetRawAxis(3) > Deadband) {

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



		//drive controls

		/* Use the joystick X axis for lateral movement, Y axis for forward
			 * movement, and Z axis for rotation. This sample does not use
			 * field-oriented drive, so the gyro input is set to zero.
			 */
		double SpeedLinear = Drivestick.GetRawAxis(1) * 1; // get Left Yaxis value (forward movement)
		double SpeedLateral = Drivestick.GetRawAxis(4) * 1; // get Right Xaxis value (Rotate movement)
		double SpeedLRotate =  Drivestick.GetRawAxis(2) * 1; // get Left Straf value
		double SpeedRRotate = Drivestick.GetRawAxis(3) * 1; // get Right Straf value

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedLateral) < Deadband)
			SpeedLateral = 0.0;
		// Set dead band for R and L Rotate
		if (fabs(SpeedLRotate) < Deadband)
				SpeedLRotate = 0.0;
		if (fabs(SpeedRRotate) < Deadband)
				SpeedRRotate = 0.0;

		//Reduce turn speed when left trigger is pressed
		if (Drivestick.GetRawAxis(2) > Deadband) {
			SpeedLinear = SpeedLinear * DriveCreepSpeed;  // Reduce turn speed
			SpeedLateral = SpeedLateral * DriveCreepSpeed;  // Reduce drive speed
		}


		OutputY = SpeedLinear;
		OutputX = SpeedLateral;
		OutputZ = (SpeedRRotate - SpeedLRotate);


		//Read Gyro Angle
		//double DriveAngle = ahrs->GetAngle();

		Adrive.MecanumDrive_Cartesian(OutputZ, OutputY, OutputX);

		//drive
		if (Drivestick.GetRawButton(4)) {
			//when y button pushed
				}

		/*
		 * MANIP CODE
		 */

		// Turn on the shooter when right hand trigger is pushed
		if (OperatorStick.GetRawAxis(3) > Deadband) {

			} else {

			}



		// Turn on Kicker Wheel, conveyor, and agitators when right trigger is pressed
		if (OperatorStick.GetRawAxis(3) > Deadband ) {
		} else {
		}

		//Spin intake when left trigger is pushed
		if (OperatorStick.GetRawAxis(2) > Deadband) {
		} else {
		}

		// RH Bumper - Release Gear
		if (OperatorStick.GetRawButton(6)) {
			Gear1.Set(GearSpeed);
			Gear2.Set(-GearSpeed);
		}
		// LH Bumper - Hold Gear
		else if (OperatorStick.GetRawButton(5)) {
			Gear1.Set(-GearSpeed);
			Gear2.Set(GearSpeed);
		}
		else {
			Gear1.Set(0);
			Gear2.Set(0);
		}
		//A button for climb
		if (OperatorStick.GetRawButton(1)){
			Climb1.Set(ClimbSpeed);
		}
		else{
			Climb1.Set(0);
		}





	}
	void SmartDashboardUpdate() {


		// PWM displays
		SmartDashboard::PutNumber("Drive L0 Output", DriveLeftFwd.Get());
		SmartDashboard::PutNumber("Drive L1 Output", DriveLeftRear.Get());
		SmartDashboard::PutNumber("Drive R0 Output", DriveRightFwd.Get());
		SmartDashboard::PutNumber("Drive R1 Output", DriveRightRear.Get());
		SmartDashboard::PutNumber("Gear 1 Output", Gear1.Get());
		SmartDashboard::PutNumber("Gear 2 Output", Gear2.Get());
		SmartDashboard::PutNumber("Climb 1 Output", Climb1.Get());


	}



	void TestPeriodic() {
		lw->Run();
	}

private:

}
;

START_ROBOT_CLASS(Robot)

// Read Driver Joystick
//double DRIV_CONTROL_LX = Drivestick.GetRawAxis(0), DriveCONTROL_LY = Drivestick.GetRawAxis(1);
//double DRIV_CONTROL_LTRIG = Drivestick.GetRawAxis(2), DRIV_CONTROL_RTRIG = Drivestick.GetRawAxis(3) ;
//double DRIV_CONTROL_RX = Drivestick.GetRawAxis(4), DRIV_CONTROL_RY=Drivestick.GetRawAxis(5);
//bool DRIV_CONTROL_A = Drivestick.GetRawButton(1), DRIV_CONTROL_B = Drivestick.GetRawButton(2);
//bool DRIV_CONTROL_X = Drivestick.GetRawButton(3), Drivestick.GetRawButton(4)= Drivestick.GetRawButton(4);
//bool DRIV_CONTROL_LBUM = Drivestick.GetRawButton(5), Drivestick.GetRawButton(6) = Drivestick.GetRawButton(6);
//bool DRIV_CONTROL_BACK = Drivestick.GetRawButton(7), DRIV_CONTROL_START = Drivestick.GetRawButton(8);
//bool DRIV_CONTROL_LCLICK = Drivestick.GetRawButton(9), DRIV_CONTROL_RCLICK = Drivestick.GetRawButton(10);

// Read Operator Joystick
//double OPER_CONTROL_LX = OperatorStick.GetRawAxis(0), OPER_CONTROL_LY = OperatorStick.GetRawAxis(1);
//double OPER_CONTROL_LTRIG = OperatorStick.GetRawAxis(2), OPER_CONTROL_RTRIG = OperatorStick.GetRawAxis(3);
//double OPER_CONTROL_RX = OperatorStick.GetRawAxis(4), OPER_CONTROL_RY = OperatorStick.GetRawAxis(5);
//bool OPER_CONTROL_A = OperatorStick.GetRawButton(1), OPER_CONTROL_B = OperatorStick.GetRawButton(2);
//bool OPER_CONTROL_X = OperatorStick.GetRawButton(3), OPER_CONTROL_Y = OperatorStick.GetRawButton(4);
//bool OPER_CONTROL_LBUM = OperatorStick.GetRawButton(5), OPER_CONTROL_RBUM = OperatorStick.GetRawButton(6);
//bool OPER_CONTROL_BACK = OperatorStick.GetRawButton(7), OPER_CONTROL_START = OperatorStick.GetRawButton(8);
//bool OPER_CONTROL_LCLICK = OperatorStick.GetRawButton(9), OPER_CONTROL_RCLICK = OperatorStick.GetRawButton(10);


