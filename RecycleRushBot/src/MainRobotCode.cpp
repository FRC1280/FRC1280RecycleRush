//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2015 Recycle Rush ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                      // header file
#include <cmath>

#include "../H/CameraLights.h"
#include "../H/Grabber.h"
#include "../H/Elevator.h"

#define CONSOLE

//------------------------------------------------------------------------------
// DEFINE RecycleRushRobot CLASS
//------------------------------------------------------------------------------
// Derived from IterativeRobot class in WPILib.h.  Inherits attributes and
// functions of the base IterativeRobot class.
// Extends class through:
// - Overrides to virtual methods contained in the IterativeRobot base class
// - Methods specific to the 2012 Team 1280 Rebound Rumble robot
// - Constants used by class
// - Pointers to required objects
// - Additional local variables required
//------------------------------------------------------------------------------
class RecycleRushRobot : public IterativeRobot
{
	//--------------------------------------------------------------------------
	// DECLARATION OF PUBLIC METHODS
	//--------------------------------------------------------------------------
	public:
		// Constructor and destructor methods
		RecycleRushRobot();
		~RecycleRushRobot();

		//----------------------------------------------------------------------
		// OVERRIDES TO IterativeRobot BASE CLASS VIRTUAL METHODS
		//----------------------------------------------------------------------
		// Robot and state initialization methods
		void   RobotInit();
		void   DisabledInit();
		void   AutonomousInit();
		void   TeleopInit();

		// Robot periodic methods performed in loops
		void   DisabledPeriodic();
		void   AutonomousPeriodic();
		void   TeleopPeriodic();

		//----------------------------------------------------------------------
		// CUSTOM METHODS SPECIFIC TO TEAM 1280 ROBOT
		//----------------------------------------------------------------------
		// Initialization and reset methods

		// Driver station and robot input gathering methods
		void   GetDriverStationInput();
		void   GetRobotSensorInput();
		void   ShowDSValues();
		void   ShowRobotValues();

		// Miscellaneous robot function methods
		int  GetTime();

		// Autonomous mode methods
		void   GetAutoModeSwitches();
		void   RunAutonomousMode();
		void   AMDriveRobot();
		void   ShowAMStatus();

	private:
		//----------------------------------------------------------------------
		// CONSTANTS USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION PORTS AND CHANNELS
		//----------------------------------------------------------------------
		// Driver Station Joystick ports

		static const uint JS_PORT          =  0;
		static const uint CCI_PORT         =  1;  // eStop Robots CCI Inputs

		// Driver Station CCI Channels (Uses joystick button references)
		static const uint LED_LIGHTS_SW_CH          =  1;
		static const uint GRABBER_SW_CH			    =  3;
		static const uint ELEVATOR_AUTO_SW_CH	    =  4;
		static const uint ELEVATOR_GROUND_SW_CH     =  5;
		static const uint ELEVATOR_POSITION1_SW_CH  =  5;
		static const uint ELEVATOR_POSITION2_SW_CH  =  6;
		static const uint ELEVATOR_POSITION3_SW_CH  =  7;
		static const uint ELEVATOR_POSITION4_SW_CH  =  8;
		static const uint ELEVATOR_POSITION5_SW_CH  =  9;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------

		// cRIO Digital Sidecar Channels - GPIO & Spare Power Outputs
		// Robot inputs
		// - Air compressor pressure switch (indicates when pressure is
		//   too high)
		static const uint PRESSURE_SW_CH           = 1;
		// Autonomous Mode switches

		// cRIO Analog Breakout Input - Banner Sensors
		static const uint ELEVATOR_POT_CH		   = 1;
		static const uint DRIVE_GYRO_CH			   = 2;
		// Robot inputs
		// Banner Sensors for elevator, loader and shooter positions
		static const uint TOP_LIMIT_SW_CH		   =  0;
		static const uint BOTTOM_LIMIT_SW_CH	   =  1;


		// cRIO Digital Sidecar Channels - Robot PWM Controls (Outputs)
		// Motors with Victor & Talon speed controllers
		// PWM = Pulsed width modulation
		static const uint LEFT_FRONT_MOTOR_CH	   = 0;
		static const uint LEFT_REAR_MOTOR_CH	   = 1;
		static const uint RIGHT_FRONT_MOTOR_CH	   = 2;
		static const uint RIGHT_REAR_MOTOR_CH      = 3;
		static const uint BELT_MOTOR_CH			   = 4;

		// cRIO Digital Sidecar Channels - Robot Relay Controls (Outputs)
		static const uint COMPRESSOR_CH         =  1;
		static const uint CAMERA_LIGHTS_CH      =  2;

		// cRIO Solenoid Breakout Channels
		static const uint GRABBER_CH			=  1;

		//----------------------------------------------------------------------
		// CONSTANTS USED TO DETERMINE STATUS OF DRIVER STATION AND ROBOT INPUTS
		// AND TO INSTRUCT ROBOT
		// Private static constants used in multiple methods
		//----------------------------------------------------------------------
		// CONSTANTS USED IN DECLARING OBJECTS
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// AUTONOMOUS MODE ROBOT CONTROL CONSTANTS (OUTPUTS)
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// AUTONOMOUS MODE ROBOT STATE & TIMING TRACKING
		// Used to determine what robot is or should be doing in autonomous mode
		//----------------------------------------------------------------------

		//----------------------------------------------------------------------
		// POINTERS FOR REQUIRED OBJECTS
		//----------------------------------------------------------------------
		// DRIVER STATION INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------
		// Includes driver station laptop, joysticks, switches and other digital
		// and analog devices connected through the eStop Robotics CCI.
		//----------------------------------------------------------------------
		DriverStation	 *pDriverStation;
		Joystick		 *pDriveStick;

		// eStop Robotics Custom Control Interface (CCI)
		Joystick         *pCCI;                 // CCI
		JoystickButton 	 *pCameraLightSwitch;   // CCI Digital Inputs
		JoystickButton	 *pGrabberSwitch;
		JoystickButton   *pElevatorAutoSwitch;
		JoystickButton   *pElevatorGroundSwitch;
		JoystickButton   *pElevatorPostition1Switch;
		JoystickButton   *pElevatorPostition2Switch;
		JoystickButton   *pElevatorPostition3Switch;
		JoystickButton   *pElevatorPostition4Switch;
		JoystickButton   *pElevatorPostition5Switch;

		//----------------------------------------------------------------------
		// ROBOT INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------

		//Analog Inputs
		Gyro            *pDriveGyro;
		//----------------------------------------------------------------------
		// Robot Digital Inputs - GPIO Inputs including Encoders
		//----------------------------------------------------------------------
		// Autonomous Mode Switches
	
		//----------------------------------------------------------------------
		// Robot Digital Outputs - Relays (Spikes)
		//----------------------------------------------------------------------
		CameraLights	*pCameraLights;			// Camera LED lights
		//----------------------------------------------------------------------
		// Robot Objects
		//----------------------------------------------------------------------
		RobotDrive		*pDriveTrain;
		Grabber 		*pGrabber;
		Elevator		*pElevator;
		//----------------------------------------------------------------------
		// VARIABLES USED IN CLASS
		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Analog inputs from joysticks and
		// eStop Robotics CCI.
		//----------------------------------------------------------------------
		// Joystick drive speed inputs for tank drive
		// Disk shooting aim angle position
		// - Manual input based on potentionmeter setting
		// Elevator Position
		// - Manual input based on potentiometer setting
		//----------------------------------------------------------------------
		float  leftDriveSpeed;
		float  rightDriveSpeed;

		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Digital Inputs from eStop Robotics CCI
		//----------------------------------------------------------------------
		// Camera Switches
		bool   lightsOn;
		// Robot Switches
		bool   grabberIn;
		float  elevatorTarget;

		//----------------------------------------------------------------------
		// CLASS VARIABLES USED TO TRACK ROBOT STATUS
		//----------------------------------------------------------------------
		// General status tracking
		//----------------------------------------------------------------------
		// Class variables to track look (packet) counts
		uint loopCount;
        float  loopsPerMinute;

        // Class variables to track time
		int  startSec;
		int  elapsedSec;
		int  oldSec;

		//----------------------------------------------------------------------
		// Camera Image Processing
		//----------------------------------------------------------------------
		
		//----------------------------------------------------------------------
		// Grabber Speed / Shifting
		//----------------------------------------------------------------------
		int    tranSpeed;            // Grabber speed

		//----------------------------------------------------------------------
		// Autonomous Mode Switches & variables
		//----------------------------------------------------------------------

};
//------------------------------------------------------------------------------
// INITIALIZE STATIC CONSTANTS
//------------------------------------------------------------------------------

START_ROBOT_CLASS(RecycleRushRobot);

//------------------------------------------------------------------------------
// METHOD DEFINITONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::RecycleRushRobot
// Type:	Public default constructor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Defines pointers to required robot objects and initializes packet count and
// loop count variables.
//------------------------------------------------------------------------------
RecycleRushRobot::RecycleRushRobot()
{
	//----------------------------------------------------------------------
	// DEFINE POINTERS TO REQUIRED ROBOT OBJECTS
	//----------------------------------------------------------------------
	// DRIVER STATION INPUTS
	//----------------------------------------------------------------------
	// Acquire the Driver Station object
	pDriverStation = DriverStation::GetInstance();

	// Define joysticks & CCI
	pDriveStick			 = new Joystick(JS_PORT);
	pCCI                 = new Joystick(CCI_PORT); // CCI uses joystick object

    // Joystick Buttons

	// CCI Switches
	pCameraLightSwitch   			 = new JoystickButton(pCCI,LED_LIGHTS_SW_CH);
	pGrabberSwitch		 			 = new JoystickButton(pCCI,GRABBER_SW_CH);
	pElevatorAutoSwitch				 = new JoystickButton(pCCI, ELEVATOR_AUTO_SW_CH);
	pElevatorGroundSwitch			 = new JoystickButton(pCCI, ELEVATOR_GROUND_SW_CH);
	pElevatorPostition1Switch		 = new JoystickButton(pCCI, ELEVATOR_POSITION1_SW_CH);
	pElevatorPostition2Switch		 = new JoystickButton(pCCI, ELEVATOR_POSITION2_SW_CH);
	pElevatorPostition3Switch		 = new JoystickButton(pCCI, ELEVATOR_POSITION3_SW_CH);
	pElevatorPostition4Switch		 = new JoystickButton(pCCI, ELEVATOR_POSITION4_SW_CH);
	pElevatorPostition5Switch		 = new JoystickButton(pCCI, ELEVATOR_POSITION5_SW_CH);

	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	
	pDriveGyro           = new Gyro(DRIVE_GYRO_CH);
	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	// Spike Relays (Relay Connections)
	// lights
	pCameraLights		 = new CameraLights(CAMERA_LIGHTS_CH);

	// Drive Train
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
									      RIGHT_FRONT_MOTOR_CH,RIGHT_REAR_MOTOR_CH);

	pGrabber 			 = new Grabber(GRABBER_CH);

	pElevator			 = new Elevator(BELT_MOTOR_CH, ELEVATOR_POT_CH,TOP_LIMIT_SW_CH,BOTTOM_LIMIT_SW_CH );
	
	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop and time counters
	loopCount  = 0;
	startSec   = 0;
	elapsedSec = 0;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::~RecycleRushRobot
// Type:	Public default destructor for RecycleRushRobot class
//------------------------------------------------------------------------------
RecycleRushRobot::~RecycleRushRobot()
{
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::RobotInit()
// Type:	Performs robot initiation functions.  Overrides RobotInit() virtual
//          method contained in WPILib.
//
//			These actions are performed once and only once when the robot is
//	        powered on.
//------------------------------------------------------------------------------
// Functions:
// - Initializes the SmartDashboard
// - Turns air compressor on
// - Sets type of input used to determine elevator position
// - Sets target values for three preset elevator positions
// - Initializes robot status tracking settings
//------------------------------------------------------------------------------
void RecycleRushRobot::RobotInit()
{
	SmartDashboard::init();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::DisabledInit()
// Type:	Executes when the robot is placed in Disabled mode.  Overrides
//			DisabledInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets loop counter for disabled mode
// - Starts the clock that counts how long robot has been in disabled mode
//------------------------------------------------------------------------------
void RecycleRushRobot::DisabledInit()
{
	// Reset loop counter
	loopCount  = 0;

	// Capture time disabled mode was entered
	elapsedSec = 0;
	startSec   = (int)GetClock();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::AutonomousInit()
// Type:	Executes when the robot is placed in Autonomous mode.  Overrides
//			AutonomousInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counter for autonomous mode
// - Resets the AutoState
// - Resets the encoders on the wheels that measure distance traveled.
// - Shifts the Grabber into low gear
// - Optionally prints the status of the autonomous mode switches for debugging
//   purposes
//------------------------------------------------------------------------------
void RecycleRushRobot::AutonomousInit()
{
	// Reset loop counter
	loopCount  = 0;

	// Capture time autonomous mode was entered
	elapsedSec = 0;
	startSec   = (int)GetClock();
	
	GetRobotSensorInput();
		
	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::TeleopInit()
// Type:	Executes when the robot is placed in Teleoperated mode.  Overrides
//			TeleopInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop and packet counters for teleoperated mode
// - Resets the distance tracking variables
// - Resets the wheel encoder counters
// - Obtain the current position of the elevator from the robot
//------------------------------------------------------------------------------
void RecycleRushRobot::TeleopInit()
{
	// General loop count & elapsed time initialization
	loopCount      = 0;
	loopsPerMinute = 0;

	elapsedSec = 0;
	startSec   = (int)GetClock();
	oldSec     = startSec;

	GetDriverStationInput();
	GetRobotSensorInput();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::DisabledPeriodic()
// Type:	Executes when the robot is in disabled mode.  Overrides the
//			DisabledPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Define a variable that captures the time this method is invoked.
// - Feed watchdog to prevent robot shut-down
// - Increment the disabled loop counter
// - Increments the time the robot is in disabled mode
// - Optionally can print to the console the time robot is in disabled mode
//------------------------------------------------------------------------------
void RecycleRushRobot::DisabledPeriodic()
{

    // Increment & display loop counter
	loopCount++;

	// Calculate & display elapsed seconds
	elapsedSec = (int)GetClock() - startSec;
	
	GetAutoModeSwitches();
	ShowRobotValues();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::AutonomousPeriodic()
// Type:	Executes when the robot is in autonomous mode.  Overrides the
//			AutonomousPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops while in autonomous mode.
// - Feeds to watchdog to prevent robot shut-down
//------------------------------------------------------------------------------
void RecycleRushRobot::AutonomousPeriodic()
{
    // Increment & display loop counter
	loopCount++;

	// Calculate & display elapsed seconds
	elapsedSec = (int)GetClock() - startSec;


	GetRobotSensorInput();
	
	ShowRobotValues();

	ShowAMStatus();
	
	RunAutonomousMode();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::TeleopPeriodic()
// Type:	Executes when the robot is in teleoperated mode each time a new
//          packet of information has been received by the Driver Station.  Any
//          code which needs new information from the Driver Station should be
//          placed here.  This method overrides the TeleopPeriodic() virtual
//          method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops processed and packets received while
//   in teleoperated mode.
// - Feeds to watchdog to prevent robot shut-down
// - Obtains input from the driver station (joystick inputs, switches, arm
//   rotator potentiometer)
// - If a Grabber shift was requested by the driver station, shift
//   Grabber
// - Sets the drive motor values based on joystick movement
// - Sets the ball launcher motor speed values based on on/off switch and
//   potentiometer position
//------------------------------------------------------------------------------
void RecycleRushRobot::TeleopPeriodic()
{
	// Increment & display loop counter
	loopCount++;

	// Calculate & display elapsed seconds
	elapsedSec = (int)GetClock() - startSec;

	// Get inputs from the driver station
	GetDriverStationInput();
	GetAutoModeSwitches();
	
	// Get robot sensor input
	GetRobotSensorInput();

	//Set Drive Speed
	pDriveTrain->MecanumDrive_Cartesian (pDriveStick->GetX(), pDriveStick->GetY(), pDriveStick->GetTwist()); //,pDriveGyro->GetAngle());


	// Turn camera LED lights on or off
	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();

	if ( grabberIn )
		pGrabber->ToggleGrabber(Grabber::kGrabberIn);
	else
		pGrabber->ToggleGrabber(Grabber::kGrabberOut);

	pElevator->SetTarget(elevatorTarget);
	pElevator->CheckLimits();

#ifdef CONSOLE
	// Display SmartDashboard Values
   
#endif
	return;
}

//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetDriverStationInput()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// - Ball Launcher Motor On (SPSD fitch)
// May also need to include reading additional switches depending on where
// functions are included on the robot or driver station.
// - Grabber High/Low speed (SPST switch)
// Include also:
// - Reading joystick functions (left and right)
// - Reading arm rotation potentiometer (analog input)
// Does not include:
// - Tank drive input from the joysticks.  This is coded directly in the
//   SetDriveMotors() method.
//------------------------------------------------------------------------------
void RecycleRushRobot::GetDriverStationInput()
{

	// Obtain the position of switches on the driver station
	// Camera Switches
    lightsOn  				 = pCameraLightSwitch->Get();
    grabberIn				 = pGrabberSwitch->Get();

    if (pElevatorGroundSwitch->Get()){
    	elevatorTarget = Elevator::kGROUNDPOS;
    }
	else if (pElevatorPostition1Switch->Get()){
		elevatorTarget = Elevator::kBOX1;
	}
	else if (pElevatorPostition2Switch->Get()){
		elevatorTarget = Elevator::kBOX2;
	}
	else if (pElevatorPostition3Switch->Get()){
		elevatorTarget = Elevator::kBOX3;
	}
	else if (pElevatorPostition4Switch->Get()){
		elevatorTarget = Elevator::kBOX4;
	}
	else if (pElevatorPostition5Switch->Get()){
		elevatorTarget = Elevator::kBOX5;
	}

    ShowDSValues();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::ShowDSValues()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowDSValues()
{
// Show the values for driver station inputs

    // Camera Light Switch Value
	SmartDashboard::PutBoolean("Camera Lights",lightsOn);
	SmartDashboard::PutBoolean("Grabber In",grabberIn);
	SmartDashboard::PutNumber("Joystick X",pDriveStick->GetX());
	SmartDashboard::PutNumber("Joystick Y",pDriveStick->GetY());
	SmartDashboard::PutNumber("Joystick Twist",pDriveStick->GetTwist());


	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::ShowRobotValues()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowRobotValues()
{
// Show the values from the robot components

	// Show Time
	SmartDashboard::PutNumber("Elapsed Seconds", elapsedSec);
	/*SmartDashboard::PutNumber("Front Left Motor Speed", pDriveTrain->m_frontLeftMotor->Get());
	SmartDashboard::PutNumber("Front Right Motor Speed", pDriveTrain->m_frontRightMotor->Get());
	SmartDashboard::PutNumber("Rear Left Motor Speed", pDriveTrain->m_rearLeftMotor->Get());
	SmartDashboard::PutNumber("Rear Right Motor Speed", pDriveTrain->m_rearRightMotor->Get());*/

}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetRobotSensorInput()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Looks at data in the packet and obtains input coming from the robot to be
// used in both Autonomous and Teleoperated Modes.
// - Distance traveled by right wheels
// - Distance traveled by left wheels
//------------------------------------------------------------------------------
void RecycleRushRobot::GetRobotSensorInput()
{
	// Get distanced traveled by encoder

	ShowRobotValues();
	
	return;
}

//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetAutoModeSwitches()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::GetAutoModeSwitches()
{
	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetTime()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Shifts gear on Grabber to match input from driver station.  Gear shifts
// are via a pneumatic cylinder controlled by a solenoid.
//------------------------------------------------------------------------------
int RecycleRushRobot::GetTime()
{
	int currentTime = (int)GetClock();

	return currentTime;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::RunAutonomousMode()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::RunAutonomousMode()
{

	return;
}

//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::ShowAMStatus()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Shifts gear on Grabber to match input from driver station.  Gear shift
// are via a pneumatic cylinder controlled by a solenoid.
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowAMStatus()
{
	
	return;
}
