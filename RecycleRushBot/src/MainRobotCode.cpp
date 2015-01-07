//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2015 Recycle Rush ROBOT CODE
//------------------------------------------------------------------------------
#include "WPILib.h" // Instruction to preprocessor to include the WPI Library
                      // header file
#include <cmath>

#include "../H/CameraLights.h"
#include "../H/TankDrive.h"
#include "../H/Transmission.h"

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
		static const uint LEFT_JS_PORT          =  1;
		static const uint RIGHT_JS_PORT         =  0;
		static const uint CCI_PORT              =  3;  // eStop Robots CCI Inputs

		// Driver Station CCI Channels (Uses joystick button references)
		static const uint LED_LIGHTS_CH            =   1;

		// Joystick Buttons
		static const uint TRAN_LOW_SPEED_BT        =  2;
		static const uint TRAN_HIGH_SPEED_BT       =  3;
		static const uint TRAN_GRANNY_BT		     =  8;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------

		// cRIO Digital Sidecar Channels - GPIO & Spare Power Outputs
		// Robot inputs
		// - Air compressor pressure switch (indicates when pressure is
		//   too high)
		static const uint PRESSURE_SW_CH         =  1;
		// Autonomous Mode switches

		// Wheel Encoders
		static const uint LEFT_ENCODER_CHA	   =  7;
		static const uint LEFT_ENCODER_CHB	   =  8;
		static const uint RIGHT_ENCODER_CHA	   =  9;
		static const uint RIGHT_ENCODER_CHB	   = 10;

		// cRIO Analog Breakout Input - Banner Sensors
		// Robot inputs
		// Banner Sensors for elevator, loader and shooter positions

		// cRIO Digital Sidecar Channels - Robot PWM Controls (Outputs)
		// Motors with Victor & Talon speed controllers
		// PWM = Pulsed width modulation
		static const uint RIGHT_REAR_MOTOR_CH	   = 1;
		static const uint RIGHT_FRONT_MOTOR_CH   = 2;
		static const uint LEFT_REAR_MOTOR_CH	   = 3;
		static const uint LEFT_FRONT_MOTOR_CH	   = 4;

		// cRIO Digital Sidecar Channels - Robot Relay Controls (Outputs)
		static const uint COMPRESSOR_CH         =  1;
		static const uint CAMERA_LIGHTS_CH      =  2;

		// cRIO Solenoid Breakout Channels
		static const uint TRANSMISSION		  =  2;

		//----------------------------------------------------------------------
		// CONSTANTS USED TO DETERMINE STATUS OF DRIVER STATION AND ROBOT INPUTS
		// AND TO INSTRUCT ROBOT
		// Private static constants used in multiple methods
		//----------------------------------------------------------------------
		// CONSTANTS USED IN DECLARING OBJECTS
		//----------------------------------------------------------------------
		// Encoder Values
		static const bool   ENCODER_NORMAL_DIR   = false;
		static const bool   ENCODER_REVERSE_DIR  = true;
		constexpr static const double WHEEL_DIST_PER_PULSE = 0.052;
		// Transmission constants
		static const int    GRANNY_GEAR          = 0;

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

		// Joysticks
		Joystick		 *pRightStick;			// joystick 1 (right tank stick)
		Joystick		 *pLeftStick;	        // joystick 2 (left tank stick)
		JoystickButton   *pHighSpeedRightBt;    // Joystick buttons
		JoystickButton   *pHighSpeedLeftBt;
		JoystickButton   *pLowSpeedRightBt;
		JoystickButton   *pLowSpeedLeftBt;
		JoystickButton   *pGrannyRightBt;
		JoystickButton   *pGrannyLeftBt;

		// eStop Robotics Custom Control Interface (CCI)
		Joystick         *pCCI;                 // CCI
		JoystickButton	 *pCameraLightSwitch;   // CCI Digital Inputs

		//----------------------------------------------------------------------
		// ROBOT INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------
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
		TankDrive		*pDriveMotors;

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
		// Disk Shooter Switches


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
		// Transmission Speed / Shifting
		//----------------------------------------------------------------------
		int    tranSpeed;            // Transmission speed

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
	pRightStick		     = new Joystick(RIGHT_JS_PORT);
	pLeftStick			 = new Joystick(LEFT_JS_PORT);
	pCCI                 = new Joystick(CCI_PORT); // CCI uses joystick object

    // Joystick Buttons
	pHighSpeedRightBt    = new JoystickButton(pRightStick,TRAN_HIGH_SPEED_BT);
	pHighSpeedLeftBt     = new JoystickButton(pLeftStick,TRAN_HIGH_SPEED_BT);
	pLowSpeedRightBt     = new JoystickButton(pRightStick,TRAN_LOW_SPEED_BT);
	pLowSpeedLeftBt      = new JoystickButton(pLeftStick,TRAN_LOW_SPEED_BT);
	pGrannyRightBt       = new JoystickButton(pRightStick,TRAN_GRANNY_BT);
	pGrannyLeftBt        = new JoystickButton(pLeftStick,TRAN_GRANNY_BT);

	// CCI Switches
	pCameraLightSwitch   = new JoystickButton(pCCI,LED_LIGHTS_CH);

	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	
	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	// Spike Relays (Relay Connections)
	// lights
	pCameraLights		 = new CameraLights(CAMERA_LIGHTS_CH);

	// Drive Train
	pDriveMotors		 = new TankDrive(RIGHT_REAR_MOTOR_CH,RIGHT_FRONT_MOTOR_CH,
										 LEFT_REAR_MOTOR_CH,LEFT_FRONT_MOTOR_CH,
										 LEFT_ENCODER_CHA, LEFT_ENCODER_CHB,
										 RIGHT_ENCODER_CHA,RIGHT_ENCODER_CHB);

	
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
// DKE Review these to ensure they are in the right method.  Some
//     seem like they could be moved to AutonomousInit() or
//     TeleopInit().
// - Initializes the SmartDashboard
// - Turns air compressor on
// - Shifts robot transmission into low gear
// - Sets type of input used to determine elevator position
// - Sets target values for three preset elevator positions
// - Sets target values for three preset disk loader positions
// - Initializes robot status tracking settings
// - Sets distance per pulse for wheel encoders used for driving
// - Sets formula values to translate aim angle from driver station
//   POT setting to banner sensor setting (DKE - Don't we need to initialize
//   the banner sensor count for the default starting position?)
//------------------------------------------------------------------------------
void RecycleRushRobot::RobotInit()
{
	SmartDashboard::init();

	pDriveMotors->SetDistancePerPulse(WHEEL_DIST_PER_PULSE);

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
// - Shifts the transmission into low gear
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
	

	pDriveMotors->ResetWheelEncoders();
	
	tranSpeed = Transmission::kLowSpeed;

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

	tranSpeed = Transmission::kLowSpeed;
	pDriveMotors->ResetWheelEncoders();

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
// - If a transmission shift was requested by the driver station, shift
//   transmission
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

	SmartDashboard::PutNumber("tranSpeed", tranSpeed);
	SmartDashboard::PutNumber("LeftDriveSpeed", leftDriveSpeed);

	//Set Drive Speed
	pDriveMotors->SetDriveMotors(tranSpeed, leftDriveSpeed, rightDriveSpeed);


	// Turn camera LED lights on or off
	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();

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
// - Transmission High/Low speed (SPST switch)
// Include also:
// - Reading joystick functions (left and right)
// - Reading arm rotation potentiometer (analog input)
// Does not include:
// - Tank drive input from the joysticks.  This is coded directly in the
//   SetDriveMotors() method.
//------------------------------------------------------------------------------
void RecycleRushRobot::GetDriverStationInput()
{
	if ( pHighSpeedLeftBt->Get() &&
	        pHighSpeedRightBt->Get() )
	{
	    	tranSpeed = Transmission::kHighSpeed;
	}
	else
	{
		if ( pLowSpeedLeftBt->Get() &&
				pLowSpeedRightBt->Get() )
		{
			tranSpeed  = Transmission::kLowSpeed;
		}
		else if ( pGrannyLeftBt->Get() ||
				pGrannyRightBt->Get() )
		{
			tranSpeed  = GRANNY_GEAR;
		}
	}

	// Joystick Drive Values
	leftDriveSpeed  = pLeftStick->GetY();
	rightDriveSpeed = pRightStick->GetY();


	// Obtain the position of switches on the driver station
	// Camera Switches
    lightsOn  				 = pCameraLightSwitch->Get();




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

	// Show Joystick Buttons for Shifting
	SmartDashboard::PutBoolean("Right JS Button 3",pHighSpeedRightBt->Get());
	SmartDashboard::PutBoolean("Left JS Button 3",pHighSpeedLeftBt->Get());
	SmartDashboard::PutBoolean("Left JS Button 2",pLowSpeedLeftBt->Get());
	SmartDashboard::PutBoolean("Right JS Button 2",pLowSpeedRightBt->Get());
	SmartDashboard::PutBoolean("Right JS Button 8",pGrannyRightBt->Get());
	SmartDashboard::PutBoolean("Left JS Button 8",pGrannyLeftBt->Get());

    // Camera Light Switch Value

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
// Shifts gear on transmission to match input from driver station.  Gear shifts
// are via a pneumatic cylinder controlled by a solenoid.
//------------------------------------------------------------------------------
int RecycleRushRobot::GetTime()
{
	int  currentTime;

	currentTime = (int)GetClock();

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
// Shifts gear on transmission to match input from driver station.  Gear shifts
// are via a pneumatic cylinder controlled by a solenoid.
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowAMStatus()
{
	
	return;
}
