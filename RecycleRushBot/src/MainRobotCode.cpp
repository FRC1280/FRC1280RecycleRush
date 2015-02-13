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
#include "../H/Vision.h"
#include "../H/IMU.h"
#include "../H/IMUAdvanced.h"

#define CONSOLE
//#define VISION

//------------------------------------------------------------------------------
// DEFINE RecycleRushRobot CLASS
//------------------------------------------------------------------------------
// Derived from IterativeRobot class in WPILib.h.  Inherits attributes and
// functions of the base IterativeRobot class.
// Extends class through:
// - Overrides to virtual methods contained in the IterativeRobot base class
// - Methods specific to the 2015 Team 1280 Recycle Rush robot
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
		void   GetElevatorTarget();
		void   GetElevatorBase();
		void   GetElevatorOffset();
		void   ShowDSValues();
		void   GetRobotSensorInput();
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

		// Joystick Buttons
		static const uint USE_FIELD_ORIENT_BUTTON   =  8;
		static const uint USE_ROBOT_ORIENT_BUTTON   = 12;
		static const uint RESET_COMPASS_1_BUTTON    =  9;
		static const uint RESET_COMPASS_2_BUTTON    = 11;

		// Driver Station CCI Channels (Uses joystick button references)
		static const uint OFFSET_DIVIDER_SW_CH      =  1;
	    static const uint OFFSET_GROUND_SW_CH       =  2;
		static const uint ELEV_AUTO_MAN_SW_CH       =  3;
		static const uint GRABBER_SW_CH			    =  4;
		static const uint ELEVATOR_POSITION1_SW_CH  =  5;
		static const uint ELEVATOR_POSITION2_SW_CH  =  6;
		static const uint ELEVATOR_POSITION3_SW_CH  =  7;
		static const uint ELEVATOR_POSITION4_SW_CH  =  8;
		static const uint ELEVATOR_POSITION5_SW_CH  =  9;
		static const uint ELEVATOR_POSITION6_SW_CH  = 10;
		static const uint CAMERA_LIGHTS_SW_CH       = 11;

		//----------------------------------------------------------------------
		// ROBOT CHANNELS - INPUTS AND OUTPUTS
		//----------------------------------------------------------------------
        // ROBOT INPUTS
		//----------------------------------------------------------------------

		// roboRio GPIO Channels
		static const uint TOP_LIMIT_SW_CH		     =  0;
		static const uint BOTTOM_LIMIT_SW_CH	     =  1;
		static const uint TOP_LIMIT_SW_CH		     =  0;
		static const uint BOTTOM_LIMIT_SW_CH	     =  1;
		static const uint AUTO_MODE_OFF_SW_CH        =  2;
		static const uint AUTO_MODE_GET_PIECES_SW_CH =  3;
		static const uint AUTO_MODE_GET_BIN_SW_CH    =  4;
		static const uint AUTO_MODE_STACK_BINS_SW_CH =  5;

		// roboRio Analog Channels
		static const uint ELEVATOR_POT_CH		   =  0;
		//static const uint DRIVE_GYRO_CH			   =  2;

		// navX MXP Inertial Measurement Unit (IMU) Constants
		static const uint8_t IMU_UPDATE_RATE       = 50;

		//----------------------------------------------------------------------
        // ROBOT OUTPUTS
		//----------------------------------------------------------------------

		// roboRio PWM Channels
		// PWM = Pulsed width modulation
		static const uint LEFT_FRONT_MOTOR_CH	   = 0;
		static const uint LEFT_REAR_MOTOR_CH	   = 1;
		static const uint RIGHT_FRONT_MOTOR_CH	   = 2;
		static const uint RIGHT_REAR_MOTOR_CH      = 3;
		static const uint ELEVATOR_MOTOR_CH		   = 4;

		// roboRio Relay Channels
		static const uint CAMERA_LIGHTS_CH         = 2;

		// roboRio Solenoid Channels
		static const uint GRABBER_OPEN_CH          = 0;
		static const uint GRABBER_CLOSE_CH		   = 1;

		//----------------------------------------------------------------------
		// CONSTANTS USED TO DETERMINE STATUS OF DRIVER STATION AND ROBOT INPUTS
		// AND TO INSTRUCT ROBOT
		// Private static constants used in multiple methods
		//----------------------------------------------------------------------
		// CONSTANTS USED IN DECLARING OBJECTS
		//----------------------------------------------------------------------
        const double DS_POT_UPPER_LIMIT    		   =  1.0;
        const double DS_POT_LOWER_LIMIT     	   = -1.0;
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

		// Joystick Buttons
		JoystickButton   *pUseFieldOrientButton;
		JoystickButton   *pUseRobotOrientButton;
		JoystickButton   *pResetCompass1Button;
		JoystickButton   *pResetCompass2Button;

		// eStop Robotics Custom Control Interface (CCI)
		Joystick         *pCCI;                 // CCI
		JoystickButton   *pElevOffsetGroundSwitch;       // CCI Digital Inputs
		JoystickButton   *pElevOffsetDividerSwitch;
		JoystickButton   *pElevManualSwitch;
		JoystickButton	 *pGrabberSwitch;
		JoystickButton   *pElevatorPosition1Switch;
		JoystickButton   *pElevatorPosition2Switch;
		JoystickButton   *pElevatorPosition3Switch;
		JoystickButton   *pElevatorPosition4Switch;
		JoystickButton   *pElevatorPosition5Switch;
		JoystickButton   *pElevatorPosition6Switch;
		JoystickButton 	 *pCameraLightSwitch;

		//----------------------------------------------------------------------
		// ROBOT INPUT & OUTPUT POINTERS
		//----------------------------------------------------------------------

		// navX MXP Inertial Measurement Unit (IMU)
		SerialPort       *pIMUPort;
		IMU              *pIMU;

		//----------------------------------------------------------------------
		// Robot Digital Inputs - GPIO Inputs including Encoders
		//----------------------------------------------------------------------
		// Autonomous Mode Switches
		DigitalInput    *pAutoModeOffSwitch;
		DigitalInput    *pAutoModeGetPiecesSwitch;
		DigitalInput    *pAutoModeGetBinSwitch;
		DigitalInput    *pAutoModeStackBinsSwitch;
	
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
#ifdef VISION
		Vision			*pVision;
#endif
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
//		float  leftDriveSpeed;
//		float  rightDriveSpeed;

		//----------------------------------------------------------------------
		// DRIVER STATION INPUTS - Digital Inputs from eStop Robotics CCI
		//----------------------------------------------------------------------
		// Field Orientation Buttons
		bool   fieldOrientationOn;
		// Camera Switches
		bool   lightsOn;
		// Robot Switches
		bool   grabberOpen;
		double elevatorTarget;
		uint   elevatorBase;
		uint   elevatorOffset;

		//----------------------------------------------------------------------
		// CLASS VARIABLES USED TO TRACK ROBOT STATUS
		//----------------------------------------------------------------------
		// General status tracking
		//----------------------------------------------------------------------
		// Class variables to track look (packet) counts
		uint   loopCount;
        float  loopsPerMinute;

        // Class variables to track time
		int  startSec;
		int  elapsedSec;
		int  oldSec;

		//----------------------------------------------------------------------
		// Camera Image Processing
		//----------------------------------------------------------------------
		

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
	pUseFieldOrientButton            = new JoystickButton(pDriveStick,USE_FIELD_ORIENT_BUTTON);
	pUseRobotOrientButton            = new JoystickButton(pDriveStick,USE_ROBOT_ORIENT_BUTTON);
	pResetCompass1Button             = new JoystickButton(pDriveStick,RESET_COMPASS_1_BUTTON);
	pResetCompass2Button             = new JoystickButton(pDriveStick,RESET_COMPASS_2_BUTTON);

	// CCI Switches
    pElevOffsetGroundSwitch          = new JoystickButton(pCCI,OFFSET_GROUND_SW_CH);
    pElevOffsetDividerSwitch         = new JoystickButton(pCCI,OFFSET_DIVIDER_SW_CH);
	pGrabberSwitch		 			 = new JoystickButton(pCCI,GRABBER_SW_CH);
	pElevManualSwitch	        	 = new JoystickButton(pCCI,ELEV_AUTO_MAN_SW_CH);
	pElevatorPosition1Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION1_SW_CH);
	pElevatorPosition2Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION2_SW_CH);
	pElevatorPosition3Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION3_SW_CH);
	pElevatorPosition4Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION4_SW_CH);
	pElevatorPosition5Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION5_SW_CH);
	pElevatorPosition6Switch		 = new JoystickButton(pCCI,ELEVATOR_POSITION6_SW_CH);
	pCameraLightSwitch   			 = new JoystickButton(pCCI,CAMERA_LIGHTS_SW_CH);

	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	pAutoModeOffSwitch       = new DigitalInput(AUTO_MODE_OFF_SW_CH);
	pAutoModeGetPiecesSwitch = new DigitalInput(AUTO_MODE_GET_PIECES_SW_CH);
	pAutoModeGetBinSwitch    = new DigitalInput(AUTO_MODE_GET_BIN_SW_CH);
	pAutoModeStackBinsSwitch = new DigitalInput(AUTO_MODE_STACK_BINS_SW_CH);
	
	// navX MXP Intertial Measurement Unit (IMU)
	pIMUPort             = new SerialPort(57600,SerialPort::kMXP);
	pIMU                 = new IMU(pIMUPort,IMU_UPDATE_RATE);

	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	// Spike Relays (Relay Connections)
	// lights
	pCameraLights		 = new CameraLights(CAMERA_LIGHTS_CH);

	// Drive Train
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
									      RIGHT_FRONT_MOTOR_CH,RIGHT_REAR_MOTOR_CH);

	pGrabber 			 = new Grabber(GRABBER_OPEN_CH, GRABBER_CLOSE_CH);

	pElevator			 = new Elevator(ELEVATOR_MOTOR_CH, ELEVATOR_POT_CH,TOP_LIMIT_SW_CH,BOTTOM_LIMIT_SW_CH );
	
	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop and time counters
	loopCount      = 0;
	startSec       = 0;
	elapsedSec     = 0;
	loopsPerMinute = 0;
	oldSec         = 0;

	fieldOrientationOn = false;  // CONFIGURE
	lightsOn           = false;  // CONFIGURE
	grabberOpen        = true;   // CONFIGURE
	elevatorTarget     = 0;
	elevatorBase       = 0;
	elevatorOffset     = 0;

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
// - Sets type of input used to determine elevator position
// - Sets target values for three preset elevator positions
// - Initializes robot status tracking settings
//------------------------------------------------------------------------------
void RecycleRushRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif

	pElevator->SetInputPotRange(DS_POT_LOWER_LIMIT, DS_POT_UPPER_LIMIT);

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
	
	// Set Robot Components to Default Starting Positions
	pIMU->ZeroYaw();
	pCameraLights->TurnOn();                   // Configure
	pGrabber->OpenGrabber();                   // Configure

	elevatorTarget = pElevator->GetCurrentPosition();
	elevatorBase   = Elevator::kPosition2;     // Configure
	elevatorOffset = Elevator::kGround;        // Configure

	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

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
#ifdef VISION
	pVision = new Vision;
#endif

	// General loop count & elapsed time initialization
	loopCount      = 0;
	loopsPerMinute = 0;

	elapsedSec = 0;
	startSec   = (int)GetClock();
	oldSec     = startSec;

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
	GetRobotSensorInput();
	ShowAMStatus();

#ifdef CONSOLE
	ShowRobotValues();
#endif

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

#ifdef VISION
	pVision->processImage();
#endif

	// Get inputs from the driver station
	GetDriverStationInput();
	
	// Get robot sensor input
	GetRobotSensorInput();

	//Set Drive Speed and drive mode with or without field orientation
	if ( fieldOrientationOn )
		pDriveTrain->MecanumDrive_Cartesian (pDriveStick->GetX(), pDriveStick->GetY(), pDriveStick->GetTwist(),pIMU->GetYaw());
	else
		pDriveTrain->MecanumDrive_Cartesian (pDriveStick->GetX(), pDriveStick->GetY(), pDriveStick->GetTwist());

	// Turn camera LED lights on or off based on driver station input
	if ( lightsOn )
		pCameraLights->TurnOn();
	else
		pCameraLights->TurnOff();

	// Open or Close Brabber based on driver station input
	if ( grabberOpen )
		pGrabber->OpenGrabber();
	else
		pGrabber->CloseGrabber();

	if ( pElevManualSwitch->Get() )
		pElevator->MoveElevator(elevatorTarget);
	else
		pElevator->MoveElevator(elevatorBase, elevatorOffset);

	return;
}

//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetDriverStationInput()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// -
// May also need to include reading additional switches depending on where
// functions are included on the robot or driver station.
// -
// Include also:
// - Reading joystick function
// - Reading elevator position potentiometer (analog input)
// Does not include:
// - Robot drive input from the joystick.  This is coded directly in the
//   SetDriveMotors() method.
//------------------------------------------------------------------------------
void RecycleRushRobot::GetDriverStationInput()
{
	// Obtain the position of switches on the driver station
    // Field Orientation Joystick Button Values
	if ( pUseFieldOrientButton->Get() )
		fieldOrientationOn = true;
	else
		if ( pUseRobotOrientButton->Get() )
			fieldOrientationOn = false;

	if ( pResetCompass1Button->Get() && pResetCompass2Button->Get() )
		pIMU->ZeroYaw();

	// Camera Switches
    lightsOn  				 = pCameraLightSwitch->Get();

    // Get grabber position
    grabberOpen				 = pGrabberSwitch->Get();

    GetElevatorTarget();

#ifdef CONSOLE
    ShowDSValues();
#endif

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetElevatorTarget()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void  RecycleRushRobot::GetElevatorTarget()
{
	elevatorTarget = pCCI->GetX();

	GetElevatorBase();
	GetElevatorOffset();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetElevatorBase()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::GetElevatorBase()
{
    if ( !pElevatorPosition1Switch->Get() )
       	elevatorBase = Elevator::kPosition1;
    else
    {
    	if ( !pElevatorPosition2Switch->Get() )
    		elevatorBase = Elevator::kPosition2;
    	else
    	{
    		if ( !pElevatorPosition3Switch->Get() )
    			elevatorBase = Elevator::kPosition3;
    		else
    		{
    			if ( !pElevatorPosition4Switch->Get() )
    				elevatorBase = Elevator::kPosition4;
    			else
    			{
    				if ( !pElevatorPosition5Switch->Get() )
    					elevatorBase = Elevator::kPosition5;
    				else
    				{
    					if ( !pElevatorPosition6Switch->Get() )
    						elevatorBase = Elevator::kPosition6;
    				}
    			}
    		}
    	}
	}

	return;
}
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::GetElevatorOffset()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::GetElevatorOffset()
{
    if ( pElevOffsetGroundSwitch->Get() )
       	elevatorOffset = Elevator::kGround;
    else
    {
    	if ( pElevOffsetDividerSwitch->Get() )
    		elevatorOffset = Elevator::kDivider;
    	else
    	{
    		elevatorOffset = Elevator::kBurm;
    	}
    }

	return;
}
#ifdef CONSOLE
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::ShowDSValues()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowDSValues()
{
// Show the values for driver station inputs

	SmartDashboard::PutBoolean("Field Orient Button",pUseFieldOrientButton->Get());
	SmartDashboard::PutBoolean("Robot Orient Button",pUseRobotOrientButton->Get());
//	SmartDashboard::PutBoolean("Reset Compass Button 9",pResetCompass1Button->Get());
//	SmartDashboard::PutBoolean("Reset Compass Button 11",pResetCompass2Button->Get());
	SmartDashboard::PutBoolean("Use Field Orientation",fieldOrientationOn);
	SmartDashboard::PutBoolean("Camera Lights Switch",lightsOn);
	SmartDashboard::PutBoolean("Offset Ground Switch",pElevOffsetGroundSwitch->Get());
	SmartDashboard::PutBoolean("Offset Divider Switch",pElevOffsetDividerSwitch->Get());
	SmartDashboard::PutBoolean("Elevator Manual Switch",pElevManualSwitch->Get());
	SmartDashboard::PutBoolean("Grabber Switch",pGrabberSwitch->Get());
	SmartDashboard::PutBoolean("Elevator Position 1",pElevatorPosition1Switch->Get());
	SmartDashboard::PutBoolean("Elevator Position 2",pElevatorPosition2Switch->Get());
	SmartDashboard::PutBoolean("Elevator Position 3",pElevatorPosition3Switch->Get());
	SmartDashboard::PutBoolean("Elevator Position 4",pElevatorPosition4Switch->Get());
	SmartDashboard::PutBoolean("Elevator Position 5",pElevatorPosition5Switch->Get());
	SmartDashboard::PutBoolean("Elevator Position 6",pElevatorPosition6Switch->Get());
	SmartDashboard::PutNumber("Elev Input POT",pCCI->GetX());

	SmartDashboard::PutNumber("Joystick X",pDriveStick->GetX());
	SmartDashboard::PutNumber("Joystick Y",pDriveStick->GetY());
	SmartDashboard::PutNumber("Joystick Twist",pDriveStick->GetTwist());

	return;
}
#endif
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
#ifdef CONSOLE
	ShowRobotValues();
#endif
	
	return;
}
#ifdef CONSOLE
//------------------------------------------------------------------------------
// METHOD:  RecycleRushRobot::ShowRobotValues()
// Type:	Public accessor for RecycleRushRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void RecycleRushRobot::ShowRobotValues()
{
	SmartDashboard::PutBoolean("AM Off Switch",pAutoModeOffSwitch->Get());
	SmartDashboard::PutBoolean("AM Get Pieces Switch",pAutoModeGetPiecesSwitch->Get());
	SmartDashboard::PutBoolean("AM Get Bin Switch",pAutoModeGetBinSwitch->Get());
	SmartDashboard::PutBoolean("AM Stack Pieces Switch",pAutoModeStackBinsSwitch->Get());
	SmartDashboard::PutBoolean("IMU Connected",pIMU->IsConnected());
	SmartDashboard::PutBoolean("IMU Calibrating",pIMU->IsCalibrating());
	SmartDashboard::PutNumber("IMU Gyro Angle",pIMU->GetCompassHeading());
	SmartDashboard::PutNumber("IMU Yaw",pIMU->GetYaw());
	SmartDashboard::PutNumber("IMU Pitch",pIMU->GetPitch());
	SmartDashboard::PutNumber("IMU Roll",pIMU->GetRoll());
	SmartDashboard::PutBoolean("Camera Lights",pCameraLights->GetCameraStatus());
    SmartDashboard::PutBoolean("Grabber Position",pGrabber->GetPosition());
    SmartDashboard::PutNumber("Elev POT Current Position",pElevator->GetCurrentPosition());
    SmartDashboard::PutNumber("Elev POT Target Position",pElevator->GetPositionTarget());
    SmartDashboard::PutBoolean("Upper Limit Switch",pElevator->GetUpperLimitSwitch());
    SmartDashboard::PutBoolean("Lower Limit Switch",pElevator->GetLowerLimitSwitch());
    SmartDashboard::PutBoolean("Using PID Controller",pElevator->GetControlType());
    SmartDashboard::PutNumber("Elevator Target Motor Speed",pElevator->GetTargetMotorSpeed());
    SmartDashboard::PutNumber("Elevator Motor Speed",pElevator->GetMotorSpeed());

#ifdef VISION
	SmartDashboard::PutBoolean("Camera sees bright", pVision->getIsBright());
#endif

	return;
}
#endif
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
