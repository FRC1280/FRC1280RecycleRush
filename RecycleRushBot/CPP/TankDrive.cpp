#include "../H/TankDrive.h"
#include "../H/Transmission.h"

//------------------------------------------------------------------------------
// METHOD:  TankDrive::TankDrive()
// Type:	Constructor
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TankDrive::TankDrive(uint rRearMotorCh, uint rFrontMotorCh, uint lRearMotorCh
                                        , uint lFrontMotorCh, uint lEncoderChA
                                        , uint lEncoderChB, uint rEncoderChA
                                        , uint rEncoderChB)
{

    static const bool ENCODER_NORMAL_DIR         = true;
//  static const bool ENCODER_REVERSE_DIR        = false;

	static const uint TRANSMISSION_CH		     =  1;



    pDriveRightRear  = new Victor(rRearMotorCh);
    pDriveRightFront = new Victor(rFrontMotorCh);
    pDriveLeftRear   = new Victor(lRearMotorCh);
    pDriveLeftFront  = new Victor(lFrontMotorCh);

    pRightEncoder    = new Encoder(rEncoderChA,rEncoderChB
                                      ,ENCODER_NORMAL_DIR);
    pLeftEncoder     = new Encoder(lEncoderChA,lEncoderChB
                                      ,ENCODER_NORMAL_DIR);
	pTransmission = new Transmission(TRANSMISSION_CH);
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::TankDrive()
// Type:	Constructor
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TankDrive::TankDrive(uint lEncoderChA, uint lEncoderChB, uint rEncoderChA
                                       , uint rEncoderChB)
{
    static const uint RIGHT_REAR_MOTOR_CH      = 1;
    static const uint RIGHT_FRONT_MOTOR_CH     = 2;
    static const uint LEFT_REAR_MOTOR_CH       = 3;
    static const uint LEFT_FRONT_MOTOR_CH      = 4;

	static const uint TRANSMISSION_CH		   = 1;


    static const bool ENCODER_NORMAL_DIR         = true;
//  static const bool ENCODER_REVERSE_DIR        = false;


    pDriveRightRear  = new Victor(RIGHT_REAR_MOTOR_CH);
    pDriveRightFront = new Victor(RIGHT_FRONT_MOTOR_CH);
    pDriveLeftRear   = new Victor(LEFT_REAR_MOTOR_CH);
    pDriveLeftFront  = new Victor(LEFT_FRONT_MOTOR_CH);

    pRightEncoder    = new Encoder(rEncoderChA,rEncoderChB
                                      ,ENCODER_NORMAL_DIR);
    pLeftEncoder     = new Encoder(lEncoderChA,lEncoderChB
                                      ,ENCODER_NORMAL_DIR);
	pTransmission = new Transmission(TRANSMISSION_CH);
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::~TankDrive()
// Type:	Destructor
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TankDrive::~TankDrive()
{
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::ResetWheelEncoders()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Resets the wheel encoders
//------------------------------------------------------------------------------
void TankDrive::ResetWheelEncoders()
{
	pRightEncoder->Reset();

	pLeftEncoder->Reset();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::SetDriveMotors()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Resets the wheel encoders
//------------------------------------------------------------------------------
void TankDrive::SetDriveMotors(int tranSpeed, float leftJSThrottle
		                                    , float rightJSThrottle)
{
    static const float NORMAL_SPEED_ADJ = 1.00;
    static const float GRANNY_SPEED_ADJ = 0.30;

    float leftMotorSpeed;
    float rightMotorSpeed;

    if ( tranSpeed != pTransmission->GetGear())
    {
    	pTransmission->ShiftGear(tranSpeed);
    }

    if ( tranSpeed == Transmission::kHighSpeed  ||
    	 tranSpeed == Transmission::kLowSpeed      )
    {
    	leftMotorSpeed  = NORMAL_SPEED_ADJ * leftJSThrottle;
    	rightMotorSpeed = NORMAL_SPEED_ADJ * rightJSThrottle;
    }
    else
    {
    	leftMotorSpeed  = GRANNY_SPEED_ADJ * leftJSThrottle;
    	rightMotorSpeed = GRANNY_SPEED_ADJ * rightJSThrottle;
    }

    driveTranSpeed = tranSpeed;

    pDriveLeftFront->Set(leftMotorSpeed);
    pDriveLeftRear->Set(leftMotorSpeed);
    pDriveRightFront->Set(rightMotorSpeed);
    pDriveRightRear->Set(rightMotorSpeed);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::StopMotors()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Sets the wheel motor speed to zero to stop the robot
//------------------------------------------------------------------------------
void TankDrive::StopMotors()
{
    static const float STOP_MOTOR = 0.0;

    pDriveLeftFront->Set(STOP_MOTOR);
    pDriveLeftRear->Set(STOP_MOTOR);
    pDriveRightFront->Set(STOP_MOTOR);
    pDriveRightRear->Set(STOP_MOTOR);

	return;
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::GetMotorSpeed()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Gets the speed of the motors on the specified side
//------------------------------------------------------------------------------
float TankDrive::GetMotorSpeed(int wheelSide)
{
	float speed;

	if ( wheelSide == kLeftWheels )
		speed = pDriveLeftFront->Get();
	else
		speed = pDriveRightFront->Get();

	return speed;
}
//------------------------------------------------------------------------------
// METHOD:  TankDrive::GetDistance()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Sets the distance tracked by the wheel encoders
//------------------------------------------------------------------------------
double TankDrive::GetDistance(int wheelSide)
{
	double distance;

	if ( wheelSide == kLeftWheels )
		distance = pLeftEncoder->GetDistance();
	else
		distance = pRightEncoder->GetDistance();

	return distance;
}

//------------------------------------------------------------------------------
// METHOD:  TankDrive::ResetWheelEncoders()
// Type:	Public Accessor for TankDrive
//------------------------------------------------------------------------------
// Resets the wheel encoders
//------------------------------------------------------------------------------
void TankDrive::SetDistancePerPulse(float WHEEL_DIST_PER_PULSE)
{
	pRightEncoder->SetDistancePerPulse(WHEEL_DIST_PER_PULSE);
	pLeftEncoder->SetDistancePerPulse(WHEEL_DIST_PER_PULSE);

	return;
}

int TankDrive::GetGearStatus()
{
	return pTransmission->GetGear();
}

int TankDrive::GetLeftEncoderPulses()
{
	return pLeftEncoder->GetRaw();
}

int TankDrive::GetRightEncoderPulses()
{
	return pRightEncoder->GetRaw();
}

int TankDrive::GetTranSpeed()
{
	return driveTranSpeed;
}
