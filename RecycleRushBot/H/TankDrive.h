#ifndef TANKDRIVE_H
#define TANKDRIVE_H

#include "WPILib.h"
#include "Transmission.h"

//------------------------------------------------------------------------------
// DEFINE TankDrive CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TankDrive
{
	public:
		static const int kLeftWheels  = 1;
		static const int kRightWheels = 2;
		TankDrive(uint rRearMotorCh, uint rFrontMotorCh, uint lRearMotorCh
		                             , uint lFrontMotorCh, uint lEncoderChA
		                             , uint lEncoderChB, uint rEncoderChA
		                             , uint rEncoderChB);
		TankDrive(uint lEncoderChA , uint lEncoderChB, uint rEncoderChA
		                             , uint rEncoderChB);
		~TankDrive();
		void    ResetWheelEncoders();
		void    SetDriveMotors(int tranSpeed, float leftJSThrottle
                                            , float rightJSThrottle);
		void    StopMotors();
		float   GetMotorSpeed(int wheelSide);
		double  GetDistance(int wheelSide);
		void    SetDistancePerPulse(float WHEEL_DIST_PER_PULSE);
		int   GetLeftEncoderPulses();
		int   GetRightEncoderPulses();
		int     GetTranSpeed();
		int     GetGearStatus();

		int driveTranSpeed;

	protected:
		Victor   	 *pDriveRightRear;
		Victor   	 *pDriveRightFront;
		Victor   	 *pDriveLeftRear;
		Victor   	 *pDriveLeftFront;
		Encoder 	 *pRightEncoder;
		Encoder 	 *pLeftEncoder;
		Transmission *pTransmission;
};
#endif
