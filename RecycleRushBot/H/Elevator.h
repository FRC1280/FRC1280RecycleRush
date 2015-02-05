#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Elevator CLASS
//------------------------------------------------------------------------------
// Positions the elevator based on a target potentiometer reading.  Class
// has two primary input methods:
// 1) Driver station potentiometer reading
// 2) Driver station switch values for base position plus an off-set
//------------------------------------------------------------------------------
class Elevator
{
	public:

//		enum elevatorMode {kManual, kAutomatic};  // Probably don't need these
		enum  offset {kGround, kBurm, kDivider};
		enum  target {kPosition1, kPosition2, kPosition3, kPosition4, kPosition5, kPosition6};

		Elevator(uint elevMotorCh, uint elevPotCh, uint upperLimitSwCh, uint lowerLimitSwCh);
		~Elevator();

		void   SetInputPotRange(double minPotValue, double maxPotValue);
		bool   MoveElevator(double inputPotReading);
		bool   MoveElevator(uint inputTarget, uint inputOffset);
		float  GetMotorSpeed() const;
		double GetCurrentPosition() const;
		double GetPositionTarget() const;
		double GetPositionTargetInput() const;
		bool   GetUpperLimitSwitch() const;
		bool   GetLowerLimitSwitch() const;

	private:
		const float   MOTOR_SPEED_UP            =  0.10;   // CONFIGURE
		const float   MOTOR_SPEED_DOWN          = -0.10;   // CONFIGURE
		const float   ALL_STOP                  =  0.0;

		const double  DEFAULT_INPUT_UPPER_LIMIT =  5.0;    // CONFIGURE
		const double  DEFAULT_INPUT_LOWER_LIMIT =  0.0;    // CONFIGURE
		const double  ELEV_POS_UPPER_LIMIT      = 50.0;    // CONFIGURE
		const double  ELEV_POS_LOWER_LIMIT      = 10.0;    // CONFIGURE
		const double  TARGET_TOLERANCE          =  1.0;    // CONFIGURE

		const double  POSITION1_BASE_TARGET     = 10.0;    // CONFIGURE
		const double  POSITION2_BASE_TARGET     = 16.0;    // CONFIGURE
		const double  POSITION3_BASE_TARGET     = 22.0;    // CONFIGURE
		const double  POSITION4_BASE_TARGET     = 28.0;    // CONFIGURE
		const double  POSITION5_BASE_TARGET     = 34.0;    // CONFIGURE
		const double  POSITION6_BASE_TARGET     = 40.0;    // CONFIGURE
		const double  OFFSET_GROUND             =  0.0;
		const double  OFFSET_BURM               =  2.0;    // CONFIGURE
		const double  OFFSET_DIVIDER            =  4.0;    // CONFIGURE

		const bool    PID_CONTROLLER_ON         =  true;
		const bool    PID_CONTROLLER_OFF        =  false;

		const double  POT_FULL_RANGE            = 75.0;    // CONFIGURE
		const double  POT_OFFSET                =  0.0;    // CONFIGURE

		const float kP=27.9237; //PID controller constants
		const float kI=0;
		const float kD=0; //these will need to be set to the right values

		void   CalcTargetRatioConstant();
		double CalcTargetPotValue(double inputPotValue);
		double CalcBaseTarget(uint basePosition);
		double CalcOffsetTarget(uint offsetPosition);
		bool   GoToPotTargetNoPID(double potTarget);
		bool   GoToPotTargetPID(double potTarget);

		Talon               *pElevatorMotor;
		AnalogPotentiometer *pElevatorPot;
		DigitalInput        *pUpperLimitHit;
		DigitalInput        *pLowerLimitHit;
		PIDController       *pPIDController;

		double              inputPOTLowerLimit;
		double              inputPOTUpperLimit;
		double              targetRatio;
		double              targetConstant;
		double              targetInput;
		double              elevatorTarget;
		bool                usePIDController = PID_CONTROLLER_OFF;  // CONFIGURE
};

#endif
