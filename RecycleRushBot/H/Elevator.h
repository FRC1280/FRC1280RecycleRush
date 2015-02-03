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

		void   SetInputPotRange(double minPotValue, double maxPotValue);  // DONE
		bool   MoveElevator(double inputPotReading);                      // DONE
		bool   MoveElevator(uint inputTarget, uint inputOffset);
		float  GetMotorSpeed() const;                                     // DONE
		double GetCurrentPosition() const;                                // DONE
		double GetPositionTarget() const;                                 // DONE
		double GetPositionTargetInput() const;                            // DONE
		bool   GetUpperLimitSwitch() const;                               // DONE
		bool   GetLowerLimitSwitch() const;                               // DONE

	private:
		const float   MOTOR_SPEED_UP            =  0.10;
		const float   MOTOR_SPEED_DOWN          = -0.10;
		const float   ALL_STOP                  =  0.0;

		const double  DEFAULT_INPUT_UPPER_LIMIT =  5.0;
		const double  DEFAULT_INPUT_LOWER_LIMIT =  0.0;
		const double  ELEV_POS_UPPER_LIMIT      =  5.0;
		const double  ELEV_POS_LOWER_LIMIT      =  0.0;
		const double  TARGET_TOLERANCE          =  0.02;

		const double  POSITION1_BASE_TARGET     =  0.0;
		const double  POSITION2_BASE_TARGET     =  1.0;
		const double  POSITION3_BASE_TARGET     =  2.0;
		const double  POSITION4_BASE_TARGET     =  3.0;
		const double  POSITION5_BASE_TARGET     =  4.0;
		const double  POSITION6_BASE_TARGET     =  5.0;
		const double  OFFSET_GROUND             =  0.0;
		const double  OFFSET_BURM               =  1.0;
		const double  OFFSET_DIVIDER            =  2.0;

		const bool    PID_CONTROLLER_ON         =  true;
		const bool    PID_CONTROLLER_OFF        =  false;

		const double  POT_FULL_RANGE            =  5.0;
		const double  POT_OFFSET                =  0.0;

		const float kP=27.9237; //PID controller constants
		const float kI=0;
		const float kD=0; //these will need to be set to the right values

		void   CalcTargetRatioConstant();                  // DONE
		double CalcTargetPotValue(double inputPotValue);   // DONE
		double CalcBaseTarget(uint basePosition);
		double CalcOffsetTarget(uint offsetPosition);
		bool   GoToPotTargetNoPID(double potTarget);       // DONE
		bool   GoToPotTargetPID(double potTarget);         // DONE

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
		bool                usePIDController = PID_CONTROLLER_OFF;
};

#endif
