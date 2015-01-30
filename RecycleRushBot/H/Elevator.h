#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Elevator CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Elevator
{
	public:

		enum elevatorMode {kManual, kAutomatic};
		enum offsets {kGROUND=0, kBURM=4, kDIVIDER=6};
		enum targets {kGROUNDPOS=2, kBOX1=40, kBOX2=50, kBOX3=40, kBOX4=50, kBOX5=50}; //these need to be set to the right values

		Elevator(uint winchCh, uint potCh, uint limitTopCh, uint limitBottomCh);
		~Elevator();

		void SetMode(bool mode);
		void SetOffset(offsets newOffset);
		void SetTarget(float target);
		void CheckLimits();

	private:

		float offset;
		bool  autoMode; //1 for automatic mode, 0 for manual

		constexpr static const float KP=27.9237; //PID controller constants
		constexpr static const float KI=0;
		constexpr static const float KD=0; //these will need to be set to the right values

		constexpr static const float POT_TURNS=10.0; //the total number of turns

		constexpr static const float bottomThreshold = 10;
		constexpr static const float topThreshold = POT_TURNS*360-bottomThreshold;


		Talon               *pWinch;
		AnalogPotentiometer *pPot;
		DigitalInput        *pLimitTop;
		DigitalInput        *pLimitBottom;
		PIDController       *pPid;
};

#endif
