#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Transmission CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Transmission
{
	public:
		static const int kHighSpeed = 2;
		static const int kLowSpeed  = 1;
		Transmission(uint tranChannel);
		~Transmission();
		void ShiftGear(int gearSpeed);
		int  GetGear();
		void ShowTranValues();
	protected:
		void ShiftHigh();
		void ShiftLow();

		Solenoid *pGear;

		int  gearStatus;
		static const bool ON  = true;
		static const bool OFF = false;
};
#endif
