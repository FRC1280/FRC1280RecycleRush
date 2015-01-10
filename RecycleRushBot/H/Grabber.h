#ifndef GRABBER_H
#define GRABBER_H

#include "WPILib.h"

//------------------------------------------------------------------------------
// DEFINE Grabber CLASS
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Grabber
{
	public:
		static const int kGrabberIn = 1;
		static const int kGrabberOut  = 0;
		Grabber(uint grabberChannel);
		~Grabber();
		void ToggleGrabber(int grabberPosition);
		int  GetPosition();
	protected:
		void MoveIn();
		void MoveOut();

		Solenoid *pGrabberSolenoid;

		int  grabberPos;
		static const bool ON  = true;
		static const bool OFF = false;
};
#endif
