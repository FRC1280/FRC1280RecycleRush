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
		static const bool kGrabberOpen    = 1;
		static const bool kGrabberClosed  = 0;
		Grabber(uint grabberChannel);
		~Grabber();
		void OpenGrabber();
		void CloseGrabber();
		bool GetPosition();
	private:
		Solenoid *pGrabberSolenoid;
		bool      grabberPosition;
};
#endif
