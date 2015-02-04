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
		static const bool kGrabberOpen    = true;
		static const bool kGrabberClosed  = false;

		Grabber(uint grabberOpenCh, uint grabberCloseCh);
		~Grabber();

		void  OpenGrabber();
		void  CloseGrabber();
		bool  GetPosition() const;
	protected:
		DoubleSolenoid *pGrabberSolenoid;

		bool   grabberPosition;
};
#endif
