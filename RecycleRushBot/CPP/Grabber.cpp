#include "../H/Grabber.h"

// METHOD:  Grabber::Grabber()
// Type:	Constructor
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Grabber::Grabber(uint grabberOpenCh, uint grabberCloseCh)
{

	pGrabberSolenoid = new DoubleSolenoid(grabberOpenCh, grabberCloseCh);

	OpenGrabber();
}
//------------------------------------------------------------------------------
// METHOD:  Grabber::~Grabber()
// Type:	Destructor
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Grabber::~Grabber()
{
	// Does nothing
}
//------------------------------------------------------------------------------
// METHOD:  Grabber::OpenGrabber()
// Type:	Method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void  Grabber::OpenGrabber()
{
	pGrabberSolenoid->Set(DoubleSolenoid::kForward);

	grabberPosition = kGrabberOpen;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Grabber::CloseGrabber()
// Type:	Method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void  Grabber::CloseGrabber()
{
	pGrabberSolenoid->Set(DoubleSolenoid::kReverse);

	grabberPosition = kGrabberClosed;

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Grabber::GetPosition()
// Type:	GET
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool  Grabber::GetPosition() const
{
	return grabberPosition;
}
