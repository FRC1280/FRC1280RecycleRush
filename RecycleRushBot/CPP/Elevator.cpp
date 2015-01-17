#include "../H/Elevator.h"

Elevator::Elevator(uint winchCh, uint potCh, uint limitTopCh, uint limitBottomCh)
{
	pWinch       = new Talon(winchCh);
	pPot         = new AnalogPotentiometer(potCh, POT_TURNS*360, 0);
	pLimitTop    = new DigitalInput(limitTopCh);
	pLimitBottom = new DigitalInput(limitBottomCh);
	pPid         = new PIDController(KP, KI, KD, pPot, pWinch);

	//Initialize PID controller
	offset = kGROUND;
	pPid->SetInputRange(0,POT_TURNS*360);
	pPid->SetOutputRange(-.25,.25);
	pPid->SetTolerance(5);
}

Elevator::~Elevator()
{
}

void Elevator::SetOffset(offsets newOffset)
{
	if (autoMode)
	{
		SetTarget(pPid->GetSetpoint()-offset+newOffset);

		offset=newOffset;
	}
}

void Elevator::SetTarget(float target)
{
	pPid->SetSetpoint(target+offset*(!autoMode));
}

void Elevator::CheckLimits()
{
	if (pPid->IsEnabled())
	{
		if (pLimitTop->Get())
		{
			pPid->Disable();
		}
		else if (pLimitBottom->Get())
		{
			pPid->Disable();
		}
	}
	else
	{
		if (pLimitTop->Get())
		{
			if (pPid->GetSetpoint()<topThreshold)
			{
				pPid->Enable();
			}
		}
		else if (pLimitBottom->Get())
		{
			if (pPid->GetSetpoint()<bottomThreshold)
			{
				pPid->Enable();
			}
		}
	}
}
