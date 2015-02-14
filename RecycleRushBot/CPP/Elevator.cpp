#include "../H/Elevator.h"

Elevator::Elevator(uint elevMotorCh, uint elevPotCh, uint upperLimitSwCh, uint lowerLimitSwCh)
{
	pElevatorMotor    = new Talon(elevMotorCh);
	pElevatorPot      = new AnalogPotentiometer(elevPotCh, POT_FULL_RANGE, POT_OFFSET);
	pUpperLimitHit    = new DigitalInput(upperLimitSwCh);
	pLowerLimitHit    = new DigitalInput(lowerLimitSwCh);

	pPIDController    = new PIDController(kP, kI, kD, pElevatorPot, pElevatorMotor);

	// Initialize class variables
	targetRatio      = 0.0;
	targetConstant   = 0.0;
	elevatorTarget   = 0.0;
	targetMotorSpeed = 0.0;

	usePIDController = PID_CONTROLLER_OFF;  // CONFIGURE

	// Configure PID Controller
	pPIDController->SetInputRange(ELEV_POS_LOWER_LIMIT,ELEV_POS_UPPER_LIMIT);
	pPIDController->SetOutputRange(MOTOR_SPEED_DOWN,MOTOR_SPEED_UP);
	pPIDController->SetAbsoluteTolerance(TARGET_TOLERANCE);

	// Set Input to Robot POT ratio and offset using default input values
	SetInputPotRange(DEFAULT_INPUT_LOWER_LIMIT,DEFAULT_INPUT_UPPER_LIMIT);

	// Set default starting position for elevator to current position
	MoveElevator(pElevatorPot->Get());
}

Elevator::~Elevator()
{
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::SetInputPotRange()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Uses as input the upper and lower limits on an input potentiometer to
// calculate the ratio and offset between the input potentiometer and the
// robot potentiometer.  This must be invoked by the calling program to ensure
// proper translation from the input values and the target elevator POT
// reading.
//------------------------------------------------------------------------------
void   Elevator::SetInputPotRange(double minPotValue, double maxPotValue)
{
	inputPOTLowerLimit = minPotValue;
	inputPOTUpperLimit = maxPotValue;

	CalcTargetRatioConstant();

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::MoveElevator()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on an input POT and initiates
// moving elevator to that target.
//------------------------------------------------------------------------------
bool  Elevator::MoveElevator(double inputPotReading)
{
	bool targetFound = false;

	targetInput = inputPotReading;

	elevatorTarget = CalcTargetPotValue(inputPotReading);

	if ( usePIDController == PID_CONTROLLER_ON)
	{
		targetFound = GoToPotTargetPID(elevatorTarget);
	}
	else
	{
		targetFound = GoToPotTargetNoPID(elevatorTarget);
	}

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::MoveElevator()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Calculates a target robot POT value based on input base and offset
// positions.
//------------------------------------------------------------------------------
bool  Elevator::MoveElevator(uint inputTarget, uint inputOffset)
{
	bool   targetFound  = false;
	double baseTarget   = 0;
	double offsetTarget = 0;

	targetInput    = 9999999;   // Set to 9999999 when moving elevator to predefined position

	baseTarget     = CalcBaseTarget(inputTarget);
	offsetTarget   = CalcOffsetTarget(inputOffset);

	elevatorTarget = baseTarget + offsetTarget;

	if ( usePIDController == PID_CONTROLLER_ON)
	{
		targetFound = GoToPotTargetPID(elevatorTarget);
	}
	else
	{
		targetFound = GoToPotTargetNoPID(elevatorTarget);
	}

	return targetFound;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetControlType()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted elevator motor speed.
//------------------------------------------------------------------------------
bool   Elevator::GetControlType() const
{
	return usePIDController;
}

//------------------------------------------------------------------------------
// METHOD:  Elevator::GetTargetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current targeted elevator motor speed.
//------------------------------------------------------------------------------
float  Elevator::GetTargetMotorSpeed() const
{
	return targetMotorSpeed;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetMotorSpeed()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current elevator motor speed.
//------------------------------------------------------------------------------
float  Elevator::GetMotorSpeed() const
{
	return pElevatorMotor->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetCurrentPosition()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the current elevator position potentiometer reading.
//------------------------------------------------------------------------------
double Elevator::GetCurrentPosition() const
{
	return pElevatorPot->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetPositionTarget()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target elevator position potentiometer reading.
//------------------------------------------------------------------------------
double Elevator::GetPositionTarget() const
{
	return elevatorTarget;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetPositionTargetInput()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the target elevator input potentiometer reading.
//------------------------------------------------------------------------------
double Elevator::GetPositionTargetInput() const
{
	return targetInput;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetUpperLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the upper limit switch.
//------------------------------------------------------------------------------
bool   Elevator::GetUpperLimitSwitch() const
{
	return pUpperLimitHit->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GetLowerLimitSwitch()
// Type:	Public accessor method
//------------------------------------------------------------------------------
// Returns the value of the lower limit switch
//------------------------------------------------------------------------------
bool   Elevator::GetLowerLimitSwitch() const
{
	return pLowerLimitHit->Get();
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::CalcTargetRatioOffset()
// Type:	Private method
//------------------------------------------------------------------------------
// Performs the actual calculation to determine the ratio and offset used to
// convert an input potentiometer reading into a target elevator potentiometer
// reading.
//------------------------------------------------------------------------------
void   Elevator::CalcTargetRatioConstant()
{
	targetRatio    = ( ( ELEV_POS_UPPER_LIMIT - ELEV_POS_LOWER_LIMIT )
                   / (inputPOTUpperLimit - inputPOTLowerLimit ) );

	targetConstant = ( ELEV_POS_LOWER_LIMIT - ( inputPOTLowerLimit * targetRatio ) );

	return;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::CalcTargetPotValue()
// Type:	Private method
//------------------------------------------------------------------------------
// Converts an input potentiometer reading into a target elevator potentiometer
// reading.
//------------------------------------------------------------------------------
double Elevator::CalcTargetPotValue(double inputPotValue)
{
	double targetPot;

	targetPot = ( ( inputPotValue * targetRatio ) + targetConstant );

	return targetPot;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::CalcBaseTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the base elevator position target based on an input position.
//------------------------------------------------------------------------------
double Elevator::CalcBaseTarget(uint basePosition)
{
	double targetPot = 0;

    switch ( basePosition )
    {
    	case kPosition1:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION0_INCREMENT;
    		break;

    	case kPosition2:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION1_INCREMENT;
			break;

    	case kPosition3:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION2_INCREMENT;
			break;

    	case kPosition4:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION3_INCREMENT;
			break;

    	case kPosition5:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION4_INCREMENT;
			break;

    	case kPosition6:
			targetPot = ELEV_POS_LOWER_LIMIT + POSITION5_INCREMENT;
			break;

    	default:
    		targetPot = pElevatorPot->Get();
    		break;
    }

	return targetPot;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::CalcOffsetTarget()
// Type:	Private method
//------------------------------------------------------------------------------
// Determines the offset to the elevator position target based on an input
// offset.
//------------------------------------------------------------------------------
double Elevator::CalcOffsetTarget(uint offsetPosition)
{
	double offsetPot = 0;

    switch ( offsetPosition )
    {
    	case kGround:
			offsetPot = OFFSET_GROUND;
    		break;

    	case kBurm:
			offsetPot = OFFSET_BURM;
			break;

    	case kDivider:
			offsetPot = OFFSET_DIVIDER;
			break;

    	default:
    		offsetPot = OFFSET_GROUND;
    		break;
    }

	return offsetPot;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GoToPotTargetPID()
// Type:	Private method
//------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------
bool  Elevator::GoToPotTargetPID(double inputPotValue)
{
	bool targetHit = false;

	targetMotorSpeed = 9999999;  // Set to 9999999 when using the PID controller (PID sets the target motor speed)

	pPIDController->SetSetpoint(inputPotValue);
	pPIDController->Enable();

	if ( pPIDController->Get() < pPIDController->GetSetpoint() )  // Moving up
	{
		if ( pUpperLimitHit->Get() )
		{
			pPIDController->Disable();
			targetHit = true;
		}
		else
		{
			targetHit = pPIDController->OnTarget();
		}
	}
	else
	{
		if ( pPIDController->Get() > pPIDController->GetSetpoint() )  // Moving down
			{
				if ( pLowerLimitHit->Get() )
				{
					pPIDController->Disable();
					targetHit = true;
				}
				else
				{
					targetHit = pPIDController->OnTarget();
				}
			}
		else
		{
			targetHit = pPIDController->OnTarget();
		}
	}

	return targetHit;
}
//------------------------------------------------------------------------------
// METHOD:  Elevator::GoToPotTargetNoPID()
// Type:	Private method
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool  Elevator::GoToPotTargetNoPID(double inputPotValue)
{
	bool   potTargetFound  = false;
	double targetLowValue  = inputPotValue - TARGET_TOLERANCE;
	double targetHighValue = inputPotValue + TARGET_TOLERANCE;

	if ( pElevatorPot->Get() >= targetLowValue  &&
		 pElevatorPot->Get() <= targetHighValue )
	{
		pElevatorMotor->Set(ALL_STOP);
		targetMotorSpeed = ALL_STOP;
		potTargetFound = true;
	}
	else
	{
		if ( pElevatorPot->Get() > targetHighValue )  // Elevator moving down
		{
			if ( pLowerLimitHit->Get() )  // If lower limit switch is hit, stop motors
			{
				pElevatorMotor->Set(ALL_STOP);
				targetMotorSpeed = ALL_STOP;
				potTargetFound = true;
			}
			else
			{
				pElevatorMotor->Set(MOTOR_SPEED_DOWN);
				targetMotorSpeed = MOTOR_SPEED_DOWN;
			}
		}
		else
			if ( pElevatorPot->Get() < targetLowValue )  // Elevator moving up
			{
				if ( pUpperLimitHit->Get() )  // If upper limit switch is hit, stop motors
				{
					pElevatorMotor->Set(ALL_STOP);
					targetMotorSpeed = ALL_STOP;
					potTargetFound = true;
				}
				else
				{
					pElevatorMotor->Set(MOTOR_SPEED_UP);
					targetMotorSpeed = MOTOR_SPEED_UP;
				}
			}
	}

	return potTargetFound;
}
