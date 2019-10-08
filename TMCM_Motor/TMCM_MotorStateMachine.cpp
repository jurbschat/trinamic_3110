/*----- PROTECTED REGION ID(TMCM_MotorStateMachine.cpp) ENABLED START -----*/
//=============================================================================
//
// file :        TMCM_MotorStateMachine.cpp
//
// description : State machine file for the TMCM_Motor class
//
// project :     
//
// This file is part of Tango device class.
// 
// Tango is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Tango is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Tango.  If not, see <http://www.gnu.org/licenses/>.
// 
//
//
//=============================================================================
//                This file is generated by POGO
//        (Program Obviously used to Generate tango Object)
//=============================================================================

#include <TMCM_Motor.h>

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::TMCM_MotorStateMachine.cpp

//================================================================
//  States  |  Description
//================================================================
//  ON      |  
//  FAULT   |  
//  ALARM   |  
//  MOVING  |  


namespace TMCM_Motor_ns
{
//=================================================
//		Attributes Allowed Methods
//=================================================

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Position_allowed()
 *	Description : Execution allowed for Position attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Position_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Check access type.
	if ( type!=Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for WRITE 
		if (get_state()==Tango::FAULT ||
			get_state()==Tango::ALARM ||
			get_state()==Tango::MOVING)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::PositionStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::PositionStateAllowed_WRITE
			return false;
		}
		return true;
	}
	else

	//	Check access type.
	if ( type==Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for READ 
		if (get_state()==Tango::FAULT)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::PositionStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::PositionStateAllowed_READ
			return false;
		}
		return true;
	}
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Velocity_allowed()
 *	Description : Execution allowed for Velocity attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Velocity_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Check access type.
	if ( type!=Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for WRITE 
		if (get_state()==Tango::FAULT ||
			get_state()==Tango::ALARM ||
			get_state()==Tango::MOVING)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::VelocityStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::VelocityStateAllowed_WRITE
			return false;
		}
		return true;
	}
	else

	//	Check access type.
	if ( type==Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for READ 
		if (get_state()==Tango::FAULT)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::VelocityStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::VelocityStateAllowed_READ
			return false;
		}
		return true;
	}
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Acceleration_allowed()
 *	Description : Execution allowed for Acceleration attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Acceleration_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Check access type.
	if ( type!=Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for WRITE 
		if (get_state()==Tango::FAULT ||
			get_state()==Tango::ALARM ||
			get_state()==Tango::MOVING)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::AccelerationStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::AccelerationStateAllowed_WRITE
			return false;
		}
		return true;
	}
	else

	//	Check access type.
	if ( type==Tango::READ_REQ )
	{
		//	Compare device state with not allowed states for READ 
		if (get_state()==Tango::FAULT)
		{
		/*----- PROTECTED REGION ID(TMCM_Motor::AccelerationStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::AccelerationStateAllowed_READ
			return false;
		}
		return true;
	}
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_RunCurrent_allowed()
 *	Description : Execution allowed for RunCurrent attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_RunCurrent_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for RunCurrent attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::RunCurrentStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::RunCurrentStateAllowed_WRITE

	//	Not any excluded states for RunCurrent attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::RunCurrentStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::RunCurrentStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_HoldCurrent_allowed()
 *	Description : Execution allowed for HoldCurrent attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_HoldCurrent_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for HoldCurrent attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::HoldCurrentStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::HoldCurrentStateAllowed_WRITE

	//	Not any excluded states for HoldCurrent attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::HoldCurrentStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::HoldCurrentStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_InvertDirection_allowed()
 *	Description : Execution allowed for InvertDirection attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_InvertDirection_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for InvertDirection attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::InvertDirectionStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::InvertDirectionStateAllowed_WRITE

	//	Not any excluded states for InvertDirection attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::InvertDirectionStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::InvertDirectionStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_SoftLimitEnable_allowed()
 *	Description : Execution allowed for SoftLimitEnable attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_SoftLimitEnable_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for SoftLimitEnable attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftLimitEnableStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftLimitEnableStateAllowed_WRITE

	//	Not any excluded states for SoftLimitEnable attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftLimitEnableStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftLimitEnableStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_SoftCwLimit_allowed()
 *	Description : Execution allowed for SoftCwLimit attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_SoftCwLimit_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for SoftCwLimit attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCwLimitStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCwLimitStateAllowed_WRITE

	//	Not any excluded states for SoftCwLimit attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCwLimitStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCwLimitStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_SoftCcwLimit_allowed()
 *	Description : Execution allowed for SoftCcwLimit attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_SoftCcwLimit_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for SoftCcwLimit attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCcwLimitStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCcwLimitStateAllowed_WRITE

	//	Not any excluded states for SoftCcwLimit attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCcwLimitStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCcwLimitStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_SoftCwLimitFault_allowed()
 *	Description : Execution allowed for SoftCwLimitFault attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_SoftCwLimitFault_allowed(TANGO_UNUSED(Tango::AttReqType type))
{

	//	Not any excluded states for SoftCwLimitFault attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCwLimitFaultStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCwLimitFaultStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_SoftCcwLimitFault_allowed()
 *	Description : Execution allowed for SoftCcwLimitFault attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_SoftCcwLimitFault_allowed(TANGO_UNUSED(Tango::AttReqType type))
{

	//	Not any excluded states for SoftCcwLimitFault attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::SoftCcwLimitFaultStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::SoftCcwLimitFaultStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_HomeOffset_allowed()
 *	Description : Execution allowed for HomeOffset attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_HomeOffset_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for HomeOffset attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::HomeOffsetStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::HomeOffsetStateAllowed_WRITE

	//	Not any excluded states for HomeOffset attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::HomeOffsetStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::HomeOffsetStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_CwLimitFault_allowed()
 *	Description : Execution allowed for CwLimitFault attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_CwLimitFault_allowed(TANGO_UNUSED(Tango::AttReqType type))
{

	//	Not any excluded states for CwLimitFault attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::CwLimitFaultStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::CwLimitFaultStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_CcwLimitFault_allowed()
 *	Description : Execution allowed for CcwLimitFault attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_CcwLimitFault_allowed(TANGO_UNUSED(Tango::AttReqType type))
{

	//	Not any excluded states for CcwLimitFault attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::CcwLimitFaultStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::CcwLimitFaultStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Microsteps_allowed()
 *	Description : Execution allowed for Microsteps attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Microsteps_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for Microsteps attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::MicrostepsStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::MicrostepsStateAllowed_WRITE

	//	Not any excluded states for Microsteps attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::MicrostepsStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::MicrostepsStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_RampDivisor_allowed()
 *	Description : Execution allowed for RampDivisor attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_RampDivisor_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for RampDivisor attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::RampDivisorStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::RampDivisorStateAllowed_WRITE

	//	Not any excluded states for RampDivisor attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::RampDivisorStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::RampDivisorStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_PulseDivisor_allowed()
 *	Description : Execution allowed for PulseDivisor attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_PulseDivisor_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for PulseDivisor attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::PulseDivisorStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::PulseDivisorStateAllowed_WRITE

	//	Not any excluded states for PulseDivisor attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::PulseDivisorStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::PulseDivisorStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_StepInterpolation_allowed()
 *	Description : Execution allowed for StepInterpolation attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_StepInterpolation_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for StepInterpolation attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::StepInterpolationStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::StepInterpolationStateAllowed_WRITE

	//	Not any excluded states for StepInterpolation attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::StepInterpolationStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::StepInterpolationStateAllowed_READ
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_FreeWheeling_allowed()
 *	Description : Execution allowed for FreeWheeling attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_FreeWheeling_allowed(TANGO_UNUSED(Tango::AttReqType type))
{
	//	Not any excluded states for FreeWheeling attribute in Write access.
	/*----- PROTECTED REGION ID(TMCM_Motor::FreeWheelingStateAllowed_WRITE) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::FreeWheelingStateAllowed_WRITE

	//	Not any excluded states for FreeWheeling attribute in read access.
	/*----- PROTECTED REGION ID(TMCM_Motor::FreeWheelingStateAllowed_READ) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::FreeWheelingStateAllowed_READ
	return true;
}


//=================================================
//		Commands Allowed Methods
//=================================================

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Stop_allowed()
 *	Description : Execution allowed for Stop attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Stop_allowed(TANGO_UNUSED(const CORBA::Any &any))
{
	//	Compare device state with not allowed states.
	if (get_state()==Tango::FAULT)
	{
	/*----- PROTECTED REGION ID(TMCM_Motor::StopStateAllowed) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::StopStateAllowed
		return false;
	}
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_ClearAlarm_allowed()
 *	Description : Execution allowed for ClearAlarm attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_ClearAlarm_allowed(TANGO_UNUSED(const CORBA::Any &any))
{
	//	Compare device state with not allowed states.
	if (get_state()==Tango::FAULT)
	{
	/*----- PROTECTED REGION ID(TMCM_Motor::ClearAlarmStateAllowed) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::ClearAlarmStateAllowed
		return false;
	}
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Calibrate_allowed()
 *	Description : Execution allowed for Calibrate attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Calibrate_allowed(TANGO_UNUSED(const CORBA::Any &any))
{
	//	Not any excluded states for Calibrate command.
	/*----- PROTECTED REGION ID(TMCM_Motor::CalibrateStateAllowed) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::CalibrateStateAllowed
	return true;
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Motor::is_Home_allowed()
 *	Description : Execution allowed for Home attribute
 */
//--------------------------------------------------------
bool TMCM_Motor::is_Home_allowed(TANGO_UNUSED(const CORBA::Any &any))
{
	//	Not any excluded states for Home command.
	/*----- PROTECTED REGION ID(TMCM_Motor::HomeStateAllowed) ENABLED START -----*/
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::HomeStateAllowed
	return true;
}


/*----- PROTECTED REGION ID(TMCM_Motor::TMCM_MotorStateAllowed.AdditionalMethods) ENABLED START -----*/

//	Additional Methods

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::TMCM_MotorStateAllowed.AdditionalMethods

}	//	End of namespace
