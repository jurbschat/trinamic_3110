/*----- PROTECTED REGION ID(TMCM_Motor.h) ENABLED START -----*/
//=============================================================================
//
// file :        TMCM_Motor.h
//
// description : Include file for the TMCM_Motor class
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


#ifndef TMCM_Motor_H
#define TMCM_Motor_H

#include <tango.h>
#include <fmt/format.h>
#include "../core/core.h"


/*----- PROTECTED REGION END -----*/	//	TMCM_Motor.h

/**
 *  TMCM_Motor class description:
 *    
 */

namespace TMCM_Motor_ns
{
enum _MicrostepsEnum {
	_0,
	_1,
	_4,
	_8,
	_16,
	_32,
	_64,
	_128,
	_256,
} ;
typedef _MicrostepsEnum MicrostepsEnum;

/*----- PROTECTED REGION ID(TMCM_Motor::Additional Class Declarations) ENABLED START -----*/

//	Additional Class Declarations

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::Additional Class Declarations

class TMCM_Motor : public TANGO_BASE_CLASS
{

/*----- PROTECTED REGION ID(TMCM_Motor::Data Members) ENABLED START -----*/

//	Add your own data members
	TMCM::ControllerInterface* ci = nullptr;

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::Data Members

//	Device property data members
public:
	//	moduleId:	module id (0 ... 255)
	Tango::DevLong	moduleId;
	//	motorId:	motor id (0...2)
	Tango::DevLong	motorId;

//	Attribute data members
public:
	Tango::DevDouble	*attr_Position_read;
	Tango::DevDouble	*attr_Velocity_read;
	Tango::DevDouble	*attr_Acceleration_read;
	Tango::DevDouble	*attr_ConversionFactor_read;
	Tango::DevBoolean	*attr_SoftLimitEnable_read;
	Tango::DevDouble	*attr_SoftCwLimit_read;
	Tango::DevDouble	*attr_SoftCcwLimit_read;
	Tango::DevBoolean	*attr_SoftCwLimitFault_read;
	Tango::DevBoolean	*attr_SoftCcwLimitFault_read;
	Tango::DevBoolean	*attr_CwLimitFault_read;
	Tango::DevBoolean	*attr_CcwLimitFault_read;
	Tango::DevDouble	*attr_RunCurrent_read;
	Tango::DevDouble	*attr_HoldCurrent_read;
	MicrostepsEnum	*attr_Microsteps_read;
	Tango::DevULong	*attr_RampDivisor_read;
	Tango::DevULong	*attr_PulseDivisor_read;
	Tango::DevBoolean	*attr_StepInterpolation_read;
	Tango::DevULong	*attr_FreeWheeling_read;
	Tango::DevLong	*attr_HomeOffset_read;
	Tango::DevBoolean	*attr_DisableLeftLimit_read;
	Tango::DevBoolean	*attr_DisableRightLimit_read;

//	Constructors and destructors
public:
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	TMCM_Motor(Tango::DeviceClass *cl,string &s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	TMCM_Motor(Tango::DeviceClass *cl,const char *s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device name
	 *	@param d	Device description.
	 */
	TMCM_Motor(Tango::DeviceClass *cl,const char *s,const char *d);
	/**
	 * The device object destructor.
	 */
	~TMCM_Motor() {delete_device();};


//	Miscellaneous methods
public:
	/*
	 *	will be called at device destruction or at init command.
	 */
	void delete_device();
	/*
	 *	Initialize the device
	 */
	virtual void init_device();
	/*
	 *	Read the device properties from database
	 */
	void get_device_property();
	/*
	 *	Always executed method before execution command method.
	 */
	virtual void always_executed_hook();


//	Attribute methods
public:
	//--------------------------------------------------------
	/*
	 *	Method      : TMCM_Motor::read_attr_hardware()
	 *	Description : Hardware acquisition for attributes.
	 */
	//--------------------------------------------------------
	virtual void read_attr_hardware(vector<long> &attr_list);
	//--------------------------------------------------------
	/*
	 *	Method      : TMCM_Motor::write_attr_hardware()
	 *	Description : Hardware writing for attributes.
	 */
	//--------------------------------------------------------
	virtual void write_attr_hardware(vector<long> &attr_list);

/**
 *	Attribute Position related methods
 *	Description: position in real world units (affected by conversion factor)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_Position(Tango::Attribute &attr);
	virtual void write_Position(Tango::WAttribute &attr);
	virtual bool is_Position_allowed(Tango::AttReqType type);
/**
 *	Attribute Velocity related methods
 *	Description: velocity in real world units/s (affected by conversion factor)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_Velocity(Tango::Attribute &attr);
	virtual void write_Velocity(Tango::WAttribute &attr);
	virtual bool is_Velocity_allowed(Tango::AttReqType type);
/**
 *	Attribute Acceleration related methods
 *	Description: acceleration in real world units/s^2 (affected by conversion factor)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_Acceleration(Tango::Attribute &attr);
	virtual void write_Acceleration(Tango::WAttribute &attr);
	virtual bool is_Acceleration_allowed(Tango::AttReqType type);
/**
 *	Attribute ConversionFactor related methods
 *	Description: sets the conversion between real and internal units for the position/acceleration/velocity and soft limits
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_ConversionFactor(Tango::Attribute &attr);
	virtual void write_ConversionFactor(Tango::WAttribute &attr);
	virtual bool is_ConversionFactor_allowed(Tango::AttReqType type);
/**
 *	Attribute SoftLimitEnable related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_SoftLimitEnable(Tango::Attribute &attr);
	virtual void write_SoftLimitEnable(Tango::WAttribute &attr);
	virtual bool is_SoftLimitEnable_allowed(Tango::AttReqType type);
/**
 *	Attribute SoftCwLimit related methods
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_SoftCwLimit(Tango::Attribute &attr);
	virtual void write_SoftCwLimit(Tango::WAttribute &attr);
	virtual bool is_SoftCwLimit_allowed(Tango::AttReqType type);
/**
 *	Attribute SoftCcwLimit related methods
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_SoftCcwLimit(Tango::Attribute &attr);
	virtual void write_SoftCcwLimit(Tango::WAttribute &attr);
	virtual bool is_SoftCcwLimit_allowed(Tango::AttReqType type);
/**
 *	Attribute SoftCwLimitFault related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_SoftCwLimitFault(Tango::Attribute &attr);
	virtual bool is_SoftCwLimitFault_allowed(Tango::AttReqType type);
/**
 *	Attribute SoftCcwLimitFault related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_SoftCcwLimitFault(Tango::Attribute &attr);
	virtual bool is_SoftCcwLimitFault_allowed(Tango::AttReqType type);
/**
 *	Attribute CwLimitFault related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_CwLimitFault(Tango::Attribute &attr);
	virtual bool is_CwLimitFault_allowed(Tango::AttReqType type);
/**
 *	Attribute CcwLimitFault related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_CcwLimitFault(Tango::Attribute &attr);
	virtual bool is_CcwLimitFault_allowed(Tango::AttReqType type);
/**
 *	Attribute RunCurrent related methods
 *	Description: current when the motor is moving
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_RunCurrent(Tango::Attribute &attr);
	virtual void write_RunCurrent(Tango::WAttribute &attr);
	virtual bool is_RunCurrent_allowed(Tango::AttReqType type);
/**
 *	Attribute HoldCurrent related methods
 *	Description: hold current if no move is in action
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
	virtual void read_HoldCurrent(Tango::Attribute &attr);
	virtual void write_HoldCurrent(Tango::WAttribute &attr);
	virtual bool is_HoldCurrent_allowed(Tango::AttReqType type);
/**
 *	Attribute Microsteps related methods
 *	Description: amount of steps between fullsteps
 *
 *	Data type:	Tango::DevEnum
 *	Attr type:	Scalar
 */
	virtual void read_Microsteps(Tango::Attribute &attr);
	virtual void write_Microsteps(Tango::WAttribute &attr);
	virtual bool is_Microsteps_allowed(Tango::AttReqType type);
/**
 *	Attribute RampDivisor related methods
 *	Description: divisor to calculate the acceelration ramp
 *
 *	Data type:	Tango::DevULong
 *	Attr type:	Scalar
 */
	virtual void read_RampDivisor(Tango::Attribute &attr);
	virtual void write_RampDivisor(Tango::WAttribute &attr);
	virtual bool is_RampDivisor_allowed(Tango::AttReqType type);
/**
 *	Attribute PulseDivisor related methods
 *	Description: divisor to calculate the ustep ferquency
 *
 *	Data type:	Tango::DevULong
 *	Attr type:	Scalar
 */
	virtual void read_PulseDivisor(Tango::Attribute &attr);
	virtual void write_PulseDivisor(Tango::WAttribute &attr);
	virtual bool is_PulseDivisor_allowed(Tango::AttReqType type);
/**
 *	Attribute StepInterpolation related methods
 *	Description: interpolates 16 microsteps to 265
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_StepInterpolation(Tango::Attribute &attr);
	virtual void write_StepInterpolation(Tango::WAttribute &attr);
	virtual bool is_StepInterpolation_allowed(Tango::AttReqType type);
/**
 *	Attribute FreeWheeling related methods
 *	Description: duration untill the motor gets shut down power (0 disables it)
 *
 *	Data type:	Tango::DevULong
 *	Attr type:	Scalar
 */
	virtual void read_FreeWheeling(Tango::Attribute &attr);
	virtual void write_FreeWheeling(Tango::WAttribute &attr);
	virtual bool is_FreeWheeling_allowed(Tango::AttReqType type);
/**
 *	Attribute HomeOffset related methods
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
	virtual void read_HomeOffset(Tango::Attribute &attr);
	virtual void write_HomeOffset(Tango::WAttribute &attr);
	virtual bool is_HomeOffset_allowed(Tango::AttReqType type);
/**
 *	Attribute DisableLeftLimit related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_DisableLeftLimit(Tango::Attribute &attr);
	virtual void write_DisableLeftLimit(Tango::WAttribute &attr);
	virtual bool is_DisableLeftLimit_allowed(Tango::AttReqType type);
/**
 *	Attribute DisableRightLimit related methods
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
	virtual void read_DisableRightLimit(Tango::Attribute &attr);
	virtual void write_DisableRightLimit(Tango::WAttribute &attr);
	virtual bool is_DisableRightLimit_allowed(Tango::AttReqType type);


	//--------------------------------------------------------
	/**
	 *	Method      : TMCM_Motor::add_dynamic_attributes()
	 *	Description : Add dynamic attributes if any.
	 */
	//--------------------------------------------------------
	void add_dynamic_attributes();




//	Command related methods
public:
	/**
	 *	Command State related method
	 *	Description: This command gets the device state (stored in its device_state data member) and returns it to the caller.
	 *
	 *	@returns Device state
	 */
	virtual Tango::DevState dev_state();
	/**
	 *	Command Stop related method
	 *	Description: stopps the motor
	 *
	 */
	virtual void stop();
	virtual bool is_Stop_allowed(const CORBA::Any &any);
	/**
	 *	Command ClearAlarm related method
	 *	Description: 
	 *
	 */
	virtual void clear_alarm();
	virtual bool is_ClearAlarm_allowed(const CORBA::Any &any);
	/**
	 *	Command Calibrate related method
	 *	Description: 
	 *
	 *	@param argin 
	 */
	virtual void calibrate(Tango::DevLong argin);
	virtual bool is_Calibrate_allowed(const CORBA::Any &any);
	/**
	 *	Command Home related method
	 *	Description: 
	 *
	 */
	virtual void home();
	virtual bool is_Home_allowed(const CORBA::Any &any);


	//--------------------------------------------------------
	/**
	 *	Method      : TMCM_Motor::add_dynamic_commands()
	 *	Description : Add dynamic commands if any.
	 */
	//--------------------------------------------------------
	void add_dynamic_commands();

/*----- PROTECTED REGION ID(TMCM_Motor::Additional Method prototypes) ENABLED START -----*/

//	Additional Method prototypes
	void ChangeState(Tango::DevState newState, const std::string& str = "");

	/*template<typename T>
	void TrySetAttribute(Tango::Attribute& attr, TMCM::TMCMResoponse response) {
		if(response.GetStatus() != TMCM::ReturnCodes::SUCCESS) {
			ChangeState(Tango::ALARM, fmt::format("error on command attribute '{}', error: {}", attr.get_name(), response.GetStatus()));
			return;
		}
		T paramType = response.PayloadAsInt();
		attr.set_value(&paramType);
	}*/

	template<typename T>
	bool ValidateResponse(TMCM::TMCMResoponse response, T msg)
	{
		if(response.GetStatus() != TMCM::ReturnCodes::SUCCESS) {
			ChangeState(Tango::ALARM, msg());
			return false;
		}
		return true;
	}

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::Additional Method prototypes
};

/*----- PROTECTED REGION ID(TMCM_Motor::Additional Classes Definitions) ENABLED START -----*/

//	Additional Classes Definitions

/*----- PROTECTED REGION END -----*/	//	TMCM_Motor::Additional Classes Definitions

}	//	End of namespace

#endif   //	TMCM_Motor_H
