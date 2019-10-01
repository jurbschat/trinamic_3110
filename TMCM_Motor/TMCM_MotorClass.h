/*----- PROTECTED REGION ID(TMCM_MotorClass.h) ENABLED START -----*/
//=============================================================================
//
// file :        TMCM_MotorClass.h
//
// description : Include for the TMCM_Motor root class.
//               This class is the singleton class for
//                the TMCM_Motor device class.
//               It contains all properties and methods which the 
//               TMCM_Motor requires only once e.g. the commands.
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


#ifndef TMCM_MotorClass_H
#define TMCM_MotorClass_H

#include <tango.h>
#include <TMCM_Motor.h>


/*----- PROTECTED REGION END -----*/	//	TMCM_MotorClass.h


namespace TMCM_Motor_ns
{
/*----- PROTECTED REGION ID(TMCM_MotorClass::classes for dynamic creation) ENABLED START -----*/


/*----- PROTECTED REGION END -----*/	//	TMCM_MotorClass::classes for dynamic creation

//=========================================
//	Define classes for attributes
//=========================================
//	Attribute Position class definition
class PositionAttrib: public Tango::Attr
{
public:
	PositionAttrib():Attr("Position",
			Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~PositionAttrib() {};
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
		{(static_cast<TMCM_Motor *>(dev))->read_Position(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
		{(static_cast<TMCM_Motor *>(dev))->write_Position(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
		{return (static_cast<TMCM_Motor *>(dev))->is_Position_allowed(ty);}
};

//	Attribute Velocity class definition
class VelocityAttrib: public Tango::Attr
{
public:
	VelocityAttrib():Attr("Velocity",
			Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~VelocityAttrib() {};
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
		{(static_cast<TMCM_Motor *>(dev))->read_Velocity(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
		{(static_cast<TMCM_Motor *>(dev))->write_Velocity(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
		{return (static_cast<TMCM_Motor *>(dev))->is_Velocity_allowed(ty);}
};

//	Attribute Acceleration class definition
class AccelerationAttrib: public Tango::Attr
{
public:
	AccelerationAttrib():Attr("Acceleration",
			Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~AccelerationAttrib() {};
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
		{(static_cast<TMCM_Motor *>(dev))->read_Acceleration(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
		{(static_cast<TMCM_Motor *>(dev))->write_Acceleration(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
		{return (static_cast<TMCM_Motor *>(dev))->is_Acceleration_allowed(ty);}
};


//=========================================
//	Define classes for commands
//=========================================
//	Command Stop class definition
class StopClass : public Tango::Command
{
public:
	StopClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	StopClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~StopClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<TMCM_Motor *>(dev))->is_Stop_allowed(any);}
};


/**
 *	The TMCM_MotorClass singleton definition
 */

#ifdef _TG_WINDOWS_
class __declspec(dllexport)  TMCM_MotorClass : public Tango::DeviceClass
#else
class TMCM_MotorClass : public Tango::DeviceClass
#endif
{
	/*----- PROTECTED REGION ID(TMCM_MotorClass::Additionnal DServer data members) ENABLED START -----*/
	
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_MotorClass::Additionnal DServer data members

	public:
		//	write class properties data members
		Tango::DbData	cl_prop;
		Tango::DbData	cl_def_prop;
		Tango::DbData	dev_def_prop;
	
		//	Method prototypes
		static TMCM_MotorClass *init(const char *);
		static TMCM_MotorClass *instance();
		~TMCM_MotorClass();
		Tango::DbDatum	get_class_property(string &);
		Tango::DbDatum	get_default_device_property(string &);
		Tango::DbDatum	get_default_class_property(string &);
	
	protected:
		TMCM_MotorClass(string &);
		static TMCM_MotorClass *_instance;
		void command_factory();
		void attribute_factory(vector<Tango::Attr *> &);
		void pipe_factory();
		void write_class_property();
		void set_default_property();
		void get_class_property();
		string get_cvstag();
		string get_cvsroot();
	
	private:
		void device_factory(const Tango::DevVarStringArray *);
		void create_static_attribute_list(vector<Tango::Attr *> &);
		void erase_dynamic_attributes(const Tango::DevVarStringArray *,vector<Tango::Attr *> &);
		vector<string>	defaultAttList;
		Tango::Attr *get_attr_object_by_name(vector<Tango::Attr *> &att_list, string attname);
};

}	//	End of namespace

#endif   //	TMCM_Motor_H
