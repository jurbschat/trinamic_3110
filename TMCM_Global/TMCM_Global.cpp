/*----- PROTECTED REGION ID(TMCM_Global.cpp) ENABLED START -----*/
//=============================================================================
//
// file :        TMCM_Global.cpp
//
// description : C++ source for the TMCM_Global class and its commands.
//               The class is derived from Device. It represents the
//               CORBA servant object which will be accessed from the
//               network. All commands which can be executed on the
//               TMCM_Global are implemented in this file.
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


#include <TMCM_Global.h>
#include <TMCM_GlobalClass.h>
#include <fmt/format.h>
#include "../core/core.h"

/*----- PROTECTED REGION END -----*/	//	TMCM_Global.cpp

/**
 *  TMCM_Global class description:
 *    
 */

//================================================================
//  The following table gives the correspondence
//  between command and method names.
//
//  Command name  |  Method name
//================================================================
//  State         |  Inherited (no method)
//  Status        |  Inherited (no method)
//================================================================

//================================================================
//  Attributes managed is:
//================================================================
//================================================================

namespace TMCM_Global_ns
{
/*----- PROTECTED REGION ID(TMCM_Global::namespace_starting) ENABLED START -----*/

//	static initializations

/*----- PROTECTED REGION END -----*/	//	TMCM_Global::namespace_starting

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::TMCM_Global()
 *	Description : Constructors for a Tango device
 *                implementing the classTMCM_Global
 */
//--------------------------------------------------------
TMCM_Global::TMCM_Global(Tango::DeviceClass *cl, string &s)
 : TANGO_BASE_CLASS(cl, s.c_str())
{
	/*----- PROTECTED REGION ID(TMCM_Global::constructor_1) ENABLED START -----*/
	init_device();

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::constructor_1
}
//--------------------------------------------------------
TMCM_Global::TMCM_Global(Tango::DeviceClass *cl, const char *s)
 : TANGO_BASE_CLASS(cl, s)
{
	/*----- PROTECTED REGION ID(TMCM_Global::constructor_2) ENABLED START -----*/
	init_device();

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::constructor_2
}
//--------------------------------------------------------
TMCM_Global::TMCM_Global(Tango::DeviceClass *cl, const char *s, const char *d)
 : TANGO_BASE_CLASS(cl, s, d)
{
	/*----- PROTECTED REGION ID(TMCM_Global::constructor_3) ENABLED START -----*/
	init_device();

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::constructor_3
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::delete_device()
 *	Description : will be called at device destruction or at init command
 */
//--------------------------------------------------------
void TMCM_Global::delete_device()
{
	DEBUG_STREAM << "TMCM_Global::delete_device() " << device_name << endl;
	/*----- PROTECTED REGION ID(TMCM_Global::delete_device) ENABLED START -----*/

	//	Delete device allocated objects

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::delete_device
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::init_device()
 *	Description : will be called at device initialization.
 */
//--------------------------------------------------------
void TMCM_Global::init_device()
{
	DEBUG_STREAM << "TMCM_Global::init_device() create device " << device_name << endl;
	/*----- PROTECTED REGION ID(TMCM_Global::init_device_before) ENABLED START -----*/

	//	Initialization before get_device_property() call

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::init_device_before
	

	//	Get the device properties from database
	get_device_property();
	
	/*----- PROTECTED REGION ID(TMCM_Global::init_device) ENABLED START -----*/

	//	Initialize device
	auto& core = TMCM::GetCore();
	try {
		core.Init(serialPort, baudrate, std::vector<int32_t>(modules.begin(), modules.end()));
		set_state(Tango::ON);
	} catch (const std::exception& ex) {
		fmt::print("error opening serial port: {}\n", ex.what());
		set_state(Tango::FAULT);
		set_status(fmt::format("unable to open serial port '{}', error: ", serialPort, ex.what()));
	}

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::init_device
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::get_device_property()
 *	Description : Read database to initialize property data members.
 */
//--------------------------------------------------------
void TMCM_Global::get_device_property()
{
	/*----- PROTECTED REGION ID(TMCM_Global::get_device_property_before) ENABLED START -----*/

	//	Initialize property data members

	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::get_device_property_before


	//	Read device properties from database.
	Tango::DbData	dev_prop;
	dev_prop.push_back(Tango::DbDatum("serialPort"));
	dev_prop.push_back(Tango::DbDatum("baudrate"));
	dev_prop.push_back(Tango::DbDatum("modules"));

	//	is there at least one property to be read ?
	if (dev_prop.size()>0)
	{
		//	Call database and extract values
		if (Tango::Util::instance()->_UseDb==true)
			get_db_device()->get_property(dev_prop);
	
		//	get instance on TMCM_GlobalClass to get class property
		Tango::DbDatum	def_prop, cl_prop;
		TMCM_GlobalClass	*ds_class =
			(static_cast<TMCM_GlobalClass *>(get_device_class()));
		int	i = -1;

		//	Try to initialize serialPort from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  serialPort;
		else {
			//	Try to initialize serialPort from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  serialPort;
		}
		//	And try to extract serialPort value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  serialPort;

		//	Try to initialize baudrate from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  baudrate;
		else {
			//	Try to initialize baudrate from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  baudrate;
		}
		//	And try to extract baudrate value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  baudrate;

		//	Try to initialize modules from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  modules;
		else {
			//	Try to initialize modules from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  modules;
		}
		//	And try to extract modules value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  modules;

	}

	/*----- PROTECTED REGION ID(TMCM_Global::get_device_property_after) ENABLED START -----*/
	
	//	Check device property data members init
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::get_device_property_after
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::always_executed_hook()
 *	Description : method always executed before any command is executed
 */
//--------------------------------------------------------
void TMCM_Global::always_executed_hook()
{
	DEBUG_STREAM << "TMCM_Global::always_executed_hook()  " << device_name << endl;
	/*----- PROTECTED REGION ID(TMCM_Global::always_executed_hook) ENABLED START -----*/
	
	//	code always executed before all requests
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::always_executed_hook
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::read_attr_hardware()
 *	Description : Hardware acquisition for attributes
 */
//--------------------------------------------------------
void TMCM_Global::read_attr_hardware(TANGO_UNUSED(vector<long> &attr_list))
{
	DEBUG_STREAM << "TMCM_Global::read_attr_hardware(vector<long> &attr_list) entering... " << endl;
	/*----- PROTECTED REGION ID(TMCM_Global::read_attr_hardware) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::read_attr_hardware
}


//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::add_dynamic_attributes()
 *	Description : Create the dynamic attributes if any
 *                for specified device.
 */
//--------------------------------------------------------
void TMCM_Global::add_dynamic_attributes()
{
	/*----- PROTECTED REGION ID(TMCM_Global::add_dynamic_attributes) ENABLED START -----*/
	
	//	Add your own code to create and add dynamic attributes if any
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::add_dynamic_attributes
}

//--------------------------------------------------------
/**
 *	Method      : TMCM_Global::add_dynamic_commands()
 *	Description : Create the dynamic commands if any
 *                for specified device.
 */
//--------------------------------------------------------
void TMCM_Global::add_dynamic_commands()
{
	/*----- PROTECTED REGION ID(TMCM_Global::add_dynamic_commands) ENABLED START -----*/
	
	//	Add your own code to create and add dynamic commands if any
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_Global::add_dynamic_commands
}

/*----- PROTECTED REGION ID(TMCM_Global::namespace_ending) ENABLED START -----*/

//	Additional Methods

/*----- PROTECTED REGION END -----*/	//	TMCM_Global::namespace_ending
} //	namespace
