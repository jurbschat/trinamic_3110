/*----- PROTECTED REGION ID(TMCM_IOClass.h) ENABLED START -----*/
//=============================================================================
//
// file :        TMCM_IOClass.h
//
// description : Include for the TMCM_IO root class.
//               This class is the singleton class for
//                the TMCM_IO device class.
//               It contains all properties and methods which the 
//               TMCM_IO requires only once e.g. the commands.
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


#ifndef TMCM_IOClass_H
#define TMCM_IOClass_H

#include <tango.h>
#include <TMCM_IO.h>


/*----- PROTECTED REGION END -----*/	//	TMCM_IOClass.h


namespace TMCM_IO_ns
{
/*----- PROTECTED REGION ID(TMCM_IOClass::classes for dynamic creation) ENABLED START -----*/


/*----- PROTECTED REGION END -----*/	//	TMCM_IOClass::classes for dynamic creation

//=========================================
//	Define classes for attributes
//=========================================
//	Attribute Value class definition
class ValueAttrib: public Tango::Attr
{
public:
	ValueAttrib():Attr("Value",
			Tango::DEV_ULONG, Tango::READ_WRITE) {};
	~ValueAttrib() {};
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
		{(static_cast<TMCM_IO *>(dev))->read_Value(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
		{(static_cast<TMCM_IO *>(dev))->write_Value(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
		{return (static_cast<TMCM_IO *>(dev))->is_Value_allowed(ty);}
};


/**
 *	The TMCM_IOClass singleton definition
 */

#ifdef _TG_WINDOWS_
class __declspec(dllexport)  TMCM_IOClass : public Tango::DeviceClass
#else
class TMCM_IOClass : public Tango::DeviceClass
#endif
{
	/*----- PROTECTED REGION ID(TMCM_IOClass::Additionnal DServer data members) ENABLED START -----*/
	
	
	/*----- PROTECTED REGION END -----*/	//	TMCM_IOClass::Additionnal DServer data members

	public:
		//	write class properties data members
		Tango::DbData	cl_prop;
		Tango::DbData	cl_def_prop;
		Tango::DbData	dev_def_prop;
	
		//	Method prototypes
		static TMCM_IOClass *init(const char *);
		static TMCM_IOClass *instance();
		~TMCM_IOClass();
		Tango::DbDatum	get_class_property(string &);
		Tango::DbDatum	get_default_device_property(string &);
		Tango::DbDatum	get_default_class_property(string &);
	
	protected:
		TMCM_IOClass(string &);
		static TMCM_IOClass *_instance;
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

#endif   //	TMCM_IO_H
