//
// Created by urbschaj on 2019-10-01.
//

#include "tangohelpers.h"
#include <fmt/format.h>
#include <tango.h>

namespace tangohelpers {

	void TranslateException(const TMCM::Exception& ex) {
		//std::stringstream ss;
		//ss << ex.GetFunction() << "(" << ex.GetFile() << ":" << ex.GetLine() << ")";
		//auto str = fmt::format("");
		Tango::Except::throw_exception("core errro", ex.what(), "somewhere");
	}
}

