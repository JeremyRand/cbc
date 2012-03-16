/* ---------------------------------------------------------------------------
   commonc++ - A C++ Common Class Library
   Copyright (C) 2005-2009  Mark A Lindner

   This file is part of commonc++.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the Free
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
   ---------------------------------------------------------------------------
*/

#ifndef __ccxx_String_hxx
#define __ccxx_String_hxx

#include <commonc++/Common.h++>
#include <commonc++/BasicString.h++>

namespace ccxx {

/** @typedef commoncpp::BasicString<char> commoncpp::String
 * An ASCII String.
 */
typedef BasicString<char> String;

/** @typedef commoncpp::BasicStringVec<char> commoncpp::StringVec
 * A String vector.
 */
typedef BasicStringVec<char> StringVec;

/** @typedef BasicString<wchar_t> WString
 * A wide (Unicode) String.
 */
typedef BasicString<wchar_t> WString;

/** @typedef commoncpp::BasicStringVec<wchar_t> commoncpp::WStringVec
 * A WString vector.
 */
typedef BasicStringVec<wchar_t> WStringVec;

}; // namespace ccxx

#endif // __ccxx_String_hxx

/* end of header file */
