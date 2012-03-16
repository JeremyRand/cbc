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

#ifndef __ccxx_Numeric_hxx
#define __ccxx_Numeric_hxx

#include <limits>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include <commonc++/OutOfBoundsException.h++>

namespace ccxx {

/** A range-checked numeric cast. If the value is outside the range of
 * the target type, an exception is thrown.
 *
 * @param value The value to cast.
 * @return The value cast to the target type.
 * @throw OutOfBoundsException If the value is outside the range of the
 * target type.
 */
template<typename S, typename T>
  inline T numeric_cast(S value) throw(OutOfBoundsException)
{
  if((value < std::numeric_limits<T>::min())
     || (value > std::numeric_limits<T>::max()))
    throw OutOfBoundsException();
  else
    return(static_cast<T>(value));
}

/** A range-clipping numeric cast. If the value is outside the range of
 * the target type, it is clipped.
 *
 * @param value The value to cast.
 * @return The value cast to the target type.
 */
template<typename S, typename T>
  inline T numeric_clipping_cast(S value) throw()
{
  if(value < std::numeric_limits<T>::min())
    return(std::numeric_limits<T>::min());
  else if(value > std::numeric_limits<T>::max())
    return(std::numeric_limits<T>::max());
  else
    return(static_cast<T>(value));
}

/** A numeric range-clipping function. If the value is outside the specified
 * range, it is clipped.
 *
 * @param value The value to cast.
 * @param min The lower bound of the allowed range.
 * @param max The upper bound of the allowed range.
 * @return The (possibly clipped) value.
 */
template<typename T>
  inline T numeric_clip(T value, T min, T max) throw()
{
  if(value < min)
    return(min);
  else if(value > max)
    return(max);
  else return(value);
}

}; // namespace ccxx

#endif // __ccxx_Numeric_hxx

/* end of header file */
