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

#ifndef __ccxx_Time_hxx
#define __ccxx_Time_hxx

#include <commonc++/Common.h++>
#include <commonc++/String.h++>

#include <ctime>

namespace ccxx {

/** A representation of a clock time.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API Time
{
  public:

  /** Construct a new Time initialized to 0:00:00:000 */
  Time() throw();

  /** Construct a new Time with the given components. */
  Time(uint_t hour, uint_t minute, uint_t second = 0, uint_t msec = 0)
    throw();

  /** Copy constructor. */
  Time(const Time& other) throw();

  /** Destructor. */
  virtual ~Time() throw();

  /** Set the hour component of the time. */
  void setHour(uint_t hour) throw();

  /** Set the hour component of the time (12-hour clock). */
  void setHour12(uint_t hour, bool pm = false) throw();

  /** Set the minute component of the time. */
  void setMinute(uint_t minute) throw();

  /** Set the second component of the time. */
  void setSecond(uint_t second) throw();

  /** Set the millisecond component of the time. */
  void setMillisecond(uint_t msec) throw();

  /** Set the hour, minute, second, and millisecond components of
   * the time.
   */
  void setTime(uint_t hour, uint_t minute, uint_t second = 0,
               uint_t msec = 0) throw();

  /** Get the hour component of the time. */
  inline uint_t getHour() const throw()
  { return(_hour); }

  /** Get the hour component of the time (12-hour clock). */
  inline uint_t getHour12() const throw()
  { return(_hour == 0 ? 12 : _hour % 12); }

  /** Test if the hour is post meridiem. */
  inline bool isPM() const throw()
  { return(_hour >= 12); }

  /** Test if the hour is ante meridiem. */
  inline bool isAM() const throw()
  { return(_hour < 12); }

  /** Get the minute component of the time. */
  inline uint_t getMinute() const throw()
  { return(_minute); }

  /** Get the second component of the time. */
  inline uint_t getSecond() const throw()
  { return(_second); }

  /** Get the millisecond component of the time. */
  inline uint_t getMillisecond() const throw()
  { return(_msec); }

  bool operator==(const Time& other) const throw();
  bool operator<(const Time& other) const throw();
  bool operator<=(const Time& other) const throw();

  inline bool operator!=(const Time& other) const throw()
  { return(! operator==(other)); }

  inline bool operator>(const Time& other) const throw()
  { return(! operator<=(other)); }

  inline bool operator>=(const Time& other) const throw()
  { return(! operator<(other)); }

  Time& operator=(const Time& other) throw();

  protected:

  uint_t _hour;
  uint_t _minute;
  uint_t _second;
  uint_t _msec;
};

}; // namespace ccxx

#endif // __ccxx_Time_hxx

/* end of header file */
