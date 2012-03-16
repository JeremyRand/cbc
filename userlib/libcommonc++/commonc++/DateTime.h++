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

#ifndef __ccxx_DateTime_hxx
#define __ccxx_DateTime_hxx

#include <commonc++/Common.h++>
#include <commonc++/Date.h++>
#include <commonc++/String.h++>
#include <commonc++/Time.h++>
#include <commonc++/TimeSpan.h++>

namespace ccxx {

/** A representation of a calendar date and clock time.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API DateTime : public Date, public Time
{
  public:

  /** Construct a DateTime for the given system time. */
  DateTime(time_s_t time) throw();

  /** Construct a DateTime for the given time (or the current time). */
  DateTime(time_ms_t time = 0) throw();

  /** Construct a DateTime with the given components. */
  DateTime(uint_t day, uint_t month, uint_t year, uint_t hour = 0,
           uint_t minute = 0, uint_t second = 0, uint_t msec = 0) throw();

  /** Construct a DateTime from the given <b>Date</b> and <b>Time</b>. */
  DateTime(const Date& date, const Time& time) throw();

  DateTime(const Date& date) throw();

  DateTime(const Time& time) throw();

  /** Copy constructor. */
  DateTime(const DateTime& other) throw();

  /** Destructor. */
  virtual ~DateTime() throw();

  /** Normalize the DateTime. This method converts invalid dates
   * into valid dates. For example, a date of "February 31, 2000" is
   * normalized to "March 2, 2000".
   */
  DateTime& normalize() throw();

  /** Cast operator. */
  operator time_s_t() const throw();

  /** Cast operator. */
  operator time_ms_t() const throw();

  bool operator==(const DateTime& other) const throw();
  bool operator<(const DateTime& other) const throw();
  bool operator<=(const DateTime& other) const throw();

  inline bool operator!=(const DateTime& other) const throw()
  { return(! operator==(other)); }

  inline bool operator>(const DateTime& other) const throw()
  { return(! operator<=(other)); }

  inline bool operator>=(const DateTime& other) const throw()
  { return(! operator<(other)); }

  DateTime& operator=(time_s_t other) throw();
  DateTime& operator=(time_ms_t other) throw();
  DateTime& operator=(const DateTime& other) throw();

  /** Set the hour, minute, second, and millisecond components of
   * the time.
   */
  inline void setTime(uint_t hour, uint_t minute, uint_t second = 0,
                      uint_t msec = 0) throw()
  { Time::setTime(hour, minute, second, msec); }

  /** Set the day, month, and year components of the date. */
  inline void setDate(uint_t day, uint_t month, uint_t year) throw()
  { Date::setDate(day, month, year); }

  DateTime& setDate(const Date& other) throw();
  DateTime& setTime(const Time& other) throw();

  DateTime& operator+=(const TimeSpan& span) throw();
  DateTime& operator-=(const TimeSpan& span) throw();

  /** Test if Daylight Saving Time is in effect for the current local
   * time.
   */
  static bool isDSTActive() throw();

  /** Get the offset, in seconds, between local time and GMT. */
  static int getGMTOffset() throw();

  /** Get the name of the local time zone. */
  static String getTimeZone() throw();
};

inline const TimeSpan operator-(const DateTime& d1, const DateTime& d2)
  throw()
{
  time_ms_t t1 = d1;
  time_ms_t t2 = d2;
  return TimeSpan(t1 >= t2 ? t1 - t2 : t2 - t1);
}

inline const DateTime operator-(const DateTime& d, const TimeSpan& ts)
{
  DateTime tmp(d);
  tmp -= ts;
  return tmp;
}

inline const DateTime operator+(const DateTime& d, const TimeSpan& ts)
{
  DateTime tmp(d);
  tmp += ts;
  return tmp;
}

}; // namespace ccxx

#endif // __ccxx_DateTime_hxx

/* end of header file */
