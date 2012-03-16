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

#ifndef __ccxx_TimeSpan_hxx
#define __ccxx_TimeSpan_hxx

#include <commonc++/Common.h++>
#include <commonc++/String.h++>

namespace ccxx {

/** A span of time (a delta), represented in milliseconds.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API TimeSpan
{
  public:

  /** Construct a TimeSpan for the given number of milliseconds.
   *
   * @param span The span, in milliseconds.
   */
  TimeSpan(int64_t span = INT64_CONST(0)) throw();

  /** Construct a TimeSpan in terms of days, hours, minutes,
   *  seconds, and/or milliseconds.
   */
  TimeSpan(int days, int hours, int minutes, int seconds, int msec = 0)
    throw();

  /** Copy constructor. */
  TimeSpan(const TimeSpan& other) throw();

  /** Destructor. */
  ~TimeSpan() throw();

  /** Set the timespan. */
  void setSpan(int days, int hours, int minutes, int seconds, int msec = 0)
    throw();

  /** Set the timespan in seconds. */
  void setSpan(int64_t seconds) throw();

  /** Get the timespan, in milliseconds. */
  inline int64_t getSpan() const throw()
  { return(_span); }

  /** Set the days component of the timespan. */
  void setDays(int days) throw();

  /** Add days to the timespan. */
  void addDays(int days) throw();

  /** Subtract days from the timespan. */
  void subtractDays(int days) throw();

  /** Get the days component of the timespan. */
  inline int getDays() const throw()
  { return(_days); }

  /** Set the hours component of the timespan. */
  void setHours(int hours) throw();

  /** Add hours to the timespan. */
  void addHours(int hours) throw();

  /** Subtract hours from the timespan. */
  void subtractHours(int hours) throw();

  /** Get the hours component of the timespan. */
  inline int getHours() const throw()
  { return(_hours); }

  /** Set the minutes component of the timespan. */
  void setMinutes(int minutes) throw();

  /** Add minutes to the timespan. */
  void addMinutes(int minutes) throw();

  /** Subtract minutes from the timespan. */
  void subtractMinutes(int minutes) throw();

  /** Get the minutes component of the timespan. */
  inline int getMinutes() const throw()
  { return(_minutes); }

  /** Set the seconds component of the timespan. */
  void setSeconds(int seconds) throw();

  /** Add seconds to the timespan. */
  void addSeconds(int seconds) throw();

  /** Subtract seconds from the timespan. */
  void subtractSeconds(int seconds) throw();

  /** Get the seconds component of the timespan. */
  inline int getSeconds() const throw()
  { return(_seconds); }

  /** Set the milliseconds component of the timespan. */
  void setMilliseconds(int msec) throw();

  /** Add milliseconds to the timespan. */
  void addMilliseconds(int msec) throw();

  /** Subtract milliseconds from the timespan. */
  void subtractMilliseconds(int msec) throw();

  /** Get the milliseconds component of the timespan. */
  inline int getMilliseconds() const throw()
  { return(_msec); }

  TimeSpan& operator=(const TimeSpan& other) throw();

  inline bool operator==(const TimeSpan& other) const throw()
  { return(_span == other._span); }

  inline bool operator<(const TimeSpan& other) const throw()
  { return(_span < other._span); }

  inline bool operator<=(const TimeSpan& other) const throw()
  { return(_span <= other._span); }

  inline bool operator!=(const TimeSpan& other) const throw()
  { return(_span != other._span); }

  inline bool operator>(const TimeSpan& other) const throw()
  { return(_span > other._span); }

  inline bool operator>=(const TimeSpan& other) const throw()
  { return(_span >= other._span); }

  TimeSpan& operator+=(const TimeSpan& span) throw();
  TimeSpan& operator+=(int seconds) throw();
  TimeSpan& operator-=(const TimeSpan& span) throw();
  TimeSpan& operator-=(int seconds) throw();

  TimeSpan operator-() const throw();

  /** Get the timespan in seconds. */
  inline int64_t toSeconds() const throw()
  { return(_span / msInSecond); }

  /** Get the timespan in milliseconds. */
  inline operator int64_t() const throw()
  { return(_span); }

  /** Get the timespan in milliseconds. */
  inline int64_t toMilliseconds() const throw()
  { return(_span); }

  /** Get a String representation of the TimeSpan. */
  String toString() const;

  /** The number of milliseconds in a second. */
  static const int msInSecond;

  /** The number of milliseconds in a minute. */
  static const int msInMinute;

  /** The number of milliseconds in an hour. */
  static const int msInHour;

  /** The number of milliseconds in a day. */
  static const int msInDay;

  private:

  void _computeFields(int days, int hours, int minutes, int seconds,
                      int msec) throw();
  void _computeFields(int64_t span) throw();

  int _days;
  int _hours;
  int _minutes;
  int _seconds;
  int _msec;
  int64_t _span;
};

}; // namespace ccxx

#endif // __ccxx_TimeSpan_hxx

/* end of header file */
