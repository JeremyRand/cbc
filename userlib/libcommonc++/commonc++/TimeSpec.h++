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

#ifndef __ccxx_TimeSpec_hxx
#define __ccxx_TimeSpec_hxx

#include <commonc++/Common.h++>
#include <commonc++/BitSet.h++>
#include <commonc++/DateTime.h++>
#include <commonc++/String.h++>

namespace ccxx {

/** A time specification, such as that used by <b>cron</b>
 * servers. The TimeSpec can be thought of as an expression for
 * matching against dates/times, down to a one-minute resolution,
 * which is adequate for most automated task scheduling. The
 * TimeSpec maintains a list of bitmasks representing all possible
 * hours of the day, minutes of the hour, days of the week, days of
 * the month, and months of the year. Individual minutes, hours,
 * days, weekdays, and months can be toggled "on" or "off" in the
 * TimeSpec.  The TimeSpec can then be compared against a time
 * value; if the hour, minute, day, weekday, and month components of
 * that time correspond to bits that are "on" in the corresponding
 * bitmasks, then the match is successful.
 *
 * A TimeSpec can be represented in textual form as five tokens
 * separated by spaces; these five tokens correspond to the minute,
 * hour, day, month, and weekday values that are set in the
 * corresponding masks.  Each of these tokens may consist of a
 * comma-separated series of wildcards (denoted by an asterisk '*'),
 * single values, or value ranges, the latter being defined as two
 * integers separated by a dash ('-'). For example, the specification:
 *
 * <pre>0 * 1-5,7 * 0</pre>
 *
 * represents a TimeSpec that matches every hour, on the hour, on
 * the first through fifth or the seventh day of any month, but only
 * if it is a Sunday.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API TimeSpec
{
  public:

  /** Construct a new TimeSpec with all masks empty. */
  TimeSpec() throw();

  /** Construct a new TimeSpec that matches the given date/time. */
  TimeSpec(const DateTime& time) throw();

  /** Construct a new TimeSpec that matches the given hour and minute,
   * and all possible dates.
   */
  TimeSpec(uint_t hour, uint_t minute) throw();

  /** Construct a new TimeSpec that matches the given hour, minute,
   * month and day, and all weekdays.
   */
  TimeSpec(uint_t hour, uint_t minute, uint_t month, uint_t day) throw();

  /** Construct a new TimeSpec from the given String representation. */
  TimeSpec(const String& spec);

  /** Destructor. */
  virtual ~TimeSpec() throw();

  /** Set the specification according to the given String. */
  void setSpec(const String& spec);

  /** Set a month in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param month The month, a value from 1 to 12, inclusive.
   */
  TimeSpec& setMonth(uint_t month) throw();

  /** Set a range of months in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param month1 The start month, a value from 1 to 12, inclusive.
   * @param month2 The start month, a value from 1 to 12, inclusive.
   */
  TimeSpec& setMonths(uint_t month1, uint_t month2) throw();

  /** Clear a month in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param month The month, a value from 1 to 12, inclusive.
   */
  TimeSpec& clearMonth(uint_t month) throw();

  /** Clear a range of months in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param month1 The start month, a value from 1 to 12, inclusive.
   * @param month2 The end month, a value from 1 to 12, inclusive.
   */
  TimeSpec& clearMonths(uint_t month1, uint_t month2) throw();

  /** Set all of the months in the TimeSpec. */
  TimeSpec& setAllMonths() throw();

  /** Clear all of the months in the TimeSpec. */
  TimeSpec& clearAllMonths() throw();

  /** Set a day in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param day The day, a value from 1 to 31, inclusive.
   */
  TimeSpec& setDay(uint_t day) throw();

  /** Set a range of days in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param day1 The start day, a value from 1 to 31, inclusive.
   * @param day2 The end day, a value from 1 to 31, inclusive.
   */
  TimeSpec& setDays(uint_t day1, uint_t day2) throw();

  /** Clear a day in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param day The day, a value from 1 to 31, inclusive.
   */
  TimeSpec& clearDay(uint_t day) throw();

  /** Clear a range of days in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param day1 The start day, a value from 1 to 31, inclusive.
   * @param day2 The end day, a value from 1 to 31, inclusive.
   */
  TimeSpec& clearDays(uint_t day1, uint_t day2) throw();

  /** Set all of the days in the TimeSpec. */
  TimeSpec& setAllDays() throw();

  /** Clear all of the days in the TimeSpec. */
  TimeSpec& clearAllDays() throw();

  /** Set a weekday in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param dow The weekday, a value from 0 to 7, inclusive, where
   * both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   */
  TimeSpec& setDayOfWeek(uint_t dow) throw();

  /** Set a range of weekdays in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param dow1 The start weekday, a value from 0 to 7, inclusive,
   * where both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   * @param dow2 The end weekday, a value from 0 to 7, inclusive, where
   * both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   */
  TimeSpec& setDaysOfWeek(uint_t dow1, uint_t dow2) throw();

  /** Clear a weekday in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param dow The weekday, a value from 0 to 7, inclusive, where
   * both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   */
  TimeSpec& clearDayOfWeek(uint_t dow) throw();

  /** Clear a range of weekdays in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param dow1 The start weekday, a value from 0 to 7, inclusive,
   * where both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   * @param dow2 The end weekday, a value from 0 to 7, inclusive, where
   * both 0 and 7 indicate Sunday, and the remaining values indicate
   * the number of days since Sunday.
   */
  TimeSpec& clearDaysOfWeek(uint_t dow1, uint_t dow2) throw();

  /** Set all of the weekdays in the TimeSpec. */
  TimeSpec& setAllDaysOfWeek() throw();

  /** Clear all of the weekdays in the TimeSpec. */
  TimeSpec& clearAllDaysOfWeek() throw();

  /** Set an hour in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param hour The hour, a value from 0 to 23, inclusive.
   */
  TimeSpec& setHour(uint_t hour) throw();

  /** Set a range of hours in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param hour1 The start hour, a value from 0 to 23, inclusive.
   * @param hour2 The end hour, a value from 0 to 23, inclusive.
   */
  TimeSpec& setHours(uint_t hour1, uint_t hour2) throw();

  /** Clear an hour in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param hour The hour, a value from 0 to 23, inclusive.
   */
  TimeSpec& clearHour(uint_t hour) throw();

  /** Clear a range of hours in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param hour1 The start hour, a value from 0 to 23, inclusive.
   * @param hour2 The end hour, a value from 0 to 23, inclusive.
   */
  TimeSpec& clearHours(uint_t hour1, uint_t hour2) throw();

  /** Set all of the hours in the TimeSpec. */
  TimeSpec& setAllHours() throw();

  /** Clear all of the hours in the TimeSpec. */
  TimeSpec& clearAllHours() throw();

  /** Set a minute in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param minute The minute, a value from 0 to 59, inclusive.
   */
  TimeSpec& setMinute(uint_t minute) throw();

  /** Set a range of minutes in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param minute1 The start minute, a value from 0 to 59, inclusive.
   * @param minute2 The end minute, a value from 0 to 59, inclusive.
   */
  TimeSpec& setMinutes(uint_t minute1, uint_t minute2) throw();

  /** Clear a minute in the TimeSpec. If the value is invalid,
   * the TimeSpec is not changed.
   *
   * @param minute The minute, a value from 0 to 59, inclusive.
   */
  TimeSpec& clearMinute(uint_t minute) throw();

  /** Clear a range of minutes in the TimeSpec. If the range is invalid,
   * the TimeSpec is not changed.
   *
   * @param minute1 The start minute, a value from 0 to 59, inclusive.
   * @param minute2 The end minute, a value from 0 to 59, inclusive.
   */
  TimeSpec& clearMinutes(uint_t minute1, uint_t minute2) throw();

  /** Set all of the minutes in the TimeSpec. */
  TimeSpec& setAllMinutes() throw();

  /** Clear all of the minutes in the TimeSpec. */
  TimeSpec& clearAllMinutes() throw();

  /** Test if the TimeSpec matches the given time.
   *
   * @param time The time to compare against.
   * @return <b>true</b> if there is a match, <b>false</b> otherwise.
   */
  bool matches(time_s_t time) const throw();

  /** Test if the TimeSpec matches the given time.
   *
   * @param time The time to compare against.
   * @return <b>true</b> if there is a match, <b>false</b> otherwise.
   */
  bool matches(time_ms_t time) const throw();

  /** Test if the TimeSpec matches the given time.
   *
   * @param month The month.
   * @param day The day.
   * @param weekday The weekday.
   * @param hour The hour.
   * @param minute The minute.
   * @return <b>true</b> if there is a match, <b>false</b> otherwise.
   */
  bool matches(uint_t month, uint_t day, uint_t weekday, uint_t hour,
               uint_t minute) const throw();

  /** Get a String representation of the TimeSpec. */
  String toString() const;

  private:

  void _parseValues(String str, uint_t base, BitSet& set) throw();
  void _set(BitSet& bset, uint_t base, uint_t pos, bool set) throw();
  void _setRange(BitSet& bset, uint_t base, uint_t min, uint_t max,
                 bool set) throw();
  void _format(const BitSet& set, uint_t base, String& s) const;

  BitSet _months;
  BitSet _days;
  BitSet _dow;
  BitSet _hours;
  BitSet _minutes;
};

}; // namespace ccxx

#endif // __ccxx_TimeSpec_hxx

/* end of header file */
