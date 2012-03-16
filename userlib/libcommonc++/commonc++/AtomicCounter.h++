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

#ifndef __ccxx_AtomicCounter_hxx
#define __ccxx_AtomicCounter_hxx

#include <commonc++/Common.h++>
#include <commonc++/Mutex.h++>

namespace ccxx {

/** An integer counter whose value is modified in an atomic fashion.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API AtomicCounter
{
  public:

  /** Construct a new <b>AtomicCounter</b> with the given initial value.
   *
   * @param value The initial value.
   */
  AtomicCounter(int32_t value = 0) throw();

  /** Destructor. */
  ~AtomicCounter() throw();

  /** Copy constructor. */
  AtomicCounter(const AtomicCounter& other) throw();

  /** Assignment operator. */
  AtomicCounter& operator=(const AtomicCounter& other) throw();

  /** Increment the counter (prefix). */
  int32_t operator++() throw();

  /** Increment the counter (postfix). */
  int32_t operator++(int) throw();

  /** Decrement the counter (prefix). */
  int32_t operator--() throw();

  /** Decrement the counter (postfix). */
  int32_t operator--(int) throw();

  /** Add a value to the counter. */
  int32_t operator+=(int32_t delta) throw();

  /** Subtract a value from the counter. */
  int32_t operator-=(int32_t delta) throw();

  /** Compute the sum of the counter and a value. */
  int32_t operator+(int32_t delta) const throw();

  /** Compute the difference between the counter and a value. */
  int32_t operator-(int32_t delta) const throw();

  /** Assign a new value to the counter. */
  int32_t operator=(int32_t value) throw();

  /** Assign a new value to the counter, returning the new value. */
  int32_t set(int32_t value) throw();

  /** Assign a new value to the counter, returning the previous value. */
  int32_t swap(int32_t value) throw();

  /** Test and set the counter value.
   *
   * @param comparand The value to compare the counter to.
   * @param value The value to set the counter to, if the current value is
   * equal to the comparand.
   * @return The original (and possibly unchanged) value of the counter.
   */
  int32_t testAndSet(int32_t value, int32_t comparand) throw();

  /** Get the current value of the counter. */
  int32_t get() const throw();

  /** Cast operator. */
  inline operator int32_t() const throw()
  { return(get()); }

  private:

  mutable int32_t _atomic;
};

}; // namespace ccxx

#endif // __ccxx_AtomicCounter_hxx

/* end of header file */
