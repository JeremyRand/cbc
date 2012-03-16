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

#ifndef __ccxx_Array_hxx
#define __ccxx_Array_hxx

#include <commonc++/Common.h++>
#include <commonc++/OutOfBoundsException.h++>

namespace ccxx {

/** A simple object wrapper for arrays. See section 17.5.4 of
 * <i>The C++ Programming Language</i>.
 *
 * @author Mark Lindner
 */

template <typename T, size_t SIZE> class Array
{
  public:

  typedef T value_type;
  typedef T* iterator;
  typedef const T* const_iterator;
  typedef T& reference;
  typedef const T& const_reference;

  Array() throw()
    : _size(SIZE)
  { }

  ~Array() throw()
  { }

  T& operator[](int index) throw(OutOfBoundsException)
  {
    if((index < 0) || (index >= static_cast<int>(_size)))
      throw OutOfBoundsException();

    return(_data[index]);
  }

  const T& operator[](int index) const throw(OutOfBoundsException)
  {
    if((index < 0) || (index >= _size))
      throw OutOfBoundsException();

    return(_data[index]);
  }

  operator T*() throw()
  { return(_data); }

  iterator begin() throw()
  { return(_data); }

  const_iterator begin() const throw()
  { return(_data); }

  iterator end() throw()
  { return(_data + _size); }

  const_iterator end() const throw()
  { return(_data + _size); }

  size_t size() const throw()
  { return(_size); }

  private:

  size_t _size;
  T _data[SIZE];
};

}; // namespace ccxx

#endif // __ccxx_Array_hxx

/* end of header file */
