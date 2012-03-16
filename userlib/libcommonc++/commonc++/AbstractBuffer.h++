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

#ifndef __ccxx_AbstractBuffer_hxx
#define __ccxx_AbstractBuffer_hxx

#include <commonc++/Common.h++>

#include <cstring>

namespace ccxx {

/** An abstract base class for buffers.
 *
 * @author Mark Lindner
 */

template<typename T> class AbstractBuffer
{
  public:

  /** Destructor. */

  virtual ~AbstractBuffer() throw();

  /** Resize the buffer.
   *
   * @param newSize The new size, in elements.
   */

  virtual void setSize(size_t newSize);

  /** Clear the buffer. */

  virtual void clear() throw();

  /** Get the size of the buffer.
   *
   * @return The size, in elements.
   */

  inline size_t getSize() const throw()
  { return(_size); }

  /** Get the number of elements remaining in the buffer.
   */

  virtual size_t getRemaining() const throw() = 0;

  /** Test if the buffer has elements available to be read or written.
   */

  inline virtual bool hasRemaining() const throw()
  { return(getRemaining() > 0); }

  /** Get a pointer to the base of the buffer. */

  inline T *getBase() throw()
  { return(_data); }

  /** Get a pointer to the base of the buffer. */

  inline const T *getBase() const throw()
  { return(_data); }

  protected:

  /** Construct a new buffer with the given size.
   *
   * @param size The capacity of the buffer, in elements.
   */

  AbstractBuffer(size_t size);

  /** A pointer to the raw buffer. */
  T *_data;
  /** The size of the buffer. */
  size_t _size;

  private:

  CCXX_COPY_DECLS(AbstractBuffer);
};

#include <commonc++/AbstractBufferImpl.h++>

}; // namespace ccxx

#endif // __ccxx_AbstractBuffer_hxx

/* end of header file */
