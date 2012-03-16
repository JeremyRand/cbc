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

#ifndef __ccxx_CircularBuffer_hxx
#define __ccxx_CircularBuffer_hxx

#include <commonc++/Common.h++>
#include <commonc++/AbstractBuffer.h++>
#include <commonc++/IOException.h++>
#include <commonc++/MemoryBlock.h++>
#include <commonc++/Stream.h++>

#include <algorithm>
#include <cstdlib>
#include <cstring>

#ifdef CCXX_OS_POSIX
#include <sys/uio.h>
#endif

namespace ccxx {

/** A circular buffer that can be used to efficiently transfer data between
 * buffers and/or streams. The buffer has a <i>read position</i> and a
 * <i>write position</i>, as well as a <i>peek position</i>. When either
 * the read position or the write position reaches the end of the buffer,
 * it wraps around to the beginning, but the read position can never
 * overtake the write position. The peek position is always between the
 * read position and the write position, and is used to "look ahead" for
 * a specific value.
 *
 * @author Mark Lindner
 */

template <typename T> class CircularBuffer : public AbstractBuffer<T>
{
  public:

  /** Construct a new <b>CircularBuffer</b> with the given size.
   *
   * @param size The capacity of the buffer, in elements.
   */
  CircularBuffer(size_t size);

  /** Destructor. */
  ~CircularBuffer() throw();

  /** Clear the buffer. Resets the buffer to an "empty" state. */
  void clear() throw();

  /** Resize the buffer.
   *
   * @param newSize The new size, in elements.
   */
  virtual void setSize(size_t newSize);

  /** Write data from an array into the buffer.
   *
   * @param buf The base of the array.
   * @param count The number of elements to read from the array.
   * @return The number of elements actually read.
   */
  size_t write(const T *buf, size_t count) throw();

  /** Write data from a <b>Buffer</b> into the buffer.
   *
   * @param buffer The buffer of data to write.
   * @param count The number of elements to write from the buffer, or
   * 0 to write as much as possible.
   * @return The number of elements actually read.
   */
  size_t write(Buffer<T>& buffer, size_t count = 0) throw();

  /** Read data from a stream and write it into the buffer.
   *
   * @param stream The stream to read from.
   * @param count The number of elements to write from the stream.
   * @return The number of elements actually written to the buffer.
   * @throw commoncpp::IOException If an I/O error occurs.
   */
  size_t write(Stream& stream, size_t count = 0) throw(IOException);

  /** Read data from the buffer into an array.
   *
   * @param buf The base of the array.
   * @param count The number of elements to write to the array.
   * @return The number of elements actually written.
   */
  size_t read(T *buf, size_t count) throw();

  /** Read data from the buffer into a <b>Buffer</b>.
   *
   * @param buffer The buffer to read into.
   * @param count The number of elements to copy to the buffer, or 0 to
   * write as much as possible.
   * @return The number of elements actually written.
   */
  size_t read(Buffer<T>& buffer, size_t count = 0) throw();

  /** Read data from the buffer and write it to a stream.
   *
   * @param stream The stream to write to.
   * @param count The number of bytes to write to the stream.
   * @return The number of bytes actually written.
   * @throw commoncpp::IOException If an I/O error occurs.
   */
  size_t read(Stream& stream, size_t count = 0) throw(IOException);

  /** Scan forward from the current peek position for an element
   * equal to the given value.
   *
   * @param value The value to scan for.
   * @param maxlen The maximum number of elements to scan.
   * @param found A flag that is set to <b>true</b> if the item was found,
   * and <b>false</b> otherwise.
   * @param resetPeek If <b>true</b>, reset the peek position to the read
   * position before scanning.
   * @return The number of elements between the current read position and
   * the peeked value, inclusive, if found; otherwise, the number of
   * elements scanned (up to <b>maxlen</b>).
   */
  size_t peek(const T &value, size_t maxlen, bool &found,
              bool resetPeek = true)
    throw();

  /** Fill the buffer with the given value. Fills the requested
   * number of items. If the requested count exceeds the number of
   * items available to be written, only the available (free) items
   * are filled.
   *
   * @param value The value to fill with.
   * @param count The number of items to fill.
   * @return The number of items actually filled.
   */
  size_t fill(const T &value, size_t count) throw();

  /** Get the read extent.
   *
   * @return The number of contiguous elements that can be read
   * beginning at the current read position, without wrapping to the
   * beginning of the buffer.
   */
  size_t getReadExtent() const throw();

  /** Get the write extent.
   *
   * @return The number of contiguous elements that can be written
   * beginning at the current write position, without wrapping to
   * the beginning of the buffer.
   */
  size_t getWriteExtent() const throw();

  /** Get the number of elements available to be read from the buffer. */
  inline size_t getRemaining() const throw()
  { return(_avail); }

  /** Get the number of elements available to be peeked, that is, the
   * number of elements between the peek position and the write position.
   */
  inline size_t getPeekRemaining() const throw()
  { return(_peekAvail); }

  /** Determine if the buffer is empty. */
  inline bool isEmpty() const throw()
  { return(_avail == 0); }

  /** Determine if the buffer is full. */
  inline bool isFull() const throw()
  { return(this->_size == _avail); }

  /** Get the number of elements available to be written to the buffer. */
  inline size_t getFree() const throw()
  { return(this->_size - _avail); }

  /** Get the current read position. */
  inline T *getReadPos() throw()
  { return(_readPos); }

  /** Get the current write position. */
  inline T *getWritePos() throw()
  { return(_writePos); }

  /** Get the current peek position. */
  inline T *getPeekPos() throw()
  { return(_peekPos); }

  /** Advance the read position by the given number of elements.
   *
   * @param count The number of elements to advance. If the value is
   * greater than the number of bytes available to be read, the read
   * position is left unchanged.
   * @return The new read position.
   */
  T *advanceReadPos(size_t count) throw();

  /** Advance the write position by the given number of elements.
   *
   * @param count The number of elements to advance. If the value is
   * greater than the number of elements available to be written, the
   * write position is left unchanged.
   *
   * @return The new write position.
   */
  T *advanceWritePos(size_t count) throw();

  /** Advance the read position by the number of elements in the
   * read extent, wrapping to the beginning of the buffer if
   * necessary.
   *
   * @return The new read position.
   */
  inline T *advanceReadPos() throw()
  { return(advanceReadPos(getReadExtent())); }

  /** Advance the write position by the number of elements in the
   * write extent, wrapping to the beginning of the buffer if
   * necessary.
   *
   * @return The new write position.
   */
  inline T *advanceWritePos() throw()
  { return(advanceWritePos(getWriteExtent())); }

  /** Advance the peek position by the given number of elements, wrapping
   * to the beginning of the buffer if necessary. If there are not enough
   * elements between the current peek position and the write position, the
   * peek position is not moved.
   *
   * @param count The number of elements to advance.
   * @return The new peek position.
   */
  T *advancePeekPos(size_t count) throw();

  /** Reset the peek position to the read position. */
  void resetPeekPos() throw();

  /** Determine if a partially-written element is in the buffer.
   * This will always be <b>false</b> when the template parameter is
   * an object of size 1.
   */
  bool isPartialWrite() const throw()
  { return(_writeShift > 0); }

  /** Determine if a partially-read element is in the buffer. This
   * will always be <b>false</b> when the template parameter is an
   * object of size 1.
   */
  bool isPartialRead() const throw()
  { return(_readShift > 0); }

  private:

  T *_end;
  T *_readPos;
  T *_writePos;
  T *_peekPos;
  size_t _avail;
  size_t _peekAvail;
  size_t _readShift;
  size_t _writeShift;

  CCXX_COPY_DECLS(CircularBuffer);
};

#include <commonc++/CircularBufferImpl.h++>

typedef CircularBuffer<byte_t> CircularByteBuffer;
typedef CircularBuffer<char> CircularCharBuffer;

}; // namespace ccxx

#endif // __ccxx_CircularBuffer_hxx

/* end of header file */
