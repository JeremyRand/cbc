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

#ifndef __ccxx_CircularBufferImpl_hxx
#define __ccxx_CircularBufferImpl_hxx

#ifndef __ccxx_CircularBuffer_hxx
#error "Do not include this header directly from application code!"
#endif

/*
 */

template <typename T>
  CircularBuffer<T>::CircularBuffer(size_t size)
    : AbstractBuffer<T>(size)
{
  _end = this->_data + this->_size;

  clear();
}

/*
 */

template <typename T>
  CircularBuffer<T>::~CircularBuffer() throw()
{
}

/*
 */

template <typename T>
  void CircularBuffer<T>::clear() throw()
{
  AbstractBuffer<T>::clear();

  _readPos = _writePos = _peekPos = this->_data;
  _readShift = _writeShift = 0;
  _avail = 0;
  _peekAvail = 0;
}

/*
 */

template <typename T>
  void CircularBuffer<T>::setSize(size_t newSize)
{
  AbstractBuffer<T>::setSize(newSize);
  _end = this->_data + this->_size;
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::write(const T *buf, size_t len) throw()
{
  if(_writeShift)
    return(0);

  size_t left = len, n, f, w = 0;

  if((_writePos > _readPos) || (_avail == 0))
  {
    f = (_end - _writePos);
    n = std::min(left, f);

    std::memcpy(static_cast<void *>(_writePos), static_cast<const void *>(buf),
                n * sizeof(T));

    w += n;
    left -= n;
    buf += n;

    advanceWritePos(n);
  }

  if(left > 0)
  {
    f = (_readPos - _writePos);

    if(f > 0)
    {
      n = std::min(left, f);

      std::memcpy(static_cast<void *>(_writePos),
                  static_cast<const void *>(buf),
                  n * sizeof(T));

      w += n;

      advanceWritePos(n);
    }
  }

  return(w);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::write(Buffer<T>& buffer, size_t count /* = 0 */)
  throw()
{
  if(_writeShift)
    return(0);

  if(count > buffer.getRemaining())
    return(0);
  else if(count == 0)
    count = buffer.getRemaining();

  const T *buf = buffer.getPointer();

  size_t r = write(buf, count);
  buffer.bump(r);

  return(r);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::write(Stream& stream, size_t count /* = 0 */)
  throw(IOException)
{
  size_t w = 0;

  MemoryBlock iov[2];
  int iol = 1;

  if((count == 0) || (count > getFree()))
    count = getFree();

  if(count == 0)
    return(0);

  // First segment extends from the write position to either the read
  // position or, if the read position is behind the write position,
  // to the end of the buffer. In the latter case, second segment
  // extends from beginning of buffer to the read position.

  iov[0].setBase(reinterpret_cast<byte_t *>(_writePos) + _writeShift);
  if(_writePos < _readPos)
    iov[0].setSize((std::min(count, static_cast<size_t>(_readPos
                                                        - _writePos))
                    * sizeof(T)) - _writeShift);
  else
  {
    size_t n = std::min(count, static_cast<size_t>(_end - _writePos));

    iov[0].setSize((n * sizeof(T)) - _writeShift);
    count -= n;

    if(count > 0)
    {
      size_t f = (_readPos - this->_data);
      if(f > 0)
      {
        ++iol;
        iov[1].setBase(reinterpret_cast<byte_t *>(this->_data));
        iov[1].setSize(std::min(count, f) * sizeof(T));
      }
    }
  }

  size_t cw = 0;
  w = stream.read(iov, iol);

  if(_writeShift)
  {
    size_t part = (sizeof(T) - _writeShift);
    if(w >= part)
    {
      ++cw;
      advanceWritePos(1);
      w -= part;
    }
  }

  if(w > 0)
  {
    div_t dv = div(static_cast<int>(w), static_cast<int>(sizeof(T)));

    if(dv.quot > 0)
    {
      cw += dv.quot;
      advanceWritePos(dv.quot);
    }

    _writeShift = dv.rem;
  }

  return(cw);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::read(T *buf, size_t count) throw()
{
  if(_readShift)
    return(0);

  size_t left = count, n, f, r = 0;

  if((_readPos > _writePos) || (_avail == this->_size))
  {
    f = (_end - _readPos);
    n = std::min(left, f);

    std::memcpy(static_cast<void *>(buf), static_cast<const void *>(_readPos),
                (n * sizeof(T)));

    r += n;
    left -= n;
    buf += n;

    advanceReadPos(n);
  }

  if(left > 0)
  {
    f = (_writePos - _readPos);

    if(f > 0)
    {
      n = std::min(left, f);

      std::memcpy(static_cast<void *>(buf),
                  static_cast<const void *>(_readPos),
                  n * sizeof(T));

      r += n;

      advanceReadPos(n);
    }
  }

  return(r);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::read(Buffer<T>& buffer,
                                  size_t count /* = 0 */) throw()
{
  if(_readShift)
    return(0);

  if(count >= buffer.getRemaining())
    return(0); // not enough space in buffer
  else if(count == 0)
    count = buffer.getRemaining();

  T *buf = buffer.getPointer();
  size_t w = read(buf, count);
  buffer.bump(w);

  return(w);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::read(Stream& stream, size_t count /* = 0 */)
  throw(IOException)
{
  size_t r = 0;

  MemoryBlock iov[2];
  int iol = 1;

  if((count == 0) || (count > _avail))
    count = _avail;

  if(count == 0)
    return(0);

  iov[0].setBase(reinterpret_cast<byte_t *>(_readPos) + _readShift);
  if(_readPos < _writePos)
    iov[0].setSize(std::min(count, (static_cast<size_t>(_writePos - _readPos)
                                    * sizeof(T)) - _readShift));
  else
  {
    iov[0].setSize(std::min(count, (static_cast<size_t>(_end - _readPos)
                                    * sizeof(T)) - _readShift));

    count -= (_end - _readPos);

    if(count > 0)
    {
      size_t f = (_writePos - this->_data);
      if(f > 0)
      {
        ++iol;
        iov[1].setBase(reinterpret_cast<byte_t *>(this->_data));
        iov[1].setSize(std::min(count, f) * sizeof(T));
      }
    }
  }

  size_t cw = 0;
  r = stream.write(iov, iol);

  if(_readShift)
  {
    size_t part = (sizeof(T) - _readShift);
    if(r >= part)
    {
      advanceReadPos(1);
      ++cw;
      r -= part;
    }
  }

  div_t dv = div(static_cast<int>(r), static_cast<int>(sizeof(T)));

  if(r > 0)
  {
    advanceReadPos(dv.quot);
    cw += dv.quot;
    _readShift = dv.rem;
  }

  return(cw);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::fill(const T &value, size_t count) throw()
{
  if(_writeShift)
    return(0);

  if(count > getFree())
    count = getFree();

  if(count == 0)
    return(0);

  T *ptr = _writePos;
  for(uint_t n = 0; n < count; ++n)
  {
    *ptr = value;
    if(++ptr == this->_end)
      ptr = this->_data;
  }

  advanceWritePos(count);

  return(count);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::peek(const T &value, size_t maxlen, bool &found,
                                 bool resetPeek /* = true */) throw()
{
  if(_avail == 0)
    return(0);

  size_t count = 0;
  found = false;

  if(resetPeek)
    _peekPos = _readPos;

  size_t i = maxlen;

  if(_peekPos < _writePos)
  {
    for(; _peekPos < _writePos; ++_peekPos)
    {
      ++count;
      --_peekAvail;

      if(*_peekPos == value)
      {
        found = true;
        break;
      }

      if(i-- == 0)
        return(count); // reached scan limit
    }
  }
  else // wraparound
  {
    for(; _peekPos < _end; ++_peekPos)
    {
      ++count;
      --_peekAvail;

      if(*_peekPos == value)
      {
        found = true;
        break;
      }

      if(i-- == 0)
        return(count);
    }

    if(! found)
    {
      for(_peekPos = this->_data; _peekPos < _writePos; ++_peekPos)
      {
        ++count;
        --_peekAvail;

        if(*_peekPos == value)
        {
          found = true;
          break;
        }

        if(i-- == 0)
          return(count);
      }
    }
  }

  return(count);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::getReadExtent() const throw()
{
  if((_readPos > _writePos) || ((_readPos == _writePos) && _avail))
    return(_end - _readPos);
  else if(_readPos < _writePos)
    return(_writePos - _readPos);
  else
    return(0);
}

/*
 */

template <typename T>
  size_t CircularBuffer<T>::getWriteExtent() const throw()
{
  if((_writePos > _readPos) || ((_writePos == _readPos) && ! _avail))
    return(_end - _writePos);
  else if(_writePos < _readPos)
    return(_readPos - _writePos);
  else
    return(0);
}

/*
 */

template <typename T>
  T * CircularBuffer<T>::advanceReadPos(size_t count) throw()
{
  if(count <= _avail)
  {
    _readPos += count;
    if(_readPos >= _end)
      _readPos -= this->_size;

    _avail -= count;
    _readShift = 0;

    if(_peekAvail > _avail)
    {
      // read position overtook peek position

      _peekAvail = _avail;
      _peekPos = _readPos;
    }
  }

  return(_readPos);
}

/*
 */

template <typename T>
  T * CircularBuffer<T>::advanceWritePos(size_t count) throw()
{
  if(count <= getFree())
  {
    _writePos += count;
    if(_writePos >= _end)
      _writePos -= this->_size;

    _avail += count;
    _peekAvail += count;
    _writeShift = 0;
  }

  return(_writePos);
}

/*
 */

template <typename T>
  T * CircularBuffer<T>::advancePeekPos(size_t count) throw()
{
  // peekPos can't go beyond writePos

  if(count <= _peekAvail)
  {
    _peekPos += count;
    _peekAvail -= count;
    if(_peekPos >= this->_end)
      _peekPos -= this->_size;
  }

  return(_peekPos);
}

/*
 */

template <typename T>
  void CircularBuffer<T>::resetPeekPos() throw()
{
  _peekPos = _readPos;
  _peekAvail = _avail;
}

#endif // __ccxx_CircularBufferImpl_hxx

/* end of header file */
