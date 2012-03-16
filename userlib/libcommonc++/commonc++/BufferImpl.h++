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

#ifndef __ccxx_BufferImpl_hxx
#define __ccxx_BufferImpl_hxx

#ifndef __ccxx_Buffer_hxx
#error "Do not include this header directly from application code!"
#endif

#ifdef min
#undef min
#endif

/*
 */

template<typename T>
  Buffer<T>::Buffer(size_t size)
    : AbstractBuffer<T>(size)
{
  clear();
}

/*
 */

template<typename T>
  Buffer<T>::~Buffer() throw()
{
}

/*
 */

template<typename T>
  void Buffer<T>::clear() throw()
{
  AbstractBuffer<T>::clear();

  _limit = this->_size;
  _pos = 0;
}

/*
 */

template<typename T>
  void Buffer<T>::flip() throw()
{
  _limit = _pos;
  _pos = 0;
}

/*
 */

template<typename T>
  void Buffer<T>::rewind() throw()
{
  _pos = 0;
}

/*
 */

template<typename T>
  void Buffer<T>::setLimit(size_t limit) throw()
{
  _limit = std::min(limit, this->_size);
}

/*
 */

template<typename T>
  void Buffer<T>::setPosition(size_t pos) throw()
{
  _pos = std::min(pos, _limit);
}

/*
 */

template<typename T>
  size_t Buffer<T>::bump(size_t delta) throw()
{
  _pos = std::min(_pos + delta, _limit);
  return(_pos);
}

/*
 */

template<typename T>
  size_t Buffer<T>::skip(size_t delta) throw()
{
  size_t newpos = std::min(_pos + delta, _limit);

  if(newpos > _pos)
  {
    std::memset(getPointer(), 0, ((newpos - _pos) * sizeof(T)));
    _pos = newpos;
  }

  return(_pos);
}

/*
 */

template<typename T>
  T& Buffer<T>::operator[](int index) throw(OutOfBoundsException)
{
  if((index < 0) || (static_cast<uint_t>(index) >= _limit))
    throw OutOfBoundsException();

  return(*(this->_data + index));
}

/*
 */

template<typename T>
  T Buffer<T>::operator[](int index) const throw(OutOfBoundsException)
{
  if((index < 0) || (static_cast<uint_t>(index) >= _limit))
    throw OutOfBoundsException();

  return(*(this->_data + index));
}

/*
 */

template<typename T>
  bool Buffer<T>::put(const T& item) throw()
{
  if(getRemaining() < 1)
    return(false);

  std::memcpy(getPointer(), &item, sizeof(T));
  bump(1);

  return(true);
}

/*
 */

template<typename T>
  void Buffer<T>::fill(const T& item, size_t count /* = 0 */) throw()
{
  size_t newpos = (count == 0 ? _limit : std::min(_pos + count, _limit));

  while(_pos < newpos)
    put(item);
}

/*
 */

template<typename T>
  bool Buffer<T>::get(T *item) throw()
{
  if(getRemaining() < 1)
    return(false);

  std::memcpy(item, getPointer(), sizeof(T));
  bump(1);

  return(true);
}

/*
 */

template<typename T>
  bool Buffer<T>::put(const T * items, size_t count) throw()
{
  if(getRemaining() < count)
    return(false);

  std::memcpy(getPointer(), items, (sizeof(T) * count));
  bump(count);

  return(true);
}

/*
 */

template<typename T>
  bool Buffer<T>::get(T * items, size_t count) throw()
{
  if(getRemaining() < count)
    return(false);

  std::memcpy(items, getPointer(), (sizeof(T) * count));
  bump(count);

  return(true);
}

/*
 */

template<typename T>
  int Buffer<T>::peek(const T& item) const throw()
{
  const T *t = (this->_data + this->_pos);
  int max = this->_limit - this->_pos;
  for(int i = 0; i < max; ++i, ++t)
  {
    if(*t == item)
      return(i);
  }

  return(-1);
}

#endif // __ccxx_BufferImpl_hxx

/* end of header file */
