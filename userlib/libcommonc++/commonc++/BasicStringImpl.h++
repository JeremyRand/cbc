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

#ifndef __ccxx_BasicStringImpl_hxx
#define __ccxx_BasicStringImpl_hxx

#ifndef __ccxx_BasicString_hxx
#error "Do not include this header directly from application code!"
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

/*
 */

template<typename T>
  const uint_t BasicString<T>::END = BasicCharTraits<T>::END;

template<typename T>
  const BasicString<T> BasicString<T>::null = BasicCharTraits<T>::null();

template<typename T>
  const BasicString<T> BasicString<T>::empty = BasicCharTraits<T>::empty();

template<typename T>
  const T *BasicString<T>::whitespaceChars
  = BasicCharTraits<T>::whitespace();

template<typename T>
  const BasicStringVec<T> BasicStringVec<T>::emptyVec;

/*
 */

template<typename T>
  BasicString<T>::BasicString()
    : _buf(new StringBuf())
{
}

/*
 */

template<typename T>
  BasicString<T>::BasicString(T *str, size_t offset /* = 0 */,
                              size_t length /* = 0 */, bool copy /* = true */)
    : _buf(new StringBuf(str, offset, length, copy))
{
}

/*
 */

template<typename T>
  BasicString<T>::BasicString(const T *other, size_t offset /* = 0 */,
                              size_t length /* = 0 */)
    : _buf(new StringBuf(other, offset, length))
{
}

/*
 */

template<typename T>
  BasicString<T>::BasicString(const BasicString<T> &other)
    : _buf(NULL)
{
  if(&other == &BasicString<T>::null)
    _buf = new StringBuf();
  else if(other._buf->_refs == -1)
    _buf = new StringBuf(*other._buf, other._buf->_length);
  else
  {
    ++(other._buf->_refs);
    _buf = other._buf;
  }
}

/*
 */

template<typename T>
  BasicString<T>::BasicString(const byte_t *data, size_t length,
                              const char *encoding)
    : _buf(new StringBuf())
{
  this->_encode(data, length, encoding);
}

/*
 */

template<typename T>
  void  BasicString<T>::_encode(const byte_t *data, size_t length,
                                const char *encoding)
{
  CharsetConverter conv(encoding, BasicCharTraits<T>::encoding());
  if(conv.isValid())
  {
    size_t inBytesLeft = length;
    const byte_t *inBuf = data;
    size_t total = 0;

    for(size_t curLen = length;; curLen += (curLen >> 1))
    {
      _makeCopy(curLen + 1);

      size_t outBytesLeft = (curLen * sizeof(T)) - total;
      byte_t *outBuf = reinterpret_cast<byte_t *>(_buf->_data) + total;

      size_t prevOutBytesLeft = outBytesLeft;

      size_t r = conv.convert(&inBuf, &inBytesLeft, &outBuf, &outBytesLeft);
      total += (prevOutBytesLeft - outBytesLeft);

      if(r != (size_t)-1)
        break;

      if(errno != E2BIG)
        break;
    }

    _buf->_length = total / sizeof(T);
    *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
  }
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::operator=(const T *other)
{
  _release();

  _buf = new StringBuf(other);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::operator=(const BasicString<T> &other)
{
  if(&other != this)
  {
    _release();

    if(&other == &BasicString<T>::null)
      _buf = new StringBuf();
    else
    {
      if(other._buf->_refs == -1) // unshareable
        _buf = new StringBuf(other._buf->_data, 0, other._buf->_length);
      else
      {
        ++(other._buf->_refs);
        _buf = other._buf;
      }
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>::~BasicString() throw()
{
  _release();
}

/*
 */

template<typename T>
  bool BasicString<T>::endsWith(const BasicString<T> &str,
                                bool ignoreCase /* = false */) const throw()
{
  size_t len = str.getLength();

  if(_buf->_length < len)
    return(false);

  if(len == 0)
    return(true);

  int r = BasicCharTraits<T>::compare(_buf->_data + _buf->_length - len,
                                      str._buf->_data,
                                      ignoreCase);

  return(r == 0);
}

/*
 */

template<typename T>
  bool BasicString<T>::endsWith(T c, bool ignoreCase /* = false */) const
  throw()
{
  if(isEmpty())
    return(false);

  if(ignoreCase)
    return(BasicCharTraits<T>::toLowerCase(*(_buf->_data + _buf->_length - 1))
           == BasicCharTraits<T>::toLowerCase(c));
  else
    return(*(_buf->_data + _buf->_length - 1) == c);
}

/*
 */

template<typename T>
  bool BasicString<T>::endsWithAnyOf(const T *chars,
                                     bool ignoreCase /* = false */) const
  throw()
{
  if(isEmpty())
    return(false);

  T c = *(_buf->_data + _buf->_length - 1);
  bool matched = false;

  for(const T *p = chars; *p; ++p)
  {
    matched = (BasicCharTraits<T>::compare(*p, c, ignoreCase) == 0);
    if(matched)
      break;
  }

  return(matched);
}

/*
 */

template<typename T>
  bool BasicString<T>::startsWith(const BasicString<T> &str,
                                  bool ignoreCase /* = false */) const throw()
{
  size_t len = str.getLength();

  if(_buf->_length < len)
    return(false);

  if(len == 0)
    return(true);

  return(BasicCharTraits<T>::compare(_buf->_data, str._buf->_data, len,
                                     ignoreCase) == 0);
}

/*
 */

template<typename T>
  bool BasicString<T>::startsWith(T c, bool ignoreCase /* = false */) const
  throw()
{
  if(isEmpty())
    return(false);

  return(BasicCharTraits<T>::compare(*(_buf->_data), c, ignoreCase) == 0);
}

/*
 */

template<typename T>
  bool BasicString<T>::startsWithAnyOf(const T *chars,
                                       bool ignoreCase /* = false */) const
  throw()
{
  if(isEmpty())
    return(false);

  T c = *(_buf->_data);
  bool matched = false;

  for(const T *p = chars; *p; ++p)
  {
    matched = (BasicCharTraits<T>::compare(*p, c, ignoreCase) == 0);
    if(matched)
      break;
  }

  return(matched);
}

/*
 */

template<typename T>
  size_t BasicString<T>::getLength() const throw()
{
  return(_buf == NULL ? 0 : _buf->_length);
}

/*
 */

template<typename T>
  void BasicString<T>::setLength(uint_t length)
{
  if(length < _buf->_length)
  {
    _makeCopy(length + 1);

    _buf->_length = length;
    *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
  }
}

/*
 */

template<typename T>
  void BasicString<T>::reserve(uint_t capacity)
{
  _buf->reserve(capacity + 1);
}

/*
 */

template<typename T>
  size_t BasicString<T>::getCapacity() const throw()
{
  return(_buf->_capacity);
}

/*
 */

template<typename T>
  void BasicString<T>::compact()
{
  // TODO
}

/*
 */

template<typename T>
  bool BasicString<T>::isEmpty() const throw()
{
  return((_buf->_data == NULL) || (_buf->_length == 0));
}

/*
 */

template<typename T>
  bool BasicString<T>::isNull() const throw()
{
  return(_buf->_data == NULL);
}

/*
 */

template<typename T>
  bool BasicString<T>::isAlphaNumeric() const throw()
{
  if(isEmpty())
    return(false);

  for(const T *p = _buf->_data; *p; p++)
  {
    if(! BasicCharTraits<T>::isAlphaNumeric(*p))
      return(false);
  }

  return(true);
}

/*
 */

template<typename T>
  bool BasicString<T>::isInteger() const throw()
{
  if(isEmpty())
    return(false);

  bool first = true;
  for(const T *p = _buf->_data; *p; p++)
  {
    if(! (BasicCharTraits<T>::isDigit(*p)
          || (first && BasicCharTraits<T>::isSign(*p))))
      return(false);

    first = false;
  }

  return(true);
}

/*
 */

template<typename T>
  int BasicString<T>::compareTo(const BasicString<T> &other,
                                bool ignoreCase /* = false */) const throw()
{
  if(_buf == other._buf)
    return(0);

  // null is "less than" non-null

  if(isNull())
    return(other.isNull() ? 0 : -1);

  if(other.isNull())
    return(1);

  return(BasicCharTraits<T>::compare(_buf->_data, other._buf->_data,
                                     ignoreCase));
}

/*
 */

template<typename T>
  int BasicString<T>::compareTo(const T *other, bool ignoreCase /* = false */)
  const throw()
{
  // null is "less than" non-null

  if(isNull())
    return(other == NULL ? 0 : -1);

  if(_buf->_data == other)
    return(0);

  if(other == NULL)
    return(1);

  return(BasicCharTraits<T>::compare(_buf->_data, other, ignoreCase));
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::trim(const T *chars /* = whitespaceChars */,
                                       bool begin /* = true */,
                                       bool end /* = true */)
{
  if(! isEmpty())
  {
    T *p = _buf->_data, *q = p + _buf->_length - 1;

    // trim from beginning

    if(begin)
      for(; *p && BasicCharTraits<T>::find(chars, *p); ++p) {}

    // trim from end

    if(end)
      for(; BasicCharTraits<T>::find(chars, *q) && (q >= p); --q) {}

    size_t nl = (q - p + 1);

    if(nl != _buf->_length)
    {
      _makeCopy(nl + 1);

      BasicCharTraits<T>::move(_buf->_data, p, nl);
      _buf->_length = nl;
      *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::chop(T c)
{
  if((! isEmpty()) && (! BasicCharTraits<T>::isNul(c)))
  {
    T *p = BasicCharTraits<T>::find(_buf->_data, c);
    if(p != NULL)
    {
      size_t nl = (p - _buf->_data);

      _makeCopy(nl + 1);

      _buf->_length = nl;
      *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::reverseChop(T c)
{
  if(! isEmpty() && (! BasicCharTraits<T>::isNul(c)))
  {
    T *p = BasicCharTraits<T>::findLast(_buf->_data, c);
    if(p != NULL)
    {
      size_t nl = (p - _buf->_data);

      _makeCopy(nl + 1);

      _buf->_length = nl;
      *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::replace(T a, T b)
{
  if((! isEmpty()) && (! BasicCharTraits<T>::isNul(a)) && (a != b))
  {
    T *p = BasicCharTraits<T>::find(_buf->_data, a);
    if(p != NULL)
    {
      _makeCopy(_buf->_capacity);

      p = BasicCharTraits<T>::find(_buf->_data, a);

      for(T *q = p; *q; q++)
      {
        if(*q == a)
          *q = b;
      }
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::replace(const T *a, const T *b)
{
  if(! isEmpty() && (a != NULL) && (b != NULL))
  {
    T *p = BasicCharTraits<T>::find(_buf->_data, a);

    if(p)
    {
      size_t l = BasicCharTraits<T>::length(a);
      BasicString<T> copy;
      T *q = _buf->_data;

      do
      {
        if(p != q)
          copy.append(q, (p - q));
        q = p + l;
        copy.append(b);
      }
      while((p = BasicCharTraits<T>::find(q, a)) != NULL);

      if(! BasicCharTraits<T>::isNul(*q))
        copy.append(q);

      *this = copy;
    }
  }
  return(*this);
}

/*
 */

template<typename T>
  T& BasicString<T>::operator[](int index) throw(OutOfBoundsException)
{
  if(isNull() || (index < 0))
    throw OutOfBoundsException();

  size_t ind = static_cast<size_t>(index);

  bool extend = false;
  if(ind >= _buf->_length)
  {
    ind = _buf->_length;
    extend = true;
  }

  _makeCopy(std::max(_buf->_length, static_cast<size_t>(ind + 1)), true);

  if(extend)
  {
    ++_buf->_length;
    *(_buf->_data  + _buf->_length) = BasicCharTraits<T>::nul();
  }

  return(*(_buf->_data + ind));
}

/*
 */

template<typename T>
  BasicChar<T> BasicString<T>::operator[](int index) const
  throw(OutOfBoundsException)
{
  if(isNull() || (index < 0))
    throw OutOfBoundsException();

  size_t ind = static_cast<size_t>(index);

  if(ind >= _buf->_length)
    throw OutOfBoundsException();

  return(BasicChar<T>(*(_buf->_data + ind)));
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::printf(const T *format, ...)
{
  if((format == NULL) || (BasicCharTraits<T>::isNul(*format)))
    return(*this);

  va_list vp;

  // make an educated guess as to the initial buffer size we need: 1.5 times
  // the length of the format string

  size_t bufsz = BasicCharTraits<T>::length(format);
  bufsz += (bufsz / 2);

  int total = 0;

  for(;; bufsz *= 2)
  {
    _makeCopy(_buf->_length + bufsz + 1);

    va_start(vp, format);
    int r = BasicCharTraits<T>::vprintf(_buf->_data + _buf->_length, bufsz,
                                        format, vp);
    va_end(vp);

    if(r >= 0)
    {
      total = r;
      break; // written successfully
    }
  }

  _buf->_length += total;
  *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();

  return(*this);
}

/*
 */

template<typename T>
  T *BasicString<T>::_makeGap(uint_t index, size_t gaplen)
{
  if((index > _buf->_length) || (index == END))
    index = static_cast<uint_t>(_buf->_length);

  _makeCopy(_buf->_length + gaplen + 1);

  size_t movelen = _buf->_length - index;
  if(movelen > 0)
    BasicCharTraits<T>::move(_buf->_data + index + gaplen,
                             _buf->_data + index,
                             movelen);

  _buf->_length += gaplen;
  *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();

  return(_buf->_data + index);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::insert(T c, uint_t index /* = 0 */)
{
  if(! BasicCharTraits<T>::isNul(c))
  {
    T *gap = _makeGap(index, 1);
    *gap = c;
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::insert(const T *str, uint_t index /* = 0 */,
                                         size_t length /* = 0 */)
{
  if(str != NULL)
  {
    size_t len = BasicCharTraits<T>::length(str);
    if((length == 0) || (length > len))
      length = len;

    if(length > 0)
    {
      T *gap = _makeGap(index, length);
      BasicCharTraits<T>::copy(gap, str, length);
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::insert(const BasicString<T> &str,
                                         uint_t index /* = 0 */,
                                       size_t length /* = 0 */)
{
  if(! str.isNull())
    insert(str._buf->_data, index, length);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::remove(uint_t fromIndex,
                                         uint_t toIndex /* = END */)
{
  if(! isEmpty())
  {
    if(toIndex == END)
      toIndex = static_cast<uint_t>(_buf->_length) + 1;
    if(fromIndex == END)
      fromIndex = static_cast<uint_t>(_buf->_length) + 1;

    if(toIndex > fromIndex)
    {
      size_t len = toIndex - fromIndex - 1;

      BasicCharTraits<T>::move(_buf->_data + fromIndex, _buf->_data + toIndex,
                               (_buf->_length - toIndex + 1));

      _buf->_length -= len;
      *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
    }
  }

  return(*this);
}

/*
 */

template<typename T>
  uint_t BasicString<T>::hash(uint_t modulo /* = 256 */) const throw()
{
  uint32_t hash = 0;

  if(! isEmpty())
    hash = Hash::SDBMHash(reinterpret_cast<const byte_t *>(_buf->_data),
                          _buf->_length * sizeof(T));

  return(hash % modulo);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::substring(uint_t fromIndex,
                                           uint_t toIndex /* = END */) const
{
  if(isNull() || (fromIndex == toIndex))
    return(BasicString<T>::empty);

  if(fromIndex > toIndex)
    fromIndex = toIndex;

  BasicString<T> substr(_buf->_data, fromIndex, (toIndex - fromIndex));

  return(substr);
}

/*
 */

template<typename T>
  int BasicString<T>::indexOf(T c, uint_t fromIndex /* = 0 */) const throw()
{
  if(isNull() || (fromIndex >= _buf->_length) || BasicCharTraits<T>::isNul(c))
    return(-1);

  int index = fromIndex;

  for(const T *p = _buf->_data + fromIndex; *p; p++, index++)
    if(*p == c)
      return(index);

  return(-1);
}

/*
 */

template<typename T>
  int BasicString<T>::indexOfAnyOf(const T *chars, uint_t fromIndex /* = 0 */)
  const throw()
{
  if(isNull() || (fromIndex >= _buf->_length) || (chars == NULL))
    return(-1);

  int index = fromIndex;

  for(const T *p = _buf->_data + fromIndex; *p; p++, index++)
    if(BasicCharTraits<T>::find(chars, *p) != NULL)
      return(index);

  return(-1);
}

/*
 */

template<typename T>
  int BasicString<T>::indexOf(const T *str, uint_t fromIndex /* = 0 */)
  const throw()
{
  if(isEmpty() || (str == NULL) || (fromIndex >= _buf->_length))
    return(-1);

  const T *p = BasicCharTraits<T>::find(_buf->_data + fromIndex, str);

  return(p ? static_cast<int>(p - _buf->_data) : -1);
}

/*
 */

template<typename T>
  int BasicString<T>::lastIndexOf(T c, uint_t fromIndex /* = END */) const
  throw()
{
  if(isEmpty() || BasicCharTraits<T>::isNul(c))
    return(-1);

  if((fromIndex >= _buf->_length) || (fromIndex == END))
    fromIndex = _buf->_length - 1;

  int index = fromIndex;

  for(const T *p = _buf->_data + fromIndex; index >= 0; index--, p--)
    if(*p == c)
      return(index);

  return(-1);
}

/*
 */

template<typename T>
  int BasicString<T>::lastIndexOfAnyOf(const T *chars,
                                       uint_t fromIndex /* = END */) const
  throw()
{
  if(isEmpty() || (chars == NULL))
    return(-1);

  if((fromIndex >= _buf->_length) || (fromIndex == END))
    fromIndex = _buf->_length - 1;

  int index = fromIndex;

  for(const T *p = _buf->_data + fromIndex; index >= 0; index--, p--)
    if(BasicCharTraits<T>::find(chars, *p) != NULL)
      return(index);

  return(-1);
}

/*
 */

template<typename T>
  int BasicString<T>::lastIndexOf(const T *str,
                                  uint_t fromIndex /* = END */) const throw()
{
  if(isEmpty() || (str == NULL))
    return(-1);

  size_t slen = BasicCharTraits<T>::length(str);

  if(_buf->_length < slen)
    return(-1);

  if((fromIndex >= _buf->_length) || (fromIndex == END))
    fromIndex = _buf->_length - 1;

  T *p = BasicCharTraits<T>::findLast(_buf->_data, str, fromIndex);

  return(p ? (p - _buf->_data) : -1);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::nextToken(
    uint_t &fromIndex, const T *delim /* = whitespaceChars */) const
{
  if(isNull() || (fromIndex >= _buf->_length) || (delim == NULL))
    return(BasicString<T>::null);

  T *p, *q;

  for(p = _buf->_data + fromIndex; *p; p++)
  {
    if(! BasicCharTraits<T>::find(delim, *p))
      break;
  }

  if(BasicCharTraits<T>::isNul(*p))
    return(BasicString<T>::null);

  for(q = p + 1; *q; q++)
  {
    if(BasicCharTraits<T>::find(delim, *q))
      break;
  }

  fromIndex = static_cast<uint_t>(q - _buf->_data);

  BasicString<T> tok(p, 0, static_cast<size_t>(q - p));

  return(tok);
}

/*
 */

template<typename T>
  uint_t BasicString<T>::split(BasicStringVec<T> &tokens,
                               const T *delim /* = whitespaceChars */,
                               uint_t limit /* = 0 */) const
{
  uint_t index = 0, count = 0;

  for(;;)
  {
    BasicString<T> s = nextToken(index, delim);
    if(s.isNull())
      break;

    tokens << s;
    ++count;

    if(limit && (count == limit))
      break;
  }

  return(count);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::toLowerCase()
{
  if(! isEmpty())
  {
    _makeCopy(_buf->_capacity);

    for(T *p = _buf->_data; *p; p++)
      *p = BasicCharTraits<T>::toLowerCase(*p);
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::toUpperCase()
{
  if(! isEmpty())
  {
    _makeCopy(_buf->_capacity);

    for(T *p = _buf->_data; *p; p++)
      *p = BasicCharTraits<T>::toUpperCase(*p);
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::fill(T c, size_t length)
{
  if((! BasicCharTraits<T>::isNul(c)) && (length > 0))
  {
    _makeCopy(length + 1);

    size_t n = length;
    for(T *p = _buf->_data; n; ++p, --n)
      *p = c;

    _buf->_length = length;
    *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::pad(T c, size_t length)
{
  if((! BasicCharTraits<T>::isNul(c)) && (length > 0))
  {
    _makeCopy(_buf->_length + length + 1);

    size_t n = length;
    for(T *p = _buf->_data + _buf->_length; n; ++p, --n)
      *p = c;

    _buf->_length += length;
    *(_buf->_data + _buf->_length) = BasicCharTraits<T>::nul();
  }

  return(*this);
}

/*
 */

template<typename T>
  bool BasicString<T>::toBool(uint_t index /* = 0 */) const throw()
{
  if(isNull() || (index >= _buf->_length))
    return(false);

  return(! BasicCharTraits<T>::compare(_buf->_data + index,
                                       BasicCharTraits<T>::trueRep(), true));
}

/*
 */

template<typename T>
  int BasicString<T>::toInt(uint_t index /* = 0 */, uint_t base /* = 10 */)
  const throw()
{
  if((base < 2) || (base > 36) || isNull() || (index >= _buf->_length))
    return(0);

  return(BasicCharTraits<T>::toLong(_buf->_data + index, NULL, base));
}

/*
 */

template<typename T>
  uint_t BasicString<T>::toUInt(uint_t index /* = 0 */,
                                uint_t base /* = 10 */) const throw()
{
  int v = toInt(index, base);
  return(static_cast<uint_t>(v < 0 ? 0 : v));
}

/*
 */

template<typename T>
  int64_t BasicString<T>::toInt64(uint_t index /* = 0 */,
                                  uint_t base /* = 10 */) const throw()
{
  if((base < 2) || (base > 36) || isNull() || (index >= _buf->_length))
    return(INT64_CONST(0));

  return(BasicCharTraits<T>::toLongLong(_buf->_data + index, NULL, base));
}

/*
 */

template<typename T>
  uint64_t BasicString<T>::toUInt64(uint_t index /* = 0 */,
                                    uint_t base /* = 10 */) const throw()
{
  if((base < 2) || (base > 36) || isNull() || (index >= _buf->_length))
    return(UINT64_CONST(0));

  return(BasicCharTraits<T>::toULongLong(_buf->_data + index, NULL, base));
}

/*
 */

template<typename T>
  float BasicString<T>::toFloat(uint_t index /* = 0 */) const throw()
{
  if(isNull() || (index >= _buf->_length))
    return(0.0f);

  return(BasicCharTraits<T>::toFloat(_buf->_data + index, NULL));
}

/*
 */

template<typename T>
  double BasicString<T>::toDouble(uint_t index /* = 0 */) const throw()
{
  if(isNull() || (index >= _buf->_length))
    return(0.0);

  return(BasicCharTraits<T>::toDouble(_buf->_data + index, NULL));
}

/*
 */

template<typename T>
  BasicString<T>::operator const void *() const throw()
{
  return(_buf == NULL ? NULL : _buf->_data);
}

/*
 */

template<typename T>
  const T * BasicString<T>::c_str() const throw()
{
  return(_buf == NULL ? NULL : _buf->_data);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(bool v)
{
  return(insert((v ? BasicCharTraits<T>::trueRep()
                 : BasicCharTraits<T>::falseRep()), END));
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(int val)
{
  T buf[16];

  BasicCharTraits<T>::printf(buf, CCXX_LENGTHOF(buf),
                             BasicCharTraits<T>::intFormat(), val);
  append(buf);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(int val, uint_t base)
{
  if((base >= 2) && (base <= 36))
  {
    T buf[34];
    bool neg = false;
    int i = 33;
    buf[i] = BasicCharTraits<T>::nul();

    if(val < 0)
    {
      neg = true;
      val = -val;

      if(val < 0) // int overflow
      {
        buf[--i] = BasicCharTraits<T>::digits()[-(val + base) % base];
        val = -(val / base);
      }
    }

    do
    {
      buf[--i] = BasicCharTraits<T>::digits()[val % base];
      val /= base;
    }
    while(val > 0);

    if(neg)
      buf[--i] = BasicCharTraits<T>::minus();

    append(buf);
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(uint_t val)
{
  T buf[16];

  BasicCharTraits<T>::printf(buf, CCXX_LENGTHOF(buf),
                             BasicCharTraits<T>::uintFormat(), val);
  append(buf);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(uint_t val, uint_t base)
{
  if((base >= 2) && (base <= 36))
  {
    T buf[34];
    int i = 33;
    buf[i] = BasicCharTraits<T>::nul();

    do
    {
      buf[--i] = BasicCharTraits<T>::digits()[val % base];
      val /= base;
    }
    while(val > 0);

    append(buf);
  }

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(int64_t val)
{
  T buf[32];

  BasicCharTraits<T>::printf(buf, CCXX_LENGTHOF(buf),
                             BasicCharTraits<T>::int64Format(), val);
  append(buf);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(uint64_t val)
{
  T buf[32];

  BasicCharTraits<T>::printf(buf, CCXX_LENGTHOF(buf),
                             BasicCharTraits<T>::uint64Format(), val);
  append(buf);

  return(*this);
}

/*
 */

template<typename T>
  BasicString<T>& BasicString<T>::append(double val,
                                         uint_t precision /* = 6 */)
{
  T buf[64];

  BasicCharTraits<T>::printf(buf, CCXX_LENGTHOF(buf),
                             BasicCharTraits<T>::doubleFormat(), precision,
                             val);
  append(buf);

  return(*this);
}

/*
 */

template<typename T>
 Blob BasicString<T>::getBytes(const char *encoding, size_t length /* = 0 */,
                               size_t maxEncodedLength /* = 0 */) const
{
  Blob blob;

  if(! isNull())
  {
    if(length == 0)
      length = _buf->_length + 1;
    else
      length = std::min(length, _buf->_length + 1);

    CharsetConverter conv(BasicCharTraits<T>::encoding(), encoding);
    if(conv.isValid())
    {
      size_t inBytesLeft = length * sizeof(T);
      const byte_t *inBuf = reinterpret_cast<const byte_t *>(_buf->_data);
      size_t total = 0;
      size_t curLen = inBytesLeft;

      for(;;)
      {
        if((maxEncodedLength > 0) && (maxEncodedLength < curLen))
          curLen = maxEncodedLength;

        blob.setLength(curLen);

        size_t outBytesLeft = curLen - total;
        byte_t *outBuf = blob.getData() + total;
        size_t prevOutBytesLeft = outBytesLeft;

        size_t r = conv.convert(&inBuf, &inBytesLeft, &outBuf, &outBytesLeft);
        total += (prevOutBytesLeft - outBytesLeft);

        if(r != (size_t)-1)
          break;

        if(errno != E2BIG)
          break;

        if((maxEncodedLength > 0) && (curLen == maxEncodedLength))
          break;

        curLen += (curLen >> 1);
      }

      blob.setLength(total);
    }
  }

  return(blob);
}

/*
 */

template<typename T>
  void BasicString<T>::_release()
{
  if(--(_buf->_refs) <= 0)
    delete _buf;
}

/*
 */

template<typename T>
  void BasicString<T>::_makeCopy(size_t size, bool unshareable /* = false */)
{
  if(_buf->_refs > 1) // and implicitly != -1 (unshareable)
  {
    StringBuf *buf = new StringBuf(*_buf, size);

    if(--(_buf->_refs) < 1)
      delete buf; // in case two threads are doing this at the same time
    else
      _buf = buf;
  }
  else
    _buf->reserve(size);

  _buf->_refs = (unshareable ? -1 : 1);
}

/*
 */

template<typename T>
  BasicString<T>::StringBuf::StringBuf(const StringBuf& other, size_t length)
  : _data(NULL), _capacity(0), _length(0), _refs(1)
{
  reserve(std::max(other._length, length) + 1);

  if(other._data != NULL)
    BasicCharTraits<T>::rawCopy(_data, other._data, other._length);

  _length = other._length;
  *(_data + _length) = BasicCharTraits<T>::nul();
}

/*
 */

template<typename T>
  BasicString<T>::StringBuf::StringBuf(const T *str,
                                       size_t offset /* = 0 */,
                                       size_t length /* = 0 */)
: _data(NULL), _capacity(0), _length(0), _refs(1)
{
  makeCopy(str, offset, length);
}

/*
 */

template<typename T>
  BasicString<T>::StringBuf::StringBuf(T *str /* = NULL */,
                                       size_t offset /* = 0 */,
                                       size_t length /* = 0 */,
                                       bool copy /* = true */)
    : _data(NULL), _capacity(0), _length(0), _refs(1)
{
  if(! copy || (str == NULL))
  {
    _data = str;
    _length = (str == NULL ? 0 : BasicCharTraits<T>::length(str));
    _capacity = (str == NULL ? 0 : _length + 1);
  }
  else
    makeCopy(str, offset, length);
}

/*
 */

template<typename T>
  BasicString<T>::StringBuf::~StringBuf() throw()
{
  delete[] _data;
}

/*
 */

template<typename T>
  void BasicString<T>::StringBuf::makeCopy(const T *str, size_t offset,
                                           size_t length)
{
  if(str != NULL)
  {
    size_t len = BasicCharTraits<T>::length(str);

    if(length == 0)
      length = len - offset;
    else
      length = std::min(length, len - offset);

    reserve(length + 1);
    BasicCharTraits<T>::rawCopy(_data, str + offset, length);
    *(_data + length) = BasicCharTraits<T>::nul();
    _length = length;
  }
}

/*
 */

template<typename T>
  void BasicString<T>::StringBuf::reserve(size_t size)
{
  if(_capacity < size)
  {
    size_t newsz = (_capacity << 1) - (_capacity >> 1); // _capacity * 1.5
    size_t needed = std::max<size_t>(newsz, size);
    size_t newcap = (needed > 0 ? 4 * ((needed - 1) / 4 + 1) : 0);
    T *newdata = (newcap > 0 ? new T[newcap] : NULL);

    if(newdata != NULL)
    {
      if(_data != NULL)
        BasicCharTraits<T>::rawCopy(newdata, _data, _length);
    }

    if(_data != NULL)
      delete[] _data;

    _data = newdata;
    _capacity = newcap;
  }
}

/*
 */

template<typename T>
  void BasicString<T>::StringBuf::compact()
{
  if(_data)
  {
    size_t needed = _length + 1;
    if(_capacity > needed)
    {
      T *newdata = new T[needed];
      BasicCharTraits<T>::rawCopy(newdata, _data, _length);
      newdata[_length] = BasicCharTraits<T>::nul();

      delete[] _data;
      _data = newdata;

      _capacity = needed;
    }
  }
}

/*
 */

template<typename T>
  void BasicString<T>::StringBuf::makeNull()
{
  if(_data)
  {
    delete[] _data;
    _data = NULL;
  }

  _length = 0;
  _capacity = 0;
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::operator+(const BasicString<T> &str)
{
  BasicString<T> s = *this;
  s += str;

  return(s);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::operator+(const T *str)
{
  BasicString<T> s = *this;
  s += str;

  return(s);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::operator+(T c)
{
  BasicString<T> s = *this;
  s += c;

  return(s);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::fromCharArray(const char *str,
                                               size_t length /* = 0 */)
{
  if(length == 0)
    length = CharTraits::length(str);

  BasicString<T> s;
  s._encode(reinterpret_cast<const byte_t *>(str), length * sizeof(char),
            CharTraits::encoding());

  return(s);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::fromUTF8(const char *str)
{
  size_t length = CharTraits::length(str);

  BasicString<T> s;
  s._encode(reinterpret_cast<const byte_t *>(str), length * sizeof(char),
            "UTF-8");

  return(s);
}

/*
 */

template<typename T>
  BasicString<T> BasicString<T>::fromWCharArray(const wchar_t *str,
                                                size_t length /* = 0 */)
{
  if(length == 0)
    length = WCharTraits::length(str);

  BasicString<T> s;
  s._encode(reinterpret_cast<const byte_t *>(str), length * sizeof(wchar_t),
            WCharTraits::encoding());

  return(s);
}

#endif // __ccxx_BasicStringImpl_hxx

/* end of header file */
