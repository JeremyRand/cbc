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

#ifndef __ccxx_BasicChar_hxx
#define __ccxx_BasicChar_hxx

#include <commonc++/Common.h++>
#include <commonc++/CharTraits.h++>

namespace ccxx {

/** A class representing a character.
 *
 * @author Mark Lindner
 */

template<typename T> class BasicChar
{
  public:

  /** Constructor. */
  BasicChar(T c = BasicCharTraits<T>::nul())
    : _char(c)
  { }

  /** Copy constructor. */
  BasicChar(const BasicChar &other)
    : _char(other._char)
  { }

  /** Destructor. */
  virtual ~BasicChar() throw() { }

  /** Test if the character is alphabetic. */
  inline bool isAlpha() const throw()
  { return(BasicCharTraits<T>::isAlpha(_char)); }

  /** Test if the character is alphanumeric. */
  inline bool isAlphaNumeric() const throw()
  { return(isAlpha() || isDigit()); }

  /** Test if the character is whitespace. */
  inline bool isWhitespace() const throw()
  { return(BasicCharTraits<T>::isWhitespace(_char)); }

  /** Test if the character is printable. */
  inline bool isPrintable() const throw()
  { return(BasicCharTraits<T>::isPrintable(_char)); }

  /** Test if the character is graphical. */
  inline bool isGraphical() const throw()
  { return(BasicCharTraits<T>::isGraphical(_char)); }

  /** Test if the character is NUL. */
  inline bool isNul() const throw()
  { return(BasicCharTraits<T>::isNul(_char)); }

  /** Test if the character is a control character. */
  inline bool isControl() const throw()
  { return(BasicCharTraits<T>::isControl(_char)); }

  /** Test if the character is a decimal digit. */
  inline bool isDigit() const throw()
  { return(BasicCharTraits<T>::isDigit(_char)); }

  /** Test if the character is a hexadecimal digit. */
  bool isHexDigit() const throw()
  { return(BasicCharTraits<T>::isHexDigit(_char)); }

  /** Test if the character is lowercase. */
  bool isLowerCase() const throw()
  { return(BasicCharTraits<T>::isLowerCase(_char)); }

  /** Test if the character is uppercase. */
  bool isUpperCase() const throw()
  { return(BasicCharTraits<T>::isUpperCase(_char)); }

  /** Test if the character is low (7-bit) ASCII. */
  bool isASCII() const throw()
  { return((_char | 0x80) == 0); }

  /** Test if the character is high (8-bit) ASCII. */
  bool isHighASCII() const throw()
  { return((_char & 0x80) != 0); }

  /** Return the uppercase form of this character. */
  BasicChar<T> upperCase() const throw()
  { return(BasicChar<T>(BasicCharTraits<T>::toUpperCase(_char))); }

  /** Return the lowercase form of this character. */
  BasicChar<T> lowerCase() const throw()
  { return(BasicChar<T>(BasicCharTraits<T>::toLowerCase(_char))); }

  /** Convert the character to uppercase. */
  inline BasicChar<T>& toUpperCase() throw()
  {
    _char = BasicCharTraits<T>::toUpperCase(_char);
    return(*this);
  }

  /** Convert the character to lowercase. */
  inline BasicChar<T>& toLowerCase() throw()
  {
    _char = BasicCharTraits<T>::toLowerCase(_char);
    return(*this);
  }

  /** Increment operator (prefix). */
  inline BasicChar<T>& operator++() throw()
  {
    ++_char;
    return(*this);
  }

  /** Increment operator (postfix). */
  inline BasicChar<T> operator++(int) throw()
  { return(BasicChar<T>(_char++)); }

  /** Decrement operator (prefix). */
  inline BasicChar<T>& operator--() throw()
  {
    --_char;
    return(*this);
  }

  /** Decrement operator (postfix). */
  inline BasicChar<T> operator--(int) throw()
  { return(BasicChar<T>(_char--)); }

  /** Assignment operator. */
  BasicChar<T>& operator=(T other) throw()
  {
    _char = other;
    return(*this);
  }

  /** Assignment operator. */
  BasicChar<T>& operator=(int other) throw()
  {
    _char = static_cast<T>(other);
    return(*this);
  }

  bool operator==(T other) const throw()
  { return(_char == other); }

  bool operator!=(T other) const throw()
  { return(_char != other); }

  bool operator<(T other) const throw()
  { return(_char < other); }

  bool operator>(T other) const throw()
  { return(_char > other); }

  bool operator<=(T other) const throw()
  { return(_char <= other); }

  bool operator>=(T other) const throw()
  { return(_char >= other); }

  bool operator==(const BasicChar<T> &other) const throw()
  { return(_char == other._char); }

  bool operator!=(const BasicChar<T> &other) const throw()
  { return(_char != other._char); }

  bool operator<(const BasicChar<T> &other) const throw()
  { return(_char < other._char); }

  bool operator>(const BasicChar<T> &other) const throw()
  { return(_char > other._char); }

  bool operator<=(const BasicChar<T> &other) const throw()
  { return(_char <= other._char); }

  bool operator>=(const BasicChar<T> &other) const throw()
  { return(_char >= other._char); }

  /** Not operator.
   *
   * @return <b>true</b> if the character is NUL, <b>false</b>
   * otherwise.
   */
  inline bool operator!() const throw()
  { return(! isNul()); }

  /** Cast operator. */
  inline operator T() const throw()
  { return(toCharType()); }

  /** Return the <code>char</code> for this character. */
  inline T toCharType() const throw()
  { return(_char); }

  /** The end of file (EOF) character. */
  static COMMONCPP_API const BasicChar<T> EndOfFile;

  /** The bell character. */
  static COMMONCPP_API const BasicChar<T> Bell;

  /** The backspace character. */
  static COMMONCPP_API const BasicChar<T> Backspace;

  /** The tab character. */
  static COMMONCPP_API const BasicChar<T> Tab;

  /** The newline character. */
  static COMMONCPP_API const BasicChar<T> NewLine;

  /** The vertical tab character. */
  static COMMONCPP_API const BasicChar<T> VTab;

  /** The form feed character. */
  static COMMONCPP_API const BasicChar<T> FormFeed;

  /** The carriage return character. */
  static COMMONCPP_API const BasicChar<T> Return;

  /** The escape character. */
  static COMMONCPP_API const BasicChar<T> Escape;

  /** The space character. */
  static COMMONCPP_API const BasicChar<T> Space;

  /** The delete character. */
  static COMMONCPP_API const BasicChar<T> Delete;

  private:

  char _char;
};

template<typename T> inline bool operator==(T c1, const BasicChar<T> &c2)
{ return(c1 == c2.toCharType()); }

template<typename T> inline bool operator!=(T c1, const BasicChar<T> &c2)
{ return(c1 != c2.toChar()); }

template<typename T> inline bool operator<(T c1, const BasicChar<T> &c2)
{ return(c1 < c2.toCharType()); }

template<typename T> inline bool operator<=(T c1, const BasicChar<T> &c2)
{ return(c1 <= c2.toCharType()); }

template<typename T> inline bool operator>(T c1, const BasicChar<T> &c2)
{ return(c1 > c2.toCharType()); }

template<typename T> inline bool operator>=(T c1, const BasicChar<T> &c2)
{ return(c1 >= c2.toCharType()); }

typedef BasicChar<char> Char;
typedef BasicChar<wchar_t> WChar;

}; // namespace ccxx

#endif // __ccxx_BasicChar_hxx

/* end of header file */
