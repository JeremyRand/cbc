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

#ifndef __ccxx_BasicString_hxx
#define __ccxx_BasicString_hxx

#include <commonc++/Common.h++>
#include <commonc++/AtomicCounter.h++>
#include <commonc++/Blob.h++>
#include <commonc++/Char.h++>
#include <commonc++/CharsetConverter.h++>
#include <commonc++/CharTraits.h++>
#include <commonc++/Hash.h++>
#include <commonc++/OutOfBoundsException.h++>

#include <string>
#include <vector>
#include <cerrno>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif // _MSC_VER

namespace ccxx {

template<typename T> class BasicStringVec; // fwd ref

/** A flexible, reference counted, copy-on-write, thread-safe, nullable
 * string.
 *
 * @author Mark Lindner
 */

template<typename T> class BasicString
{
  public:

  /** Construct a new, empty string. */
  BasicString();

  /** Construct a new string from a C string.
   *
   * @param str The C string to copy or inherit.
   * @param offset The number of characters from the beginning of the string
   * to skip over.
   * @param length The number of characters from the offset to use, or 0 to
   * use all the characters to the end of the string.
   * @param copy If <b>true</b>, the C string is copied into the BasicString;
   * otherwise the C string is inherited and the length parameter is ignored.
   */
  BasicString(T *str, size_t offset = 0, size_t length = 0,
              bool copy = true);

  /** Copy constructor. */
  BasicString(const BasicString<T>& other);

  /** Construct a new string from a C string.
   *
   * @param other The C string to copy.
   * @param offset The number of characters from the beginning of the string
   * to skip over.
   * @param length The number of characters from the offset to copy, or 0 to
   * copy to the end of the string.
   */
  BasicString(const T *other, size_t offset = 0, size_t length = 0);

  /** Construct a new string from a byte array.
   *
   * @param data The byte array.
   * @param length The length of the array.
   * @param encoding The character encoding of the data in the array.
   */
  BasicString(const byte_t *data, size_t length, const char *encoding);

  /** Destructor. */
  virtual ~BasicString() throw();

  /** Test if the string ends with the given string.
   *
   * @param str The other string.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if this string ends with <i>str</i>, <b>false</b>
   * otherwise.
   */
  bool endsWith(const BasicString<T> &str, bool ignoreCase = false)
    const throw();

  /** Test if the string ends with the given character.
   *
   * @param c The character.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if the string ends with <i>c</i>, <b>false</b>
   * otherwise.
   */
  bool endsWith(T c, bool ignoreCase = false) const throw();

  /** Test if the string ends with any character in a given set of
   * characters.
   *
   * @param chars The set of characters.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if the string ends with one of the characters
   * in the set, <b>false</b> otherwise.
   */
  bool endsWithAnyOf(const T *chars, bool ignoreCase = false) const throw();

  /** Test if the string starts with the given string.
   *
   * @param str The other string.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if the BasicString begins with <i>str</i>,
   * <b>false</b> otherwise.
   */
  bool startsWith(const BasicString<T> &str, bool ignoreCase = false)
    const throw();

  /** Test if the string starts with the given character.
   *
   * @param c The character.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if the string begins with <i>c</i>, <b>false</b>
   * otherwise.
   */
  bool startsWith(T c, bool ignoreCase = false) const throw();

  /** Test if the string starts with any character in a given set of
   * characters.
   *
   * @param chars The set of characters.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-insensitive.
   * @return <b>true</b> if the string starts with one of the
   * characters in the set, <b>false</b> otherwise.
   */
  bool startsWithAnyOf(const T *chars, bool ignoreCase = false) const throw();

  /** Get the length of the string.
   *
   * @return The length, in characters, or 0 if the BasicString is
   * <b>null</b>.
   */
  size_t getLength() const throw();

  /** Get the length of the string.
   *
   * @return The length, in characters, or 0 if the string is <b>null</b>.
   */
  inline size_t length() const throw()
  { return(getLength()); }

  /** Set the length of the string. Allows the string to be
   * truncated to an arbitrary length.
   *
   * @param length The new length. The new length must be less than the
   * current length; otherwise the string is left unmodified.
   */
  void setLength(uint_t length);

  /** Reserve space in the string for the given number of characters. If the
   * new capacity is less than or equal to the current capacity, the string
   * is not modified.
   *
   * @param capacity The new capacity.
   */
  void reserve(uint_t capacity);

  /** Get the current capacity of the string, in characters. This value
   * takes into account the space required for the final NUL terminator.
   */
  size_t getCapacity() const throw();

  /** Compact the string so that it occupies the minimum amount of memory
   * required.
   */
  void compact();

  /** Clear the string. Sets the length of the string to 0.
   *
   * @return The string.
   */
  inline BasicString<T>& clear()
  {
    setLength(0);
    return(*this);
  }

  /** Test if the string is empty.
   *
   * @return <b>true</b> if the string is <b>null</b> or has length 0,
   * <b>false</b> otherwise.
   */
  bool isEmpty() const throw();

  /** Test if the string is <b>null</b>.
   *
   * @return <b>true</b> if the string is <b>null</b>, <b>false</b>
   * otherwise.
   */
  bool isNull() const throw();

  /** Test if the string consists solely of alphanumeric characters
   * (A-Z, a-z, and 0-9).
   *
   * @return <b>true</b> if the string is alphanumeric, <b>false</b>
   * otherwise.
   */
  bool isAlphaNumeric() const throw();

  /** Test if the string holds an integer. For example, "123", "+5", and
   * "-123" are integers.
   *
   * @return <b>true</b> if the string is an integer, <b>false</b>
   * otherwise.
   */
  bool isInteger() const throw();

  /** Compare the string to another.
   *
   * @param other The string to compare to.
   * @param ignoreCase A flag indicating whether the compare should be
   * case insensitive.
   * @return A negative value if the string is lexicographically "less"
   * than <i>other</i>, a positive value if it is "greater" than
   * <i>other</i>, and 0 if they are "equal".
   */
  int compareTo(const T *other, bool ignoreCase = false) const throw();

  /** Compare the string to another.
   *
   * @param other The string to compare to.
   * @param ignoreCase A flag indicating whether the compare should be
   * case insensitive.
   * @return A negative value if the string is lexicographically "less"
   * than <i>other</i>, a positive value if it is "greater" than
   * <i>other</i>, and 0 if they are "equal".
   */
  int compareTo(const BasicString<T> &other, bool ignoreCase = false) const
    throw();

  /** Trim leading and trailing characters from the string.
   *
   * @return The string.
   */
  BasicString<T>& trim(const T *chars = whitespaceChars, bool begin = true,
                       bool end = true);

  /** Trim characters from the beginning of the string.
   *
   * @param chars The set of characters to trim (the set of whitespace
   * characters by default).
   */
  inline BasicString<T>& trimBegin(const T *chars = whitespaceChars)
  { return(trim(chars, true, false)); }

  /** Trim characters from the end of the string.
   *
   * @param chars The set of characters to trim (the set of whitespace
   * characters by default).
   */
  inline BasicString<T>& trimEnd(const T *chars = whitespaceChars)
  { return(trim(chars, false, true)); }

  /** Truncate the string at the first occurrence of the given character.
   *
   * @param c The character at which to truncate.
   * @return The string.
   */
  BasicString<T>& chop(T c);

  /** Truncate the string at the last occurrence of the given character.
   *
   * @param c The character at which to truncate.
   * @return The string.
   */
  BasicString<T>& reverseChop(T c);

  /** Replace all occurences of one character with another.
   *
   * @param a The character to replace.
   * @param b The replacement character.
   * @return The string.
   */
  BasicString<T>& replace(T a, T b);

  /** Replace all occurences of one substring with another.
   *
   * @param a The substring to replace.
   * @param b The replacement substring.
   * @return The string.
   */
  inline BasicString<T>& replace(const BasicString<T> &a,
                                 const BasicString<T> &b)
  { return(replace(a.c_str(), b.c_str())); }

  /** Replace all occurences of one substring with another.
   *
   * @param a The substring to replace.
   * @param b The replacement substring.
   * @return The string.
   */
  BasicString<T>& replace(const T *a, const T *b);

  /** Append a C-style formatted string to the string. The functionality
   * is equivalent to that of <b>sprintf()</b>.
   *
   * @param format The format string.
   */
  BasicString<T>& printf(const T *format, ...);

  /** Insert a character into the string.
   *
   * @param c The character to insert.
   * @param index The index at which to insert the character. If the index
   * is out of range, the character is appended to the end of the string.
   */
  BasicString<T>& insert(T c, uint_t index = 0);

  /** Insert a string into this string.
   *
   * @param str The string to insert.
   * @param index The index at which to insert the string. If the index
   * is out of range, the string is appended to the end of this string.
   * @param length The maximum number of characters from <i>str</i> to
   * insert.
   */
  BasicString<T>& insert(const T *str, uint_t index = 0, size_t length = 0);

  /** Insert a string into this string.
   *
   * @param str The string to insert.
   * @param index The index at which to insert the string. If the index
   * is out of range, the string is appended to the end of this string.
   * @param length The maximum number of characters from <i>str</i> to
   * insert.
   */
  BasicString<T>& insert(const BasicString<T>& str, uint_t index = 0,
                         size_t length = 0);

  /** Insert a string into this string.
   *
   * @param str The string to insert.
   * @param index The index at which to insert the string. If the index
   * is out of range, the string is appended to the end of this string.
   * @param length The maximum number of characters from <i>str</i> to
   * insert.
   */
  inline BasicString<T>& insert(const std::basic_string<T> &str,
                                uint_t index = 0, size_t length = 0)
  { return(insert(str.c_str(), index, length)); }

  /** Append a string representation of a boolean value to the end of
   * the string.
   *
   * @param v The value to append.
   */
  BasicString<T>& append(bool v);

  /** Append a string representation of an integer value onto the end of
   * the string.
   *
   * @param val The value to append.
   */
  BasicString<T>& append(int val);

  /** Append a string representation of an integer value in the
   * given base onto the end of the string.
   *
   * @param val The value to append.
   * @param base The numerical base in which to output the value. The valid
   * range is 2 - 36, inclusive.
   */
  BasicString<T>& append(int val, uint_t base);

  /** Append a string representation of an unsigned integer value
   * onto the end of the string.
   *
   * @param val The value to append.
   */
  BasicString<T>& append(uint_t val);

  /** Append a string representation of an unsigned integer value in the
   * given base onto the end of the string.
   *
   * @param val The value to append.
   * @param base The numerical base in which to output the value. The valid
   * range is 2 - 36, inclusive.
   */
  BasicString<T>& append(uint_t val, uint_t base);

  /** Append a string representation of a long long (64-bit) integer onto
   * the end of the string.
   *
   * @param val The value to append.
   */
  BasicString<T>& append(int64_t val);

  /** Append a string representation of double-precision floating point
   * value onto the end of the string.
   *
   * @param val The value to append.
   * @param precision The precision (the number of digits to include after
   * the radix).
   */
  BasicString<T>& append(double val, uint_t precision = 6);

  /** Append a string representation of an unsigned long long
   * (64-bit) integer onto the end of the string.
   *
   * @param val The value to append.
   */
  BasicString<T>& append(uint64_t val);

  /** Append a character to the end of the string.
   *
   * @param c The character to append.
   */
  inline BasicString<T>& append(T c)
  { return(insert(c, END)); }

  /** Append a string to the end of the string.
   *
   * @param str The character to append.
   * @param length The maximum number of characters from <i>str</i> to
   * append, or 0 to append all of <i>str</i>.
   */
  inline BasicString<T>& append(const T *str, size_t length = 0)
  { return(insert(str, END, length)); }

  /** Append a string to the end of the string.
   *
   * @param str The character to append.
   * @param length The maximum number of characters from <i>str</i> to
   * append, or 0 to append all of <i>str</i>.
   */
  inline BasicString<T>& append(const BasicString<T> &str,
                                size_t length = 0)
  { return(insert(str, END, length)); }

  /** Append a string to the end of the string.
   *
   * @param str The character to append.
   * @param length The maximum number of characters from <i>str</i> to
   * append, or 0 to append all of <i>str</i>.
   */
  inline BasicString<T>& append(const std::basic_string<T> &str,
                                size_t length = 0)
  { return(insert(str.c_str(), END, length)); }

  /** Prepend a character to the beginning of the string.
   *
   * @param c The character to prepend.
   */
  inline BasicString<T>& prepend(T c)
  { return(insert(c, 0)); }

  /** Prepend a string to the beginning of the string.
   *
   * @param str The character to prepend.
   * @param length The maximum number of characters from <i>str</i> to
   * prepend, or 0 to prepend all of <i>str</i>.
   */
  inline BasicString<T>& prepend(const T *str, size_t length = 0)
  { return(insert(str, 0, length)); }

  /** Prepend a string to the beginning of the string.
   *
   * @param str The character to prepend.
   * @param length The maximum number of characters from <i>str</i> to
   * prepend, or 0 to prepend all of <i>str</i>.
   */
  inline BasicString<T>& prepend(const BasicString<T>& str, size_t length = 0)
  { return(insert(str, 0, length)); }

  /** Prepend a string to the beginning of the string.
   *
   * @param str The character to prepend.
   * @param length The maximum number of characters from <i>str</i> to
   * prepend, or 0 to prepend all of <i>str</i>.
   */
  inline BasicString<T>& prepend(const std::basic_string<T> &str,
                                 size_t length = 0)
  { return(insert(str.c_str(), 0, length)); }

  /** Remove a portion of the string.
   *
   * @param fromIndex The start index of the range to remove.
   * @param toIndex The end index of the range to remove, or <i>END</i>
   * to remove to the end of the string.
   */
  BasicString<T>& remove(uint_t fromIndex, uint_t toIndex = END);

  /** Compute a hashcode for the string.
   *
   * @param modulo The desired range for the hashcode.
   * @return A hashcode in the range [0, <i>modulo</i>).
   */
  uint_t hash(uint_t modulo = 256) const throw();

  /** Extract a substring from the string.
   *
   * @param fromIndex The start index of the substring.
   * @param toIndex The end index of the substring, or <i>END</i> to indicate
   * the end of the string.
   * @return The substring as a new string. If fromIndex and toIndex are
   * equal, an empty string is returned.
   */
  BasicString<T> substring(uint_t fromIndex, uint_t toIndex = END) const;

  /** Find the first occurrence of a given character in the string.
   *
   * @param c The character to search for.
   * @param fromIndex The start index from which to begin searching.
   * @return The index of the first occurrence of the character, or -1
   * if the character was not found.
   */
  int indexOf(T c, uint_t fromIndex = 0) const throw();

  /** Find the first occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The start index from which to begin searching.
   * @return The index of the first occurrence of the string, or -1
   * if the string was not found.
   */
  int indexOf(const T *str, uint_t fromIndex = 0) const throw();

  /** Find the first occurrence in this string of any character in a set
   * of characters.
   *
   * @param chars The set of characters search for.
   * @param fromIndex The start index from which to begin searching.
   * @return The index of the first occurrence of any character in the set,
   * or -1 if no occurrence was not found.
   */
  int indexOfAnyOf(const T *chars, uint_t fromIndex = 0) const throw();

  /** Find the first occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The start index from which to begin searching.
   * @return The index of the first occurrence of the string, or -1
   * if the string was not found.
   */
  inline int indexOf(const BasicString<T>& str, uint_t fromIndex = 0) const
    throw()
  { return(indexOf(str.c_str(), fromIndex)); }

  /** Find the first occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The start index from which to begin searching.
   * @return The index of the first occurrence of the string, or -1
   * if the string was not found.
   */
  inline int indexOf(const std::basic_string<T> &str, uint_t fromIndex = 0)
    const throw()
  { return(indexOf(str.c_str(), fromIndex)); }

  /** Find the last occurrence of a given character in this string.
   *
   * @param c The character to search for.
   * @param fromIndex The index from which to begin searching backwards,
   * or <b>END</b> to start at the end of the string.
   * @return The index of the last occurrence of the character, or -1
   * if the character was not found.
   */
  int lastIndexOf(T c, uint_t fromIndex = END) const throw();

  /** Find the last occurrence in the string of any character in a set
   * of characters.
   *
   * @param chars The set of characters search for.
   * @param fromIndex The index from which to begin searching backwards,
   * or <b>END</b> to start at the end of the string.
   * @return The index of the last occurrence of any character in the set,
   * or -1 if no occurrence was not found.
   */
  int lastIndexOfAnyOf(const T *chars, uint_t fromIndex = END) const throw();

  /** Find the last occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The index from which to begin searching backwards,
   * or <b>END</b> to start at the end of the string.
   * @return The index of the last occurrence of the string, or -1
   * if the string was not found.
   */
  int lastIndexOf(const T *str, uint_t fromIndex = END) const throw();

  /** Find the last occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The index from which to begin searching backwards,
   * or <b>END</b> to start at the end of the string.
   * @return The index of the last occurrence of the string, or -1
   * if the string was not found.
   */
  inline int lastIndexOf(const BasicString<T> &str, uint_t fromIndex = END)
    const throw()
  { return(lastIndexOf(str.c_str(), fromIndex)); }

  /** Find the last occurrence of a given string in this string.
   *
   * @param str The string to search for.
   * @param fromIndex The index from which to begin searching backwards,
   * or <b>END</b> to start at the end of the string.
   * @return The index of the last occurrence of the string, or -1
   * if the string was not found.
   */
  inline int lastIndexOf(const std::basic_string<T> &str,
                         uint_t fromIndex = END) const throw()
  { return(lastIndexOf(str.c_str(), fromIndex)); }

  /** Determine if the string contains a given character.
   *
   * @param c The character to search for.
   * @return <b>true</b> if the string contains at least one occurrence of
   * the character, <b>false</b> otherwise.
   */
  inline bool contains(T c) const throw()
  { return(indexOf(c) >= 0); }

  /** Determine if the string contains any character in a given set of
   * characters.
   *
   * @param chars The set of characters.
   * @return <b>true</b> if the string contains at least one occurrence of
   * any character in the set, <b>false</b> otherwise.
   */
  inline bool containsAnyOf(const T *chars) const throw()
  { return(indexOfAnyOf(chars) >= 0); }

  /** Determine if the string contains a given substring.
   *
   * @param str The substring to search for.
   * @return <b>true</b> if the string contains at least one occurrence of
   * the substring, <b>false</b> otherwise.
   */
  inline bool contains(const T *str) const throw()
  { return(indexOf(str) >= 0); }

  /** Determine if the string contains a given substring.
   *
   * @param str The substring to search for.
   * @return <b>true</b> if the string contains at least one occurrence of
   * the substring, <b>false</b> otherwise.
   */
  inline bool contains(const BasicString<T> &str) const throw()
  { return(indexOf(str) >= 0); }

  /** Determine if the string contains a given substring.
   *
   * @param str The substring to search for.
   * @return <b>true</b> if the string contains at least one occurrence of
   * the substring, <b>false</b> otherwise.
   */
  inline bool contains(const std::basic_string<T> &str) const throw()
  { return(indexOf(str) >= 0); }

  /** Extract a token from the string.
   *
   * @param fromIndex The start index at which to begin searching for a
   * token. Will be modified on return to point to one character past the
   * extracted token.
   * @param delim The set of possible token delimiter characters (the set
   * of whitespace characters by default).
   * @return The extracted token, or <b>String::null</b> if the end of the
   * string has been reached.
   */
  BasicString<T> nextToken(uint_t &fromIndex,
                           const T *delim = whitespaceChars) const;

  /** Split the string into a series of tokens, without modifying the
   * string.
   *
   * @param tokens The vector in which to place the extracted tokens. If
   * a non-empty vector is supplied, the tokens are added to the end of
   * the vector.
   * @param delim The set of possible token delimiter characters (the set
   * of whitespace characters by default).
   * @param limit The maximum number of tokens to extract, or 0 for no
   * limit (the default).
   * @return The number of tokens extracted.
   */
  uint_t split(BasicStringVec<T> &tokens, const T *delim = whitespaceChars,
               uint_t limit = 0) const;

  /** Convert the string to lower case. */
  BasicString<T>& toLowerCase();

  /** Convert the string to upper case. */
  BasicString<T>& toUpperCase();

  /** Fill the string with a character.
   *
   * @param c The fill character.
   * @param length The number of characters to fill.
   */
  BasicString<T>& fill(T c, size_t length);

  /** Pad the string with a character.
   *
   * @param c The pad character.
   * @param length The number of characters to pad.
   */
  BasicString<T>& pad(T c, size_t length);

  /** Parse a boolean value from the string.
   *
   * @param index The index to begin parsing at.
   * @return The parsed value.
   */
  bool toBool(uint_t index = 0) const throw();

  /** Parse an integer from the string.
   *
   * @param index The index to begin parsing at.
   * @param base The base (2 - 36).
   * @return The parsed value.
   */
  int toInt(uint_t index = 0, uint_t base = 10) const throw();

  /** Parse an unsigned integer from the string.
   *
   * @param index The index to begin parsing at.
   * @param base The base (2 - 36).
   * @return The parsed value.
   */
  uint_t toUInt(uint_t index = 0, uint_t base = 10) const throw();

  /** Parse a 64-bit integer from the string.
   *
   * @param index The index to begin parsing at.
   * @param base The base (2 - 36).
   * @return The parsed value.
   */
  int64_t toInt64(uint_t index = 0, uint_t base = 10) const throw();

  /** Parse an unsigned 64-bit integer from the string.
   *
   * @param index The index to begin parsing at.
   * @param base The base (2 - 36).
   * @return The parsed value.
   */
  uint64_t toUInt64(uint_t index = 0, uint_t base = 10) const throw();

  /** Parse a floating point value from the string.
   *
   * @param index The index to begin parsing at.
   * @return The parsed value.
   */
  float toFloat(uint_t index = 0) const throw();

  /** Parse a double precision floating point value from the string.
   *
   * @param index The index to begin parsing at.
   * @return The parsed value.
   */
  double toDouble(uint_t index = 0) const throw();

  /** Pointer cast operator. */
  operator const void *() const throw();

  /** Return the contents of the string as an immutable C string. */
  const T * c_str() const throw();

  /** Encode the string using the specified character encoding, returning
   * the results as a Blob.
   *
   * @param encoding The character encoding to use.
   * @param length The number of characters to encode, or 0 to encode the
   * entire string.
   * @param maxEncodedLength The maximum number of bytes to encode, or 0
   * if there is no limit. If a limit is specified, no more complete
   * characters will be encoded than would fit in the specified number of
   * bytes.
   * @return The encoded string, as a Blob.
   */
  Blob getBytes(const char *encoding, size_t length = 0,
                size_t maxEncodedLength = 0) const;

  /** Assignment operator. */
  BasicString<T>& operator=(const T *other);

  /** Assignment operator. */
  BasicString<T>& operator=(const BasicString<T> &other);

  /** Assignment operator. */
  inline BasicString<T>& operator=(const std::basic_string<T> &other)
  { return(operator=(other.c_str())); }

  /** %Array index operator. Obtain a reference to the character at the given
   * offset in the string.
   *
   * @param index The index.
   * @return A reference to the character at the specified index.
   * @throw OutOfBoundsException If <i>index</I> is out of range.
   */
  T& operator[](int index) throw(OutOfBoundsException);

  /** %Array index operator. Obtain a copy of the character at the given
   * offset in the string.
   *
   * @param index The index.
   * @return The character at the specified index, as a Char.
   * @throw OutOfBoundsException If <i>index</I> is out of range.
   */
  BasicChar<T> operator[](int index) const throw(OutOfBoundsException);

  /** Substring operator. Extract a substring from the string.
   *
   * @param fromIndex The start index of the substring.
   * @param toIndex The end index of the substring, or <i>END</i> to indicate
   * the end of the string.
   * @return The substring as a new string. If fromIndex and toIndex are
   * equal, an empty string is returned.
   */
  inline BasicString<T> operator()(uint_t fromIndex, uint_t toIndex = END)
    const
  { return(substring(fromIndex, toIndex)); }

  /** Unary NOT operator.
   *
   * @return <b>true</b> if the string is <b>null</b>, <b>false</b>
   * otherwise.
   */
  inline bool operator!() const throw()
  { return(isNull()); }

  /** Append a character onto the end of the string.
   *
   * @param c The character to append.
   */
  inline BasicString<T>& operator+=(T c)
  { return(append(c)); }

  /** Append a string onto the end of this string.
   *
   * @param str The string to append.
   */
  inline BasicString<T>& operator+=(const T *str)
  { return(append(str)); }

  /** Append a string onto the end of this string.
   *
   * @param str The string to append.
   */
  inline BasicString<T>& operator+=(const BasicString<T> &str)
  { return(append(str)); }

  /** Append a string onto the end of this string.
   *
   * @param str The string to append.
   */
  inline BasicString<T>& operator+=(const std::basic_string<T> &str)
  { return(append(str.c_str())); }

  /** Append a string representation of a boolean value onto the end of
   * the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(bool val)
  { return(append(val)); }

  /** Append a string representation of an integer value onto the end of
   * the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(int val)
  { return(append(val)); }

  /** Append a string representation of an unsigned integer value onto the
   * end of the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(uint_t val)
  { return(append(val)); }

  /** Append a string representation of double-precision floating point
   * value onto the end of the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(double val)
  { return(append(val)); }

  /** Append a string representation of a long long (64-bit) integer onto
   * the end of the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(int64_t val)
  { return(append(val)); }

  /** Append a string representation of an unsigned long long
   * (64-bit) integer onto the end of the string.
   *
   * @param val The value to append.
   */
  inline BasicString<T>& operator+=(uint64_t val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(const T *str)
  { return(append(str)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(const BasicString<T> &str)
  { return(append(str)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(const std::basic_string<T> &str)
  { return(append(str)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(bool val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(T c)
  { return(append(c)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(int val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(uint_t val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(int64_t val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(uint64_t val)
  { return(append(val)); }

  /** Append operator. */
  inline BasicString<T>& operator<<(double val)
  { return(append(val)); }

  /** Prepend operator. */
  inline BasicString<T>& operator^=(T c)
  { return(prepend(c)); }

  /** Prepend operator. */
  inline BasicString<T>& operator^=(const T *str)
  { return(prepend(str)); }

  /** Prepend operator. */
  inline BasicString<T>& operator^=(const BasicString<T> &str)
  { return(prepend(str)); }

  /** Prepend operator. */
  inline BasicString<T>& operator^=(const std::basic_string<T> &str)
  { return(prepend(str)); }

  BasicString<T> operator+(const BasicString<T> &str);
  BasicString<T> operator+(const T *str);
  BasicString<T> operator+(T c);

  /** Create a string from the given character array. */
  static BasicString<T> fromCharArray(const char *str, size_t length = 0);

  /** Create a string from the given UTF-8 character array. */
  static BasicString<T> fromUTF8(const char *str);

  /** Create a string from the given wide character array. */
  static BasicString<T> fromWCharArray(const wchar_t *str, size_t length = 0);

  /** A pseudo-index indicating the end of the string. */
  static const uint_t END;

  /** The <b>null</b> string. */
  static const BasicString<T> null;

  /** The empty string. */
  static const BasicString<T> empty;

  /** The set of whitespace delimiter characters. */
  static const T *whitespaceChars;

  private:

  class StringBuf
  {
    friend class BasicString<T>;

    protected:

    T *_data;
    size_t _capacity;
    size_t _length;
    AtomicCounter _refs;

    StringBuf(const StringBuf& other, size_t length);
    StringBuf(T *str = NULL, size_t offset = 0, size_t length = 0,
              bool copy = true);
    StringBuf(const T *str, size_t offset = 0, size_t length = 0);
    ~StringBuf() throw();

    void reserve(size_t size);
    void compact();
    void makeNull();
    void makeCopy(const T *str, size_t offset, size_t length);

    private:

    CCXX_COPY_DECLS(StringBuf);
  };

  StringBuf *_buf;

  void _release();
  void _makeCopy(size_t size, bool unshareable = false);
  T *_makeGap(uint_t index, size_t gaplen);
  void _encode(const byte_t *data, size_t length, const char *encoding);
};

/** A String vector. A trivial extension of
 * <b>std::vector<BasicString<T> ></b>.
 *
 * @author Mark Lindner
 */

template<typename T> class BasicStringVec
  : public std::vector<BasicString<T> >
{
  public:

  BasicStringVec()
  { }

  /** Append operator. Pushes a string onto the end (back) of the vector. */
  inline BasicStringVec<T>& operator<<(BasicString<T> s)
  { push_back(s); return(*this); }

  /** An empty BasicStringVec. */
  static const BasicStringVec<T> emptyVec;
};

template<typename T>
  inline bool operator==(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) == 0); }

template<typename T>
  inline bool operator==(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) == 0); }

template<typename T>
  inline bool operator==(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) == 0); }

template<typename T>
  inline bool operator!=(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) != 0); }

template<typename T>
  inline bool operator!=(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) != 0); }

template<typename T>
  inline bool operator!=(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) != 0); }

template<typename T>
  inline bool operator<(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) < 0); }

template<typename T>
  inline bool operator<(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) < 0); }

template<typename T>
  inline bool operator<(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) > 0); }

template<typename T>
  inline bool operator>(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) > 0); }

template<typename T>
  inline bool operator>(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) > 0); }

template<typename T>
  inline bool operator>(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) < 0); }

template<typename T>
  inline bool operator<=(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) <= 0); }

template<typename T>
  inline bool operator<=(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) <= 0); }

template<typename T>
  inline bool operator<=(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) > 0); }

template<typename T>
  inline bool operator>=(const BasicString<T> &s1, const BasicString<T> &s2)
  throw()
{ return(s1.compareTo(s2) >= 0); }

template<typename T>
  inline bool operator>=(const BasicString<T> &s1, const T *s2) throw()
{ return(s1.compareTo(s2) >= 0); }

template<typename T>
  inline bool operator>=(const T *s1, const BasicString<T> &s2) throw()
{ return(s2.compareTo(s1) < 0); }

template<typename T>
  inline BasicString<T> operator+(const T *s1, const BasicString<T> &s2)
{ return(BasicString<T>(s1) + s2); }

template<typename T>
  inline BasicString<T> operator+(T c, const BasicString<T> &s)
{ BasicString<T> t; t += c; t += s; return(t); }

template<typename T>
  inline std::ostream& operator<<(std::ostream& stream,
                                  const BasicString<T> &s)
{ return(stream << (s ? s.c_str() : BasicCharTraits<T>::nullRep())); }

#include <commonc++/BasicStringImpl.h++>

}; // namespace ccxx

#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

#endif // __ccxx_BasicString_hxx

/* end of header file */
