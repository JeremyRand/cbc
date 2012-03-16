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

#ifndef __ccxx_BasicCharTraits_hxx
#define __ccxx_BasicCharTraits_hxx

#include <commonc++/Common.h++>

#include <cstring>
#include <cctype>
#include <cwchar>
#include <cwctype>
#include <cstdarg>
#include <cstdio>

namespace ccxx {

/**
 * Character traits. This class is a collection of functions for performing
 * various low level character-string operations.
 *
 * @author Mark Lindner
 */

template<typename T> struct BasicCharTraits
{
  /** Get the length of a string.
   *
   * @param str The string.
   * @return The length of the string, in characters.
   */
  static size_t length(const T *str) throw();

  /** Copy a string.
   *
   * @param dest The destination string.
   * @param src The source string.
   * @param n The maximum number of characters to copy.
   * @return The destination string.
   */
  static T *copy(T *dest, const T *src, size_t n) throw();

  /** Copy a fixed number of characters. This method differs from copy() in
   * that it copies exactly the number of characters requested, without
   * regard to nul terminators.
   *
   * @param dest The destination string.
   * @param src The source string.
   * @param n The maximum number of characters to copy.
   * @return The destination string.
   */
  static T *rawCopy(T *dest, const T *src, size_t n) throw();

  /** Concatenate a string onto another string.
   *
   * @param dest The destination string.
   * @param src The source string.
   * @param n The maximum number of characters to concatenate.
   * @return The destination string.
   */
  static T *concat(T *dest, const T *src, size_t n) throw();

  /** Copy a string to a possibly overlapping destination string.
   *
   * @param dest The destination string.
   * @param src The source string.
   * @param n The number of characters to copy.
   * @return The destination string.
   */
  static T *move(T *dest, T *src, size_t n) throw();

  /** Compare two strings.
   *
   * @param str1 The first string.
   * @param str2 The second string.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-sensitive.
   * @return A negative value if <code>str1</code> &lt; <code>str2</code>,
   * a positive value if <code>str1</code> &gt; <code>str2</code>, and zero
   * if the two strings are equal.
   */
  static int compare(const T *str1, const T *str2, bool ignoreCase = false)
    throw();

  /** Compare two strings. This method differs from compare() in
   * that it compares exactly the number of characters requested,
   * without regard to nul terminators.
   *
   * @param str1 The first string.
   * @param str2 The second string.
   * @param n The number of characters to compare.
   * @return A negative value if <code>str1</code> &lt; <code>str2</code>,
   * a positive value if <code>str1</code> &gt; <code>str2</code>, and zero
   * if the two strings are equal.
   */
  static int rawCompare(const T *str1, const T *str2, size_t n) throw();

  /** Lexicographically compare two strings based on the collation rules
   * of the current locale.
   *
   * @param str1 The first string.
   * @param str2 The second string.
   * @return A negative value if <code>str1</code> &lt; <code>str2</code>,
   * a positive value if <code>str1</code> &gt; <code>str2</code>, and zero
   * if the two strings are equal.
   */
  static int collate(const T *str1, const T *str2) throw();

  /** Compare two strings.
   *
   * @param str1 The first string.
   * @param str2 The second string.
   * @param n The maximum number of characters to compare.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-sensitive.
   * @return A negative value if <code>str1</code> &lt; <code>str2</code>,
   * a positive value if <code>str1</code> &gt; <code>str2</code>, and zero
   * if the two strings are equal.
   */
  static int compare(const T *str1, const T *str2, size_t n,
                     bool ignoreCase = false) throw();

  /** Compare two characters.
   *
   * @param c1 The first character.
   * @param c2 The second character.
   * @param ignoreCase A flag indicating whether the comparison should be
   * case-sensitive.
   * @return A negative value if <code>c1</code> &lt; <code>c2</code>,
   * a positive value if <code>c1</code> &gt; <code>c2</code>, and zero
   * if the two characters are equal.
   */
  static int compare(T c1, T c2, bool ignoreCase = false) throw();

  /** Calculate the length of the initial segment of a string which
   * consists entirely of characters in the given set.
   *
   * @param str The string.
   * @param accept The set of characters.
   * @return The length of the span.
   */
  static size_t span(const T *str, const T *accept) throw();

  /** Parse a long integer value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @param base The numeric base.
   * @return The parsed value.
   */
  static int32_t toLong(const T *str, T **endptr = NULL, int base = 10)
    throw();

  /** Parse an unsigned long integer value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @param base The numeric base.
   * @return The parsed value.
   */
  static uint32_t toULong(const T *str, T **endptr = NULL, int base = 10)
    throw();

  /** Parse a long long integer value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @param base The numeric base.
   * @return The parsed value.
   */
  static int64_t toLongLong(const T *str, T **endptr = NULL, int base = 10)
    throw();

  /** Parse an unsigned long long integer value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @param base The numeric base.
   * @return The parsed value.
   */
  static uint64_t toULongLong(const T *str, T **endptr = NULL, int base = 10)
    throw();

  /** Parse a floating-point value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @return The parsed value.
   */
  static float toFloat(const T *str, T **endptr = NULL) throw();

  /** Parse a double-precision floating point value from the string.
   *
   * @param str The string.
   * @param endptr A pointer at which to store a pointer to the first
   * non-parsed character.
   * @return The parsed value.
   */
  static double toDouble(const T *str, T **endptr = NULL) throw();

  /** Find the first occurrence of a character in a string.
   *
   * @param str The string.
   * @param c The character.
   * @param len The number of characters in the string to scan, or 0 to
   * scan the entire string (up to the NUL terminator).
   * @return A pointer to the character, or NULL if the character was not
   * found.
   */
  static T *find(T *str, T c, size_t len = 0) throw();

  /** Find the first occurrence of a character in a string.
   *
   * @param str The string.
   * @param c The character.
   * @param len The number of characters in the string to scan, or 0 to
   * scan the entire string (up to the NUL terminator).
   * @return A pointer to the character, or NULL if the character was not
   * found.
   */
  static const T *find(const T *str, T c, size_t len = 0) throw();

  /** Find the last occurrence of a character in a string.
   *
   * @param str The string.
   * @param c The character.
   * @param fromIndex The index to begin searching backwards from.
   * @return A pointer to the character, or NULL if the character was not
   * found.
   */
  static T *findLast(T *str, T c, uint_t fromIndex = END) throw();

  /** Find the last occurrence of a character in a string.
   *
   * @param str The string.
   * @param c The character.
   * @param fromIndex The index to begin searching backwards from.
   * @return A pointer to the character, or NULL if the character was not
   * found.
   */
  static const T *findLast(const T *str, T c, uint_t fromIndex = END) throw();

  /** Find the first occurrence of a substring in a string.
   *
   * @param str The string.
   * @param s The substring.
   * @return A pointer to the substring, or NULL if the substring was not
   * found.
   */
  static T *find(T *str, const T *s) throw();

  /** Find the first occurrence of a substring in a string.
   *
   * @param str The string.
   * @param s The substring.
   * @return A pointer to the substring, or NULL if the substring was not
   * found.
   */
  static const T *find(const T *str, const T *s) throw();

  /** Find the last occurrence of a substring in a string.
   *
   * @param str The string.
   * @param s The substring.
   * @param fromIndex The index to begin searching backwards from.
   * @return A pointer to the substring, or NULL if the substring was not
   * found.
   */
  static T *findLast(T *str, const T *s, uint_t fromIndex = END) throw();

  /** Find the last occurrence of a substring in a string.
   *
   * @param str The string.
   * @param s The substring.
   * @param fromIndex The index to begin searching backwards from.
   * @return A pointer to the substring, or NULL if the substring was not
   * found.
   */
  static const T *findLast(const T *str, const T *s, uint_t fromIndex = END)
    throw();

  /** Write a formatted string.
   *
   * @param str The string to write to.
   * @param len The maximum length of the string.
   * @param format The format string.
   * @return The number of characters written, or -1 if there was not
   * enough space available and/or the output was truncated.
   */
  static int printf(T *str, size_t len, const T *format, ...) throw();

  /** Write a formatted string.
   *
   * @param str The string to write to.
   * @param len The maximum length of the string.
   * @param format The format string.
   * @param vp The varargs list.
   * @return The number of characters written, or -1 if there was not
   * enough space available and/or the output was truncated.
   */
  static int vprintf(T *str, size_t len, const T *format, va_list vp)
    throw();

  /** Return the NUL character. */
  static T nul() throw();

  /** Return the '+' character. */
  static T plus() throw();

  /** Return the '-' character. */
  static T minus() throw();

  /** Return a null string. */
  static const T *null() throw();

  /** Return an empty string. */
  static const T *empty() throw();

  /** Return an array containing the set of whitespace characters. */
  static const T *whitespace() throw();

  /** Return an array containing the set of digits used to represent numbers
   * in base 2 through base 36 ('0' - '9' and 'A' - 'Z').
   */
  static const T *digits() throw();

  /** Return a string representation of the null string. Returns the string
   * "(null)".
   */
  static const T *nullRep() throw();

  /** Return a string representation of the boolean value <b>true</b>.
   * Returns the string "true".
   */
  static const T *trueRep() throw();

  /** Return a string representation of the boolean value <b>false</b>.
   * Returns the string "false".
   */
  static const T *falseRep() throw();

  /** Return the format string for integers. Returns the string "%d". */
  static const T *intFormat() throw();

  /** Return the format string for unsigned integers. Returns the string
   * "%u".
   */
  static const T *uintFormat() throw();

  /** Return the compiler-specific format string for 64-bit integers. */
  static const T *int64Format() throw();

  /** Return the compiler-specific format string for unsigned 64-bit
   * integers.
   */
  static const T *uint64Format() throw();

  /** Return the format string for double-precision floating point numbers
   * with a parameterized precision. Returns the string "%.*f".
   */
  static const T *doubleFormat() throw();

  /** Test if a character is alphabetic. */
  static bool isAlpha(T c) throw();

  /** Test if a character is alphanumeric. */
  static bool isAlphaNumeric(T c) throw();

  /** Test if a character is whitespace. */
  static bool isWhitespace(T c) throw();

  /** Test if a character is printable. */
  static bool isPrintable(T c) throw();

  /** Test if a character is graphical. */
  static bool isGraphical(T c) throw();

  /** Test if a character is Nul. */
  static bool isNul(T c) throw();

  /** Test if a character is a control character. */
  static bool isControl(T c) throw();

  /** Test if a character is a digit. */
  static bool isDigit(T c) throw();

  /** Test if a character is a sign symbol ('+' or '-'). */
  static bool isSign(T c) throw();

  /** Test if a character is a hexadecimal digit. */
  static bool isHexDigit(T c) throw();

  /** Test if a character is lowercase. */
  static bool isLowerCase(T c) throw();

  /** Test if a character is uppercase. */
  static bool isUpperCase(T c) throw();

  /** Get the uppercase equivalent of a character. */
  static T toUpperCase(T c) throw();

  /** Get the lowercase equivalent of a character. */
  static T toLowerCase(T c) throw();

  /** Convert an integer value in the range 0 - 9 to the
   * corresponding digit character '0' - '9'. If the value is
   * outside the acceptable range, the character '0' is returned.
   */
  static T fromInt(int v) throw();

  /** Convert a digit character in the range '0' - '9' to the
   * corresponding integer value 0 - 9. If the character is
   * not a digit, 0 is returned.
   */
  static int toInt(T c) throw();

  /** Get the character encoding used for the internal representation of
   * a BasicString<T>.
   */
  static const char *encoding() throw();

  /** Pseudo-index indicating the end of the string. */
  static const uint_t END = 0xFFFFFFFF;
};

typedef BasicCharTraits<char> CharTraits;
typedef BasicCharTraits<wchar_t> WCharTraits;

}; // namespace ccxx

#endif // __ccxx_BasicCharTraits_hxx

/* end of header file */
