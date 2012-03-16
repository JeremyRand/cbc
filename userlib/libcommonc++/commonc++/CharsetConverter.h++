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

#ifndef __ccxx_CharsetConverter_hxx
#define __ccxx_CharsetConverter_hxx

#include <commonc++/Common.h++>

namespace ccxx {

/** Character set converter. This class is simply a thin wrapper around
 * the <b>libiconv</b> character set conversion library.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API CharsetConverter
{
  public:

  /** Construct a new CharsetConverter for converting characters from one
   * encoding to another. After the object is constructed, a call should be
   * made to isValid() to determine if the conversion is valid. See the
   * <b>libiconv</b> documentation for a list of supported encodings.
   *
   * @param fromCode The source encoding.
   * @param toCode The target encoding.
   */
  CharsetConverter(const char *fromCode, const char *toCode);

  /** Destructor. */
  ~CharsetConverter();

  /** Determine if the conversion specified in the constructor is valid. */
  inline bool isValid() const throw()
  { return(_iconv != NULL); }

  /** Convert bytes of data from the source encoding to the target
   * encoding. This method may be called repeatedly to convert a large
   * amount of text one block at a time; the shift state is preserved
   * between calls. To begin converting a new
   *
   * @param inBuf The input buffer.
   * @param inBytesLeft The number of input bytes remaining.
   * @param outBuf The output buffer.
   * @param outBytesLeft The number of output bytes remaining.
   * @return The number of characters converted, or <code>(size_t)-1</code>
   * on failure; see <b>iconv(3)</b> for details.
   */
  size_t convert(const byte_t **inBuf, size_t *inBytesLeft, byte_t **outBuf,
                 size_t *outBytesLeft) throw();

  /** Reset the encoder. Sets the conversion state to the initial state. */
  void reset() throw();

  private:

  void *_iconv;
};

}; // namespace ccxx

#endif // __ccxx_CharsetConverter_hxx

/* end of header file */
