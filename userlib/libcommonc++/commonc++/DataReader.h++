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

#ifndef __ccxx_DataReader_hxx
#define __ccxx_DataReader_hxx

#include <commonc++/Blob.h++>
#include <commonc++/DataEncoder.h++>
#include <commonc++/IOException.h++>
#include <commonc++/String.h++>

namespace ccxx {

/** An abstract base class for data readers. A data reader reads raw
 * bytes from an input source and converts them to primitive values
 * such as integers and strings.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API DataReader : public DataEncoder
{
  public:

  /** Destructor */
  virtual ~DataReader() throw();

  /** Read operator. */
  DataReader & operator>>(bool &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(byte_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(byte_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(char &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(char *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int16_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int16_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint16_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint16_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int32_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int32_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint32_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint32_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int64_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(int64_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint64_t &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(uint64_t *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(float &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(float *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(double &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(double *v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(Blob &v) throw(IOException);
  /** Read operator. */
  DataReader & operator>>(String &v) throw(IOException);

  /** Read operator. */
  DataReader & operator>>(const Manipulator &manip) throw(IOException);

  protected:

  /** Constructor. */
  DataReader() throw();

  /** Read raw data from the input source. */
  virtual size_t read(byte_t *buf, size_t count) throw(IOException) = 0;

  private:

  CCXX_COPY_DECLS(DataReader);
};

}; // namespace ccxx

#endif // __ccxx_DataReader_hxx

/* end of header file */
