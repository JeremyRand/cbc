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

#ifndef __ccxx_POSIX_hxx
#define __ccxx_POSIX_hxx

#include <commonc++/Common.h++>
#include <commonc++/IOException.h++>
#include <commonc++/Permissions.h++>
#include <commonc++/String.h++>

#include <sys/socket.h>
#include <sys/types.h>
#include <pthread.h>

namespace ccxx {

class COMMONCPP_API POSIX
{
  public:

  static void decodePermissions(const mode_t mode, Permissions& perm);
  static void encodePermissions(const Permissions& perm, mode_t& mode);

  static bool verifyIPCName(const String& name);

  static uint64_t decodeTimevalAsDelta(struct timeval& tv);

  static void waitForIO(int fd, timespan_ms_t timeout, bool read)
    throw(IOException);

  static void timespecForDelta(timespan_ms_t timeout, struct timespec& tspec,
                               time_ms_t tv = 0);

  static void setCurrentThread(void *thread);
  static void *getCurrentThread();

  private:

  static void _initThreadKey();

  static pthread_once_t _once;
  static pthread_key_t _threadKey;
};

}; // namespace ccxx

#endif // __ccxx_POSIX_hxx

/* end of header file */

