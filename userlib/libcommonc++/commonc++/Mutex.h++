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

#ifndef __ccxx_Mutex_hxx
#define __ccxx_Mutex_hxx

#include <commonc++/Common.h++>
#include <commonc++/Lock.h++>

#ifdef CCXX_OS_POSIX
#include <pthread.h>
#endif

namespace ccxx {

/** A mutual-exclusion lock.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API Mutex : public Lock
{
  friend class CondVar;

  public:

  /** Construct a new Mutex.
   *
   * @param recursive A flag indicating whether the mutex will be
   * recursive. A recursive mutex can be re-entered by a thread that
   * already holds the mutex.
   */
  Mutex(bool recursive = false) throw();

  /** Destructor. Note that destroying a locked mutex could lead to
   * deadlock.
   */
  ~Mutex() throw();

  /** %Lock the mutex, blocking until the lock is acquired. */
  void lock() throw();

  /** Try to lock the mutex, returning if it could not be acquired
   * within the given amount of time. On platforms that do not support
   * timed mutex locks, the method returns immediately if the lock could
   * not be acquired. On some platforms (notably Linux), recursive mutexes
   * do not support timed locks; on these platforms, tryLock() behaves
   * identically to lock() when applied to a recursive mutex.
   *
   * @param timeout The timeout, in milliseconds.
   * @return <b>true</b> if the lock was acquired, <b>false</b> otherwise.
   */
  bool tryLock(timespan_ms_t timeout = 0) throw();

  /** Unlock the mutex. */
  void unlock() throw();

  /** Determine if this mutex is recursive. */
  inline bool isRecursive() const throw()
  { return(_recursive); }

  /** Determine if the host system supports timed mutex locks. */
  static bool supportsTimedLocks() throw();

  protected:

#ifdef CCXX_OS_WINDOWS
  HANDLE _mutex;
#else
  pthread_mutex_t _mutex;
#endif

  private:

  bool _recursive;

  CCXX_COPY_DECLS(Mutex);
};

}; // namespace ccxx

#endif // __ccxx_Mutex_hxx

/* end of header file */
