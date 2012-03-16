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

#ifndef __ccxx_LoadableModule_hxx
#define __ccxx_LoadableModule_hxx

#include <commonc++/Common.h++>
#include <commonc++/IOException.h++>
#include <commonc++/String.h++>
#include <commonc++/Version.h++>

namespace ccxx {

/** An encapsulation of a dynamically loadable module, such as a shared
 * library or object file on UNIX systems or a DLL on Windows. Note that on
 * Mac OS X, loadable modules are expected to have an ".so" extension, which
 * is in more common use than the ".bundle" extension recommended by Apple.
 *
 * @author Mark Lindner
 */

class COMMONCPP_API LoadableModule
{
  public:

  /** Construct a LoadableModule for the loadable object specified by the
   * given path.
   *
   * @param path The path to the object file.
   */

  LoadableModule(const String &path = String::null);

  /** Destructor. Closes the object file. */

  ~LoadableModule() throw();

  /** Open the loadable object file.
   *
   * @throw IOException If the object could not be loaded, or if no path
   * has been specified.
   */

  void open() throw(IOException);

  /** Close the loadable object file. */

  void close() throw();

  /** Determine if the object file has been opened.
   */

  inline bool isOpen() const throw()
  { return(_open); }

  /** Look up a symbol within the loadable object.
   *
   * @param symbol The symbol to look up.
   * @return A pointer to the symbol, or <b>NULL</b> if the symbol was not
   * found, or if the file is not open.
   */

  void *lookup(const char *symbol) throw();

  /** Look up a symbol within the loadable object.
   *
   * @param symbol The symbol to look up.
   * @return A pointer to the symbol, or <b>NULL</b> if the symbol was not
   * found, or if the file is not open.
   */

  inline void * lookup(const String &symbol) throw()
  { return(lookup(symbol.c_str())); }

  /** Look up a symbol within the loadable object.
   *
   * @param symbol The symbol to look up.
   * @return A pointer to the symbol, or <b>NULL</b> if the symbol was not
   * found, or if the file is not open.
   */

  inline void * operator[](const char *symbol) throw()
  { return(lookup(symbol)); }

  /** Look up a symbol within the loadable object.
   *
   * @param symbol The symbol to look up.
   * @return A pointer to the symbol, or <b>NULL</b> if the symbol was not
   * found, or if the file is not open.
   */

  inline void * operator[](const String &symbol) throw()
  { return(lookup(symbol.c_str())); }

  /** Set the object file path. If the object file is open, the call has
   * no effect.
   */

  inline void setPath(const String &path) throw()
  { if(!_open) _path = path; }

  /** Get the object file path.
   */

  inline String getPath() const throw()
  { return(_path); }

  /** Get the filename extension for loadable objects (shared
   * libraries) on the host system.  This returns "dll" on Windows,
   * "so" on Mac OS X, and "so" on all other (POSIX) systems.
   */

  static String getExtension() throw();

  /** Get the filename prefix for loadable objects (shared
   * libraries) on the host system.  This returns "lib" on POSIX
   * systems (including Mac OS X) and an empty string on Windows.
   */

  static String getPrefix() throw();

  /** Construct a complete filename for a loadable module stem based
   * on the conventions of the host system. For example, "foo"
   * produces "foo.dll" on Windows, and "libfoo.so" on Mac OS X and
   * all other (POSIX) systems.
   *
   * @param stem The stem.
   * @return The complete filename.
   */

  static String fileNameForStem(const String& stem) throw();

  /** Construct a complete filename for a shared library stem and
   * version based on the conventions of the host system. For
   * example, a stem of "foo" and version of "9:0:6" produces
   * "foo-3.dll" on Windows, "libfoo.3.6.0.so" on Mac OS X, and
   * "libfoo.so.3.6.0" on all other (POSIX) systems. (Note that a loadable
   * module on Mac OS X may also have the extension ".bundle", though ".so"
   * is more commonly used.)
   *
   * @param stem The stem.
   * @param version The version.
   * @return The complete filename.
   */

  static String fileNameForStem(const String& stem, const Version& version)
    throw();

  /** Test a filename to determine if it matches the naming convention for
   * shared libraries on the host system.
   *
   * @param fileName The filename to test.
   */

  static bool isLoadableModuleFile(const String& fileName) throw();

  private:

  String _path;
  bool _open;

#ifdef CCXX_OS_WINDOWS
  HINSTANCE _handle;
#else
  void *_handle;
#endif

  CCXX_COPY_DECLS(LoadableModule);
};

}; // namespace ccxx

#endif // __ccxx_LoadableModule_hxx

/* end of header file */
