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

#ifndef __ccxx_MemoryBlock_hxx
#define __ccxx_MemoryBlock_hxx

#include <commonc++/Common.h++>
#include <commonc++/OutOfBoundsException.h++>

namespace ccxx {

  /** An encapsulation of a block of memory. The class supports various
   * bounds-checked memory operations.
   *
   * @author Mark Lindner
   */

  class COMMONCPP_API MemoryBlock
  {
    public:

    /** Construct a new MemoryBlock.
     *
     * @param base The pointer to the beginning of the memory block.
     * @param size The size of the memory block, in bytes.
     */
    MemoryBlock(byte_t *base = NULL, size_t size = 0) throw()
      : _base(base),
        _size(size)
    { }

    /** Destructor. */
    virtual ~MemoryBlock() throw()
    { }

    /** Zero the memory block. */
    inline void zero() throw()
    { fill(0); }

    /** Fill the memory block with the given byte value. */
    void fill(byte_t value) throw();

    /** Get a pointer to the beginning of the memory block. */
    inline byte_t *getBase() throw()
    { return(_base); }

    /** Get a pointer to the beginning of the memory block. */
    inline const byte_t *getBase() const throw()
    { return(_base); }

    /** Set the pointer to the beginning of the memory block. */
    virtual inline void setBase(byte_t *base) throw()
    { _base = base; }

    /** Get the size of the memory block. */
    inline size_t getSize() const throw()
    { return(_size); }

    /** Set the size of the memory block. */
    inline void setSize(size_t size) throw()
    { _size = size; }

    /** Equality operator. */
     bool operator==(const MemoryBlock& other) const throw();

    /** Inequality operator. */
    inline bool operator!=(const MemoryBlock& other) const throw()
    { return(! operator==(other)); }

    /** Bounds-checked access. */
    byte_t operator[](int index) const throw(OutOfBoundsException);

    /** Bounds-checked access. */
    byte_t& operator[](int index) throw(OutOfBoundsException);

    /** Copy data from another memory block to this one. The number of
     * bytes copied is the minimum of the sizes of the two blocks. The
     * blocks may not overlap.
     *
     * @param other The block to copy from.
     * @return <b>true</b> if the copy succeeded, <b>false</b> if the
     * blocks overlap.
     */
    bool copyFrom(const MemoryBlock& other) throw();

    /** Copy data from another (possibly overlapping) memory block to
     * this one. The number of bytes copied is the minimum of the
     * sizes of the two blocks.
     *
     * @param other The block to copy from. Note that the copy may result
     * in both blocks being modified, if the source area overlaps the target
     * area.
     */
    void moveFrom(MemoryBlock& other) throw();

    /** Find the first occurrence of a given byte value in the memory block,
     * starting at a given index.
     *
     * @param val The value to search for.
     * @param startIndex The index to begin searching at.
     * @return The index of the first matching value, or -1 if not found.
     */
    int indexOf(byte_t val, uint_t startIndex = 0) const throw();

    /** Test if the base of the block is NULL. */
    inline bool operator!() const throw()
    { return(_base == NULL); }

    protected:

    /** A pointer to the beginning of the memory block. */
    byte_t *_base;
    /** The size of the memory block, in bytes. */
    size_t _size;

    private:

    CCXX_COPY_DECLS(MemoryBlock);
  };

}; // namespace ccxx

#endif // __ccxx_MemoryBlock_hxx

/* end of header file */
