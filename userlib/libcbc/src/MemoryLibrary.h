/* Copyright (c) 1994-2004 Pongsak Suvanpong (psksvp@ccs.neu.edu).  
 * All Rights Reserved.
 *
 * This computer software is owned by Pongsak Suvanpong, and is
 * protected by U.S. copyright laws and other laws and by international
 * treaties.  This computer software is furnished by Pongsak Suvanpong 
 * pursuant to a written license agreement and may be used, copied,
 * transmitted, and stored only in accordance with the terms of such
 * license and with the inclusion of the above copyright notice.  This
 * computer software or any other copies thereof may not be provided or
 * otherwise made available to any other person.
 */
#ifndef __BUFFER_HEADER__
#define __BUFFER_HEADER__

//#include "dll.ix.h"

#ifdef OSX_MACH_O   // bsd does not have malloc.h, everything in malloc.h is in stdlib.h
#define ON_FREEBSD
#endif

#ifdef __APPLE__   // bsd does not have malloc.h, everything in malloc.h is in stdlib.h
#define ON_FREEBSD
#endif

#ifndef ON_FREEBSD 
#include <malloc.h>
#else
#include <stdlib.h>
#endif


#include <memory.h>
//#include "dll.ix.h"

namespace MemoryLibrary
{
  template <typename T>
  void Copy(T *destination, T *source, int iCnt)
  {
    ::memcpy((void*)destination, (void*)source, iCnt * sizeof(T));
  }

  template <typename T>
  void Zero(T *destination, int iCnt)
  {
    ::memset((void*)destination, 0, iCnt * sizeof(T));
  }

  template <typename T>
  void Set(T *destination, unsigned char v, int iCnt)
  {
    ::memset((void*)destination, v, iCnt * sizeof(T));
  }

  class Buffer
  {
  protected:
    void* m_pBuffer;
    unsigned int m_uiSize;
  public:
    Buffer(void)
    {
      m_pBuffer = NULL;
      m_uiSize = 0;
    }

    Buffer(void* p, unsigned int s)
    {
      Set(p, s);
    }

    virtual ~Buffer(void)
    {
      m_pBuffer = NULL;
      m_uiSize = 0;
    }

    bool IsValid(void)
    {
      return m_pBuffer != NULL ? true : false;
    }

    bool IsEqualTo(MemoryLibrary::Buffer& p)
    {
      return this->Size() == p.Size() && ::memcmp(this->Ptr(), p.Ptr(), Size()) == 0;
    }

    void* Ptr(void)
    {
      return m_pBuffer;
    }
    
    void* PointerToThisBuffer()
    {
      return Ptr();
    }

    unsigned int Size(void)
    {
      return m_uiSize;
    }

    void Set(void* p, unsigned int s)
    {
      m_pBuffer = p;
      m_uiSize = s;
    }

    void Zero(void)
    {
      ::memset(m_pBuffer, 0, m_uiSize);
    }
    
    void CopyFrom(void* p)
    {
      ::memcpy(m_pBuffer, p, m_uiSize);
    }

    void CopyFrom(void* p, unsigned int s)
    {
      ::memcpy(m_pBuffer, p, s);
    }

    void CopyFrom(MemoryLibrary::Buffer& b)
    {
      CopyFrom(b.Ptr(), b.Size());
    }

    void CopyTo(void* p, unsigned s)
    {
      ::memcpy(p, m_pBuffer, s);
    }

    void CopyTo(MemoryLibrary::Buffer& b)
    {
      CopyTo(b.Ptr(), b.Size());
    }
    
    template <typename T>
    void CopyFrom(T* p)
    {
      if(sizeof(T) == Size())
        CopyFrom((void*)p, sizeof(T));
      else 
      {
        // need to pump error msg here
      }
    }
    
    template <typename T>
    void CopyTo(T* p)
    {
      if(Size() == sizeof(T))
        CopyTo((void*)p, sizeof(T));
      else 
      {
        // need to pump error
      }

    }
    
    template <typename T>
    void CopyFrom(T* p, unsigned int nBlocks)
    {
      if(nBlocks * sizeof(T) == Size())
        CopyFrom((void*)p, nBlocks * sizeof(T));
      else 
      {
        // need to pump error msg here
      }
    }
    
    template <typename T>
    void CopyTo(T* p, unsigned int nBlocks)
    {
      if(Size() == nBlocks * sizeof(T))
        CopyTo((void*)p, nBlocks * sizeof(T));
      else 
      {
        // need to pump error
      }
    }
    
    template <typename T>
    T makeValueFromOffset(int offset)
    {
      T result = (T)0;
      if((offset + sizeof(T)) < Size())
      {
        unsigned char *pStart = ((unsigned char*)m_pBuffer) + (unsigned char)offset;
        ::memcpy(&result, pStart, sizeof(T));
      }
      
      return result;
    }
    
    template <typename T>
    T MakeValueFromOffset(int offset)
    {
      T result = (T)0;
      if((offset + sizeof(T)) < Size())
      {
        unsigned char *pStart = ((unsigned char*)m_pBuffer) + (unsigned char)offset;
        ::memcpy(&result, pStart, sizeof(T));
      }
      
      return result;
    }
  };

  /////////////////////////////////////////////////////
  class Allocator
  {
  public:
    Allocator(void) {}
    virtual ~Allocator(void) {}
    virtual void* Allocate(unsigned int nRawByte)=0;
    virtual void Free(void* p)=0;
  };

  class MallocAllocator :public MemoryLibrary::Allocator
  {
  public:
    void* Allocate(unsigned int nRawByte)
    {
      return ::malloc((size_t)nRawByte);
    }

    void Free(void* p)
    {
      ::free(p);
    }
  };

#ifdef WIN32
#ifndef USING_GCC  // that means we re using VC ?????? correct assumption?? it could be bcc, or OpenWatcom??
#define ALIGN_MALLOC_VC_ENABLE
  class AlignedMallocAllocator :public MemoryLibrary::Allocator
  {
    size_t m_alignment;
  public:
    AlignedMallocAllocator(size_t alignment=16) // default to 16 byte alignment
    {
      m_alignment = alignment; 
    }

    void SetAlignment(size_t alignment)
    {
      m_alignment = alignment; // must be power of 2
    }

    void* Allocate(unsigned int nRawByte)
    {
      return ::_aligned_malloc((size_t)nRawByte, m_alignment);
    }

    void Free(void* p)
    {
      ::_aligned_free(p);
    }
  };
#endif
#endif

  class DynamicBuffer :public Buffer
  {
    MemoryLibrary::MallocAllocator m_MallocAllocator;
    MemoryLibrary::Allocator* m_pMyAllocator;
  public:
    DynamicBuffer(MemoryLibrary::Allocator* pAllocator=NULL)// use default allocator;
    {
      if(NULL == pAllocator)
      {
        m_pMyAllocator = &m_MallocAllocator;
      }
      else
      {
        m_pMyAllocator = pAllocator;
      }
    }
    
    virtual ~DynamicBuffer(void)
    {
      Free();
    }
   
    virtual bool Allocate(unsigned int nByte, MemoryLibrary::Allocator* pAllocator=NULL)
    {
      // new allocator?
      if(NULL != pAllocator && m_pMyAllocator != pAllocator)
      {
        Free(); //force to free and reallocate 
                //because we have a new allocator.
        m_pMyAllocator = pAllocator;
      } 
      
      // new size?
      
      if(nByte != m_uiSize)
      {
        Free();
        m_pBuffer = m_pMyAllocator->Allocate(nByte);
        if(NULL == m_pBuffer)
          return false;
        else
        {
          m_uiSize = nByte;
          return true;
        }
      }
      else
      {
        // buffer is already been allocated and 
        // same size with nByte, so just return true;
        return true;
      }
    }
    
    virtual void Free(void)
    {
      if(NULL != m_pBuffer)
      {
        m_pMyAllocator->Free(m_pBuffer);
        m_pBuffer = NULL;
        m_uiSize = 0;
      }
    }
  };

  ///////////////////////////////////////////////////////////////////
  template <typename T> 
  class IndexableBuffer // rename to IndexableBuffer
  {
    bool m_bFreeBuffer;
    MemoryLibrary::Allocator* m_pMyAllocator;
    MemoryLibrary::DynamicBuffer m_buffer;
    T* m_ptr;
    unsigned int m_nBlock;
  public:
    IndexableBuffer(void)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
      m_pMyAllocator = NULL;
    }

    IndexableBuffer(int iSize)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
      m_pMyAllocator = NULL;
      Allocate(iSize);
    }

    IndexableBuffer(T* ptrExtBuffer, int iSize)
    {
      m_bFreeBuffer = true;
      m_nBlock = 0;
      m_ptr = NULL;
      m_pMyAllocator = NULL;
      SetExternalBuffer(ptrExtBuffer, iSize);
    }

    virtual ~IndexableBuffer(void)
    {
      Free();
    }

    void SetExternalBuffer(T* ptr, unsigned int size)
    {
      Free();
      m_bFreeBuffer = false;
      m_ptr = ptr;
      m_buffer.Set((void*)ptr, size);
      m_nBlock = size;
    }

    void SetAllocator(MemoryLibrary::Allocator* pAllocator)
    {
      m_pMyAllocator = pAllocator;
    }

    void Copy(MemoryLibrary::IndexableBuffer<T>& a)
    {
      Copy(a.RawBuffer(), a.Size());
    }

    void Copy(T* Ptr, unsigned int iSize)
    {
      Allocate(iSize);
      m_buffer.CopyFrom((void*)Ptr, iSize * sizeof(T));
    }

    void Fill(T a)
    {
      for(unsigned int i = 0; i < this->Size(); i++)
        At(i) = a;
    }

    virtual bool Allocate(unsigned int nBlock, bool bZeroOutBuffer=true)
    {
      m_bFreeBuffer = true;
      m_nBlock = nBlock;
      if(true == m_buffer.Allocate(sizeof(T) * nBlock, m_pMyAllocator))
      {
        if(true == bZeroOutBuffer)
          m_buffer.Zero();
        m_ptr = (T*)m_buffer.Ptr();
        return true;
      }
      else
      {
        m_ptr = NULL;
        return false;
      }
    }

    virtual void Free(void)
    {
      if(true == m_bFreeBuffer)
      {
        m_nBlock = 0;
        m_buffer.Free();
        m_ptr = NULL;
      }
      else // this mean I am operating on external buffer, see SetExternalBuffer
      {
        m_buffer.Set(NULL, 0); // so DynamicBuffer won't delete the external ptr
        m_nBlock = 0;
      }
    }

    unsigned int Size(void)
    {
      return m_nBlock;
    }

    unsigned int RawSize(void)
    {
      return m_buffer.Size();
    }

    T* RawBuffer(void)
    {
      return m_ptr; 
    }

    T* Memory(void)
    {
      return m_ptr;
    }

    T& At(unsigned int iIdx)
    {
      //Debug::CheckUpperBoundRange(Size() - 1, iIdx, "MemoryLibrary::IndexableBuffer::At");
      return m_ptr[iIdx];
    }

    T &operator[](int i)
    {
      return At(i);
    }
  };
  

} // namespace MemoryLibrary

#endif

