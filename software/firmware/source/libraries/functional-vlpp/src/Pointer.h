/****************************************************************************************************************************
  Pointer.h

  This library provides function templates to better support C++ functional programming across platforms.
  Based on Vlpp library (https://github.com/vczh-libraries/Vlpp)
  and Marcus Rugger functional-vlpp library (https://github.com/marcusrugger/functional-vlpp)
  Built by Khoi Hoang (https://github.com/khoih-prog/functional-vlpp)
  Licensed under MIT license

  Original author
  Vczh Library++ 3.0
  Developer: Zihan Chen(vczh)
  Framework::Basic

  Classes:
  Ptr<T>							：Smart pointer

  Version: 1.0.2

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      13/02/2019 Initial coding, testing and supporting AVR architecture
  1.0.1   K Hoang      01/03/2020 Add support for STM32 and all other architectures.
  1.0.2   K Hoang      21/02/2021 Clear compiler warnings
 *****************************************************************************************************************************/

#pragma once

#ifndef VCZH_POINTER
#define VCZH_POINTER

#include "Basic.h"

namespace vl
{

  /***********************************************************************
    ReferenceCounterOperator
  ***********************************************************************/

  /// <summary>The strategy to get the pointer to the reference counter from an object. If you get the same pointer multiple times 
  /// from the same object by calling [M:vl.ReferenceCounterOperator`2.CreateCounter], than it is safe to convert a object pointer 
  /// to a [T:vl.Ptr`1]. Currently for reflectable C++ types which inherit from [T:vl.reflection.DescriptableObject] it is yet. 
  /// For others it is no.</summary>
  /// <typeparam name="T">The type of the object.</typeparam>
  /// <typeparam name="Enabled">[T:vl.Ptr`1] will always use [T:vl.YesType] as the second type parameter. This parameter is useful 
  /// when you want to do partial specialization in the SFINAE way.</typeparam>
  template<typename T, typename Enabled = YesType>
  struct ReferenceCounterOperator
  {
    /// <summary>Create a pointer to the reference counter from an object.</summary>
    /// <returns>The pointer to the reference counter.</returns>
    /// <param name="reference">The object.</param>
    static __forceinline volatile vint* CreateCounter(T* reference)
    {
      (void) reference;
      return new vint(0);
    }

    /// <summary>Destroy a pointer to the reference counter from an object.</summary>
    /// <param name="counter">The pointer to the reference counter.</param>
    /// <param name="reference">The object.</param>
    static __forceinline void DeleteReference(volatile vint* counter, void* reference)
    {
      delete counter;
      delete (T*)reference;
    }
  };

  /***********************************************************************
    Smart Ptr Class
  ***********************************************************************/

  /// <summary>A smart pointer. It is always safe to convert a pointer to an object to a smart pointer once. If you do it multiple times,
  /// it may be wrong due to different implementation of [T:vl.ReferenceCounterOperator`2]. In case of wrong, disposing the smart pointer
  /// will cause an access violation.</summary>
  /// <typeparam name="T">The type of the object.</typeparam>
  template<typename T>
  class Ptr
  {
      template<typename X>
      friend class Ptr;
      
    protected:
      typedef void		(*Destructor)(volatile vint*, void*);

      volatile vint*		counter;
      T*					reference;
      void*				originalReference;
      Destructor			originalDestructor;

      void Inc()
      {
        if (counter)
        {
          INCRC(counter);
        }
      }

      void Dec()
      {
        if (counter)
        {
          if (DECRC(counter) == 0)
          {
            originalDestructor(counter, originalReference);
            counter = 0;
            reference = 0;
            originalReference = 0;
            originalDestructor = 0;
          }
        }
      }

      volatile vint* Counter()const
      {
        return counter;
      }

      Ptr(volatile vint* _counter, T* _reference, void* _originalReference, Destructor _originalDestructor)
        : counter(_counter)
        , reference(_reference)
        , originalReference(_originalReference)
        , originalDestructor(_originalDestructor)
      {
        Inc();
      }
      
    public:

      /// <summary>Create a null pointer.</summary>
      Ptr()
        : counter(0)
        , reference(0)
        , originalReference(0)
        , originalDestructor(0)
      {
      }

      /// <summary>Convert a pointer to an object to a smart pointer.</summary>
      /// <param name="pointer">The pointer to the object.</param>
      Ptr(T* pointer)
        : counter(0)
        , reference(0)
        , originalReference(0)
        , originalDestructor(0)
      {
        if (pointer)
        {
          counter = ReferenceCounterOperator<T>::CreateCounter(pointer);
          reference = pointer;
          originalReference = pointer;
          originalDestructor = &ReferenceCounterOperator<T>::DeleteReference;
          Inc();
        }
      }

      /// <summary>Copy a smart pointer.</summary>
      /// <param name="pointer">The smart pointer to copy.</param>
      Ptr(const Ptr<T>& pointer)
        : counter(pointer.counter)
        , reference(pointer.reference)
        , originalReference(pointer.originalReference)
        , originalDestructor(pointer.originalDestructor)
      {
        Inc();
      }

      /// <summary>Move a smart pointer.</summary>
      /// <param name="pointer">The smart pointer to Move.</param>
      Ptr(Ptr<T>&& pointer)
        : counter(pointer.counter)
        , reference(pointer.reference)
        , originalReference(pointer.originalReference)
        , originalDestructor(pointer.originalDestructor)
      {
        pointer.counter = 0;
        pointer.reference = 0;
        pointer.originalReference = 0;
        pointer.originalDestructor = 0;
      }

      /// <summary>Cast a smart pointer.</summary>
      /// <typeparam name="C">The type of the object before casting.</typeparam>
      /// <param name="pointer">The smart pointer to cast.</param>
      template<typename C>
      Ptr(const Ptr<C>& pointer)
        : counter(0)
        , reference(0)
        , originalReference(0)
        , originalDestructor(0)
      {
        T* converted = pointer.Obj();
        if (converted)
        {
          counter = pointer.Counter();
          reference = converted;
          originalReference = pointer.originalReference;
          originalDestructor = pointer.originalDestructor;
          Inc();
        }
      }

      ~Ptr()
      {
        Dec();
      }

      /// <summary>Cast a smart pointer.</summary>
      /// <typeparam name="C">The type of the object after casting.</typeparam>
      /// <returns>The casted smart pointer. Returns null if failed.</returns>
      template<typename C>
      Ptr<C> Cast()const
      {
        C* converted = dynamic_cast<C*>(reference);
        return Ptr<C>((converted ? counter : 0), converted, originalReference, originalDestructor);
      }

      /// <summary>Convert a pointer to an object to a smart pointer.</summary>
      /// <returns>The converted smart pointer.</returns>
      /// <param name="pointer">The pointer to the object.</param>
      Ptr<T>& operator=(T* pointer)
      {
        Dec();
        
        if (pointer)
        {
          counter = ReferenceCounterOperator<T>::CreateCounter(pointer);
          reference = pointer;
          originalReference = pointer;
          originalDestructor = &ReferenceCounterOperator<T>::DeleteReference;
          Inc();
        }
        else
        {
          counter = 0;
          reference = 0;
          originalReference = 0;
          originalDestructor = 0;
        }
        
        return *this;
      }

      /// <summary>Copy a smart pointer.</summary>
      /// <returns>The copied smart pointer.</returns>
      /// <param name="pointer">The smart pointer to copy.</param>
      Ptr<T>& operator=(const Ptr<T>& pointer)
      {
        if (this != &pointer)
        {
          Dec();
          counter = pointer.counter;
          reference = pointer.reference;
          originalReference = pointer.originalReference;
          originalDestructor = pointer.originalDestructor;
          Inc();
        }
        
        return *this;
      }

      /// <summary>Move a smart pointer.</summary>
      /// <returns>The moved smart pointer.</returns>
      /// <param name="pointer">The smart pointer to Move.</param>
      Ptr<T>& operator=(Ptr<T>&& pointer)
      {
        if (this != &pointer)
        {
          Dec();
          counter = pointer.counter;
          reference = pointer.reference;
          originalReference = pointer.originalReference;
          originalDestructor = pointer.originalDestructor;

          pointer.counter = 0;
          pointer.reference = 0;
          pointer.originalReference = 0;
          pointer.originalDestructor = 0;
        }
        
        return *this;
      }

      /// <summary>Cast a smart pointer.</summary>
      /// <typeparam name="C">The type of the object before casting.</typeparam>
      /// <returns>The smart pointer after casting.</returns>
      /// <param name="pointer">The smart pointer to cast.</param>
      template<typename C>
      Ptr<T>& operator=(const Ptr<C>& pointer)
      {
        T* converted = pointer.Obj();
        Dec();
        
        if (converted)
        {
          counter = pointer.counter;
          reference = converted;
          originalReference = pointer.originalReference;
          originalDestructor = pointer.originalDestructor;
          Inc();
        }
        else
        {
          counter = 0;
          reference = 0;
          originalReference = 0;
          originalDestructor = 0;
        }
        
        return *this;
      }

      bool operator==(const T* pointer)const
      {
        return reference == pointer;
      }

      bool operator!=(const T* pointer)const
      {
        return reference != pointer;
      }

      bool operator>(const T* pointer)const
      {
        return reference > pointer;
      }

      bool operator>=(const T* pointer)const
      {
        return reference >= pointer;
      }

      bool operator<(const T* pointer)const
      {
        return reference < pointer;
      }

      bool operator<=(const T* pointer)const
      {
        return reference <= pointer;
      }

      bool operator==(const Ptr<T>& pointer)const
      {
        return reference == pointer.reference;
      }

      bool operator!=(const Ptr<T>& pointer)const
      {
        return reference != pointer.reference;
      }

      bool operator>(const Ptr<T>& pointer)const
      {
        return reference > pointer.reference;
      }

      bool operator>=(const Ptr<T>& pointer)const
      {
        return reference >= pointer.reference;
      }

      bool operator<(const Ptr<T>& pointer)const
      {
        return reference < pointer.reference;
      }

      bool operator<=(const Ptr<T>& pointer)const
      {
        return reference <= pointer.reference;
      }

      /// <summary>Test if it is a null pointer.</summary>
      /// <returns>Returns true if it is not null.</returns>
      operator bool()const
      {
        return reference != 0;
      }

      /// <summary>Get the pointer to the object.</summary>
      /// <returns>The pointer to the object.</returns>
      T* Obj()const
      {
        return reference;
      }

      /// <summary>Get the pointer to the object.</summary>
      /// <returns>The pointer to the object.</returns>
      T* operator->()const
      {
        return reference;
      }
  };

  /***********************************************************************
    ComPtr
  ***********************************************************************/

  template<typename T>
  class ComPtr
  {
    protected:
      volatile vint*		counter;
      T*					reference;

      void Inc()
      {
        if (counter)
        {
          INCRC(counter);
        }
      }

      void Dec()
      {
        if (counter)
        {
          if (DECRC(counter) == 0)
          {
            delete counter;
            reference->Release();
            counter = 0;
            reference = 0;
          }
        }
      }

      volatile vint* Counter()const
      {
        return counter;
      }

      ComPtr(volatile vint* _counter, T* _reference)
        : counter(_counter)
        , reference(_reference)
      {
        Inc();
      }
      
    public:

      ComPtr()
      {
        counter = 0;
        reference = 0;
      }

      ComPtr(T* pointer)
      {
        if (pointer)
        {
          counter = new volatile vint(1);
          reference = pointer;
        }
        else
        {
          counter = 0;
          reference = 0;
        }
      }

      ComPtr(const ComPtr<T>& pointer)
      {
        counter = pointer.counter;
        reference = pointer.reference;
        Inc();
      }

      ComPtr(ComPtr<T>&& pointer)
      {
        counter = pointer.counter;
        reference = pointer.reference;

        pointer.counter = 0;
        pointer.reference = 0;
      }

      ~ComPtr()
      {
        Dec();
      }

      ComPtr<T>& operator=(T* pointer)
      {
        Dec();
        
        if (pointer)
        {
          counter = new vint(1);
          reference = pointer;
        }
        else
        {
          counter = 0;
          reference = 0;
        }
        return *this;
      }

      ComPtr<T>& operator=(const ComPtr<T>& pointer)
      {
        if (this != &pointer)
        {
          Dec();
          counter = pointer.counter;
          reference = pointer.reference;
          Inc();
        }
        
        return *this;
      }

      ComPtr<T>& operator=(ComPtr<T>&& pointer)
      {
        if (this != &pointer)
        {
          Dec();
          counter = pointer.counter;
          reference = pointer.reference;

          pointer.counter = 0;
          pointer.reference = 0;
        }
        
        return *this;
      }

      bool operator==(const T* pointer)const
      {
        return reference == pointer;
      }

      bool operator!=(const T* pointer)const
      {
        return reference != pointer;
      }

      bool operator>(const T* pointer)const
      {
        return reference > pointer;
      }

      bool operator>=(const T* pointer)const
      {
        return reference >= pointer;
      }

      bool operator<(const T* pointer)const
      {
        return reference < pointer;
      }

      bool operator<=(const T* pointer)const
      {
        return reference <= pointer;
      }

      bool operator==(const ComPtr<T>& pointer)const
      {
        return reference == pointer.reference;
      }

      bool operator!=(const ComPtr<T>& pointer)const
      {
        return reference != pointer.reference;
      }

      bool operator>(const ComPtr<T>& pointer)const
      {
        return reference > pointer.reference;
      }

      bool operator>=(const ComPtr<T>& pointer)const
      {
        return reference >= pointer.reference;
      }

      bool operator<(const ComPtr<T>& pointer)const
      {
        return reference < pointer.reference;
      }

      bool operator<=(const ComPtr<T>& pointer)const
      {
        return reference <= pointer.reference;
      }

      operator bool()const
      {
        return reference != 0;
      }

      T* Obj()const
      {
        return reference;
      }

      T* operator->()const
      {
        return reference;
      }
  };

  template<typename T, typename ...TArgs>
  Ptr<T> MakePtr(TArgs ...args)
  {
    return new T(args...);
  }

  /***********************************************************************
    Traits
  ***********************************************************************/

  template<typename T>
  struct KeyType<Ptr<T>>
  {
    typedef T* Type;

    static T* GetKeyValue(const Ptr<T>& key)
    {
      return key.Obj();
    }
  };

  template<typename T>
  struct POD<Ptr<T>>
  {
    static const bool Result = false;
  };

  template<typename T>
  struct KeyType<ComPtr<T>>
  {
    typedef T* Type;

    static T* GetKeyValue(const ComPtr<T>& key)
    {
      return key.Obj();
    }
  };

  template<typename T>
  struct POD<ComPtr<T>>
  {
    static const bool Result = false;
  };
}         // namespace vl

#endif    // VCZH_POINTER
