/****************************************************************************************************************************
  Basic.h

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
  NotCopyable		: Object inherits from this type cannot be copied
  Error					: Error, unlike exception, is not encouraged to catch
  Object				  : Base class of all classes

  Macros:
  CHECK_ERROR(CONDITION,DESCRIPTION)	      	: Assert, throws an Error if failed
  CHECK_FAIL(DESCRIPTION)						        : Force an assert failure
  SCOPE_VARIABLE(TYPE,VARIABLE,VALUE){ ... }	: Scoped variable

  Version: 1.0.2

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      13/02/2019 Initial coding, testing and supporting AVR architecture
  1.0.1   K Hoang      01/03/2020 Add support for STM32 and all other architectures.
  1.0.2   K Hoang      21/02/2021 Clear compiler warnings
 *****************************************************************************************************************************/

#pragma once

#ifndef VCZH_BASIC
#define VCZH_BASIC

#ifdef VCZH_CHECK_MEMORY_LEAKS
  #define _CRTDBG_MAP_ALLOC
  #include <stdlib.h>
  #include <crtdbg.h>
  #define VCZH_CHECK_MEMORY_LEAKS_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
  #define new VCZH_CHECK_MEMORY_LEAKS_NEW
#endif

#include <stdint.h>
#include <stddef.h>
#define abstract
#define __thiscall
#define __forceinline inline

#define _I8_MIN     ((vint8_t)0x80)
#define _I8_MAX     ((vint8_t)0x7F)
#define _UI8_MAX    ((vuint8_t)0xFF)

#define _I16_MIN    ((vint16_t)0x8000)
#define _I16_MAX    ((vint16_t)0x7FFF)
#define _UI16_MAX   ((vuint16_t)0xFFFF)

#define _I32_MIN    ((vint32_t)0x80000000)
#define _I32_MAX    ((vint32_t)0x7FFFFFFF)
#define _UI32_MAX   ((vuint32_t)0xFFFFFFFF)

#define _I64_MIN    ((vint64_t)0x8000000000000000L)
#define _I64_MAX    ((vint64_t)0x7FFFFFFFFFFFFFFFL)
#define _UI64_MAX   ((vuint64_t)0xFFFFFFFFFFFFFFFFL)

#define L_(x) L__(x)
#define L__(x) L ## x

namespace vl
{
  typedef int8_t					vint8_t;
  typedef uint8_t					vuint8_t;
  typedef int16_t					vint16_t;
  typedef uint16_t				vuint16_t;
  typedef int32_t					vint32_t;
  typedef uint32_t				vuint32_t;
  typedef int64_t					vint64_t;
  typedef uint64_t				vuint64_t;


  /// <summary>Signed interface whose size is equal to sizeof(void*).</summary>
  typedef vint32_t				vint;
  /// <summary>Signed interface whose size is equal to sizeof(void*).</summary>
  typedef vint32_t				vsint;
  /// <summary>Unsigned interface whose size is equal to sizeof(void*).</summary>
  typedef vuint32_t				vuint;

  /// <summary>Signed interger representing position.</summary>
  typedef vint64_t				pos_t;

  #define ITOA_S		_itoa_s
  #define ITOW_S		_itow_s
  #define I64TOA_S	_i64toa_s
  #define I64TOW_S	_i64tow_s
  #define UITOA_S		_ui64toa_s
  #define UITOW_S		_ui64tow_s
  #define UI64TOA_S	_ui64toa_s
  #define UI64TOW_S	_ui64tow_s

  //#define INCRC(x)	(__sync_add_and_fetch(x, 1))
  //#define DECRC(x)	(__sync_sub_and_fetch(x, 1))
  #define INCRC(x)	(++(*x))
  #define DECRC(x)	(--(*x))


  /***********************************************************************
    NotCopyable Base Class
  ***********************************************************************/

  class NotCopyable
  {
    private:
      NotCopyable(const NotCopyable&)
      {
      }
      NotCopyable& operator=(const NotCopyable&)
      {
        return *this;
      }
    public:
      NotCopyable()
      {
      }
  };

  #define SCOPE_VARIABLE(TYPE, VARIABLE, VALUE)\
    if(bool __scope_variable_flag__=true)\
      for(TYPE VARIABLE = VALUE;__scope_variable_flag__;__scope_variable_flag__=false)

  /***********************************************************************
    Type Traits
  ***********************************************************************/

  template<typename T>
  struct RemoveReference
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveReference<T&>
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveReference < T&& >
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveConst
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveConst<const T>
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveVolatile
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveVolatile<volatile T>
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveCVR
  {
    typedef T			Type;
  };

  template<typename T>
  struct RemoveCVR<T&>
  {
    typedef typename RemoveCVR<T>::Type		Type;
  };

  template<typename T>
  struct RemoveCVR < T&& >
  {
    typedef typename RemoveCVR<T>::Type		Type;
  };

  template<typename T>
  struct RemoveCVR<const T>
  {
    typedef typename RemoveCVR<T>::Type		Type;
  };

  template<typename T>
  struct RemoveCVR<volatile T>
  {
    typedef typename RemoveCVR<T>::Type		Type;
  };

  template<typename T>
  typename RemoveReference<T>::Type&& MoveValue(T&& value)
  {
    return (typename RemoveReference<T>::Type&&)value;
  }

  template<typename T>
  T&& ForwardValue(typename RemoveReference<T>::Type&& value)
  {
    return (T&&)value;
  }

  template<typename T>
  T&& ForwardValue(typename RemoveReference<T>::Type& value)
  {
    return (T&&)value;
  }

  template<typename ...TArgs>
  struct TypeTuple
  {
  };

  /***********************************************************************
    Object Base Class
  ***********************************************************************/

  /// <summary>Base type of all classes.</summary>
  class Object
  {
    public:
      virtual ~Object()
      {
      }
  };

  /// <summary>Type for storing a value to wherever requiring a [T:vl.Ptr`1] to [T:vl.Object].</summary>
  /// <typeparam name="T">Type of the value.</typeparam>
  template<typename T>
  class ObjectBox : public Object
  {
    private:
      T	object;
    public:
      /// <summary>Box a value.</summary>
      /// <param name="_object">The value to box.</param>
      ObjectBox(const T& _object)
        : object(_object)
      {
      }

      /// <summary>Box a movable value.</summary>
      /// <param name="_object">The value to box.</param>
      ObjectBox(T&& _object)
        : object(MoveValue(_object))
      {
      }

      /// <summary>Copy a box.</summary>
      /// <param name="value">The box.</param>
      ObjectBox(const ObjectBox<T>& value)
        : object(value.object)
      {
      }

      /// <summary>Move a box.</summary>
      /// <param name="value">The box.</param>
      ObjectBox(ObjectBox<T>&& value)
        : object(MoveValue(value.object))
      {
      }

      /// <summary>Box a value.</summary>
      /// <returns>The boxed value.</returns>
      /// <param name="_object">The value to box.</param>
      ObjectBox<T>& operator=(const T& _object)
      {
        object = _object;
        return *this;
      }

      /// <summary>Copy a box.</summary>
      /// <returns>The boxed value.</returns>
      /// <param name="value">The box.</param>
      ObjectBox<T>& operator=(const ObjectBox<T>& value)
      {
        object = value.object;
        return *this;
      }

      /// <summary>Move a box.</summary>
      /// <returns>The boxed value.</returns>
      /// <param name="value">The box.</param>
      ObjectBox<T>& operator=(ObjectBox<T>&& value)
      {
        object = MoveValue(value.object);
        return *this;
      }

      /// <summary>Unbox the value.</summary>
      /// <returns>The original value.</returns>
      const T& Unbox()
      {
        return object;
      }
  };

  /// <summary>Type for optionally storing a value.</summary>
  /// <typeparam name="T">Type of the value.</typeparam>
  template<typename T>
  class Nullable
  {
    private:
      T*	object;
    public:
      /// <summary>Create a null value.</summary>
      Nullable()
        : object(0)
      {
      }

      /// <summary>Create a non-null value.</summary>
      /// <param name="value">The value to copy.</param>
      Nullable(const T& value)
        : object(new T(value))
      {
      }

      /// <summary>Create a non-null value.</summary>
      /// <param name="value">The value to move.</param>
      Nullable(T&& value)
        : object(new T(MoveValue(value)))
      {
      }

      /// <summary>Copy a nullable value.</summary>
      /// <param name="nullable">The nullable value to copy.</param>
      Nullable(const Nullable<T>& nullable)
        : object(nullable.object ? new T(*nullable.object) : 0)
      {
      }

      /// <summary>Move a nullable value.</summary>
      /// <param name="nullable">The nullable value to move.</param>
      Nullable(Nullable<T>&& nullable)
        : object(nullable.object)
      {
        nullable.object = 0;
      }

      ~Nullable()
      {
        if (object)
        {
          delete object;
          object = 0;
        }
      }

      /// <summary>Create a non-null value.</summary>
      /// <returns>The created nullable value.</returns>
      /// <param name="value">The value to copy.</param>
      Nullable<T>& operator=(const T& value)
      {
        if (object)
        {
          delete object;
          object = 0;
        }
        
        object = new T(value);
        return *this;
      }

      /// <summary>Copy a nullable value.</summary>
      /// <returns>The created nullable value.</returns>
      /// <param name="nullable">The nullable value to copy.</param>
      Nullable<T>& operator=(const Nullable<T>& nullable)
      {
        if (this != &nullable)
        {
          if (object)
          {
            delete object;
            object = 0;
          }
          
          if (nullable.object)
          {
            object = new T(*nullable.object);
          }
        }
        
        return *this;
      }

      /// <summary>Move a nullable value.</summary>
      /// <returns>The created nullable value.</returns>
      /// <param name="nullable">The nullable value to move.</param>
      Nullable<T>& operator=(Nullable<T>&& nullable)
      {
        if (this != &nullable)
        {
          if (object)
          {
            delete object;
            object = 0;
          }
          
          object = nullable.object;
          nullable.object = 0;
        }
        return *this;
      }

      static bool Equals(const Nullable<T>& a, const Nullable<T>& b)
      {
        return
          a.object
          ? b.object
          ? *a.object == *b.object
          : false
          : b.object
          ? false
          : true;
      }

      static vint Compare(const Nullable<T>& a, const Nullable<T>& b)
      {
        return
          a.object
          ? b.object
          ? (*a.object == *b.object ? 0 : *a.object < *b.object ? -1 : 1)
          : 1
          : b.object
          ? -1
          : 0;
      }

      bool operator==(const Nullable<T>& nullable)const
      {
        return Equals(*this, nullable);
      }

      bool operator!=(const Nullable<T>& nullable)const
      {
        return !Equals(*this, nullable);
      }

      bool operator<(const Nullable<T>& nullable)const
      {
        return Compare(*this, nullable) < 0;
      }

      bool operator<=(const Nullable<T>& nullable)const
      {
        return Compare(*this, nullable) <= 0;
      }

      bool operator>(const Nullable<T>& nullable)const
      {
        return Compare(*this, nullable) > 0;
      }

      bool operator>=(const Nullable<T>& nullable)const
      {
        return Compare(*this, nullable) >= 0;
      }

      /// <summary>Convert the nullable value to a bool value.</summary>
      /// <returns>Returns true if it is not null.</returns>
      operator bool()const
      {
        return object != 0;
      }

      /// <summary>Unbox the value. This operation will cause an access violation of it is null.</summary>
      /// <returns>The original value.</returns>
      const T& Value()const
      {
        return *object;
      }
  };

  template<typename T, size_t minSize>
  union BinaryRetriver
  {
    T t;
    char binary[sizeof(T) > minSize ? sizeof(T) : minSize];
  };

  /***********************************************************************
    Configuration Type Traits
  ***********************************************************************/

  /// <summary>Get the index type of a value for containers.</summary>
  /// <typeparam name="T">Type of the value.</typeparam>
  template<typename T>
  struct KeyType
  {
    public:
      /// <summary>The index type of a value for containers.</summary>
      typedef T Type;

      /// <summary>Convert a value to its index type.</summary>
      /// <returns>The corresponding index value.</returns>
      /// <param name="value">The value.</param>
      static T GetKeyValue(const T& value)
      {
        return value;
      }
  };

  /// <summary>Test is a type a Plain-Old-Data type for containers.</summary>
  /// <typeparam name="T">The type to test.</typeparam>
  template<typename T>
  struct POD
  {
    /// <summary>Returns true if the type is a Plain-Old-Data type.</summary>
    static const bool Result = false;
  };

  template<>struct POD<bool> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vint8_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vuint8_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vint16_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vuint16_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vint32_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vuint32_t> {
  
    static const bool Result = true;
  };
  
  template<>struct POD<vint64_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<vuint64_t> 
  {
    static const bool Result = true;
  };
  
  template<>struct POD<char> 
  {
    static const bool Result = true;
  };
  
  template<typename T>struct POD<T*> 
  {
    static const bool Result = true;
  };
  
  template<typename T>struct POD<T&> 
  {
    static const bool Result = true;
  };
  
  template<typename T>struct POD < T&& > 
  {
    static const bool Result = true;
  };
  
  template<typename T, typename C>struct POD<T C::*> 
  {
    static const bool Result = true;
  };
  
  template<typename T, vint _Size>struct POD<T[_Size]> 
  {
    static const bool Result = POD<T>::Result;
  };
  
  template<typename T>struct POD<const T> 
  {
    static const bool Result = POD<T>::Result;
  };
  
  template<typename T>struct POD<volatile T> 
  {
    static const bool Result = POD<T>::Result;
  };
  
  template<typename T>struct POD<const volatile T> 
  {
    static const bool Result = POD<T>::Result;
  };

  /***********************************************************************
    Interface Class
  ***********************************************************************/

  /// <summary>Base type of all interfaces. All interface types are encouraged to be virtual inherited.</summary>
  class Interface : private NotCopyable
  {
    public:
      virtual ~Interface()
      {
      }
  };

  /***********************************************************************
    Type Extraction Class
  ***********************************************************************/

  struct YesType {};
  struct NoType {};

  template<typename T, typename YesOrNo>
  struct AcceptType
  {
  };

  template<typename T>
  struct AcceptType<T, YesType>
  {
    typedef T Type;
  };

  template<typename T1, typename T2>
  struct YesNoAnd
  {
    typedef NoType Type;
  };

  template<>
  struct YesNoAnd<YesType, YesType>
  {
    typedef YesType Type;
  };

  template<typename T1, typename T2>
  struct YesNoOr
  {
    typedef YesType Type;
  };

  template<>
  struct YesNoOr<NoType, NoType>
  {
    typedef NoType Type;
  };

  template<typename YesOrNo>
  struct AcceptValue
  {
    static const bool Result = false;
  };

  template<>
  struct AcceptValue<YesType>
  {
    static const bool Result = true;
  };

  template<typename T>
  T ValueOf();

  template<typename TFrom, typename TTo>
  struct PointerConvertable
  {
    static YesType Test(TTo* value);
    static NoType Test(void* value);

    typedef decltype(Test(ValueOf<TFrom*>())) YesNoType;
  };

  template<typename TFrom, typename TTo>
  struct ReturnConvertable
  {
    static YesType Test(TTo&& value);
    static NoType Test(...);

    typedef decltype(Test(ValueOf < TFrom&& > ())) YesNoType;
  };

  template<typename TFrom>
  struct ReturnConvertable<TFrom, void>
  {
    typedef YesType YesNoType;
  };

  template<typename TTo>
  struct ReturnConvertable<void, TTo>
  {
    typedef NoType YesNoType;
  };

  template<>
  struct ReturnConvertable<void, void>
  {
    typedef YesType YesNoType;
  };

  template<typename T, typename U>
  struct AcceptAlways
  {
    typedef T Type;
  };

  template<typename TFrom, typename TTo>
  struct RequiresConvertable
  {
    static YesType Test(TTo* value);
    static NoType Test(void* value);

    typedef decltype(Test((TFrom*)0)) YesNoType;
  };
}         // namespace vl

#endif    // VCZH_BASIC
