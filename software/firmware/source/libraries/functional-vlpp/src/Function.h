/****************************************************************************************************************************
  Function.h

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
  Func<function-type>						：Function object

  Functions:
  Curry   :: (A->B) -> A -> B		：Currying
  Combine :: (A->B) -> (A->C) -> (B->C->D) -> (A->D)	：Combine multiple functors using an operator

  Version: 1.0.2

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      13/02/2019 Initial coding, testing and supporting AVR architecture
  1.0.1   K Hoang      01/03/2020 Add support for STM32 and all other architectures.
  1.0.2   K Hoang      21/02/2021 Clear compiler warnings
 *****************************************************************************************************************************/

#pragma once

#ifndef VCZH_FUNCTION
#define VCZH_FUNCTION

#include <stdlib.h>
#include "Basic.h"
#include "Pointer.h"

namespace vl
{

  /***********************************************************************
    vl::Func<R(TArgs...)>
  ***********************************************************************/

  template<typename T>
  class Func
  {
  };

  namespace internal_invokers
  {
  template<typename R, typename ...TArgs>
  class Invoker : public Object
  {
    public:
      virtual R Invoke(TArgs&& ...args) = 0;
  };

  //------------------------------------------------------

  template<typename R, typename ...TArgs>
  class StaticInvoker : public Invoker<R, TArgs...>
  {
    protected:
      R(*function)(TArgs ...args);

    public:
      StaticInvoker(R(*_function)(TArgs...))
        : function(_function)
      {
      }

      R Invoke(TArgs&& ...args)override
      {
        return function(ForwardValue<TArgs>(args)...);
      }
  };

  //------------------------------------------------------

  template<typename C, typename R, typename ...TArgs>
  class MemberInvoker : public Invoker<R, TArgs...>
  {
    protected:
      C*	sender;
      R(C::*function)(TArgs ...args);

    public:
      MemberInvoker(C* _sender, R(C::*_function)(TArgs ...args))
        : sender(_sender)
        , function(_function)
      {
      }

      R Invoke(TArgs&& ...args)override
      {
        return (sender->*function)(ForwardValue<TArgs>(args)...);
      }
  };

  //------------------------------------------------------

  template<typename C, typename R, typename ...TArgs>
  class ObjectInvoker : public Invoker<R, TArgs...>
  {
    protected:
      C		function;

    public:
      ObjectInvoker(const C& _function)
        : function(_function)
      {
      }

      R Invoke(TArgs&& ...args)override
      {
        return function(ForwardValue<TArgs>(args)...);
      }
  };

  //------------------------------------------------------

  template<typename C, typename ...TArgs>
  class ObjectInvoker<C, void, TArgs...> : public Invoker<void, TArgs...>
  {
    protected:
      C		function;

    public:
      ObjectInvoker(const C& _function)
        : function(_function)
      {
      }

      void Invoke(TArgs&& ...args)override
      {
        function(ForwardValue<TArgs>(args)...);
      }
  };
  }

  /// <summary>A type representing a function reference.</summary>
  /// <typeparam name="R">The return type.</typeparam>
  /// <typeparam name="TArgs">Types of parameters.</typeparam>
  template<typename R, typename ...TArgs>
  class Func<R(TArgs...)> : public Object
  {
    protected:
      Ptr<internal_invokers::Invoker<R, TArgs...>> invoker;
    public:
      typedef R FunctionType(TArgs...);
      typedef R ResultType;

      /// <summary>Create a null function reference.</summary>
      Func()
      {
      }

      /// <summary>Copy a function reference.</summary>
      /// <param name="function">The function reference to copy.</param>
      Func(const Func<R(TArgs...)>& function)
        : invoker(function.invoker)
      {
      }

      /// <summary>Create a reference using a function pointer.</summary>
      /// <param name="function">The function pointer.</param>
      Func(R(*function)(TArgs...))
      {
        invoker = new internal_invokers::StaticInvoker<R, TArgs...>(function);
      }

      /// <summary>Create a reference using a method.</summary>
      /// <typeparam name="C">Type of the class that has the method.</typeparam>
      /// <param name="sender">The object that has the method.</param>
      /// <param name="function">The function pointer.</param>
      template<typename C>
      Func(C* sender, R(C::*function)(TArgs...))
      {
        invoker = new internal_invokers::MemberInvoker<C, R, TArgs...>(sender, function);
      }

      /// <summary>Create a reference using a function object.</summary>
      /// <typeparam name="C">Type of the function object.</typeparam>
      /// <param name="function">The function object. It could be a lambda expression.</param>
      template<typename C>
      Func(const C& function)
      {
        invoker = new internal_invokers::ObjectInvoker<C, R, TArgs...>(function);
      }

      /// <summary>Invoke the function.</summary>
      /// <returns>Returns the function result.</returns>
      /// <param name="args">Arguments to invoke the function.</param>
      R operator()(TArgs ...args)const
      {
        return invoker->Invoke(ForwardValue<TArgs>(args)...);
      }

      Func<R(TArgs...)>& operator=(const Func<R(TArgs...)>& function)
      {
        invoker = function.invoker;
        return *this;
      }

      Func<R(TArgs...)>& operator=(const Func<R(TArgs...)>&& function)
      {
        invoker = MoveValue(function.invoker);
        return *this;
      }

      bool operator==(const Func<R(TArgs...)>& function)const
      {
        return invoker == function.invoker;
      }

      bool operator!=(const Func<R(TArgs...)>& function)const
      {
        return invoker != function.invoker;
      }

      /// <summary>Test is the reference a null reference.</summary>
      /// <returns>Returns true if it is not a null reference.</returns>
      operator bool()const
      {
        return invoker;
      }
  };

  /***********************************************************************
    vl::function_lambda::LambdaRetriveType<R(TArgs...)>
  ***********************************************************************/

  namespace function_lambda
  {
    template<typename T>
    struct LambdaRetriveType
    {
      typedef vint Type;
      typedef vint FunctionType;
      typedef vint ResultType;
    };

    template<typename T>
    struct FunctionObjectRetriveType
    {
      typedef typename LambdaRetriveType<decltype(&T::operator())>::Type Type;
      typedef typename LambdaRetriveType<decltype(&T::operator())>::FunctionType FunctionType;
      typedef typename LambdaRetriveType<decltype(&T::operator())>::ResultType ResultType;
      typedef typename LambdaRetriveType<decltype(&T::operator())>::ParameterTypes ParameterTypes;
    };

    template<typename TObject, typename R, typename ...TArgs>
    struct LambdaRetriveType<R (__thiscall TObject::*)(TArgs...)const>
    {
      typedef Func<R(TArgs...)> Type;
      typedef R(FunctionType)(TArgs...);
      typedef R ResultType;
    };

    template<typename TObject, typename R, typename ...TArgs>
    struct LambdaRetriveType<R (__thiscall TObject::*)(TArgs...)>
    {
      typedef Func<R(TArgs...)> Type;
      typedef R(FunctionType)(TArgs...);
      typedef R ResultType;
    };

    template<typename R, typename ...TArgs>
    struct FunctionObjectRetriveType<R(*)(TArgs...)>
    {
      typedef Func<R(TArgs...)> Type;
      typedef R(FunctionType)(TArgs...);
      typedef R ResultType;
    };

    /// <summary>Create a function reference to a function object or a lambda expression, with all type information 
    /// automatically inferred. You can use the macro called "LAMBDA" to refer to this function.</summary>
    /// <typeparam name="T">Type of the function object or the lambda expression.</typeparam>
    /// <returns>The function reference.</returns>
    /// <param name="functionObject">The function object or the lambda expression.</param>
    template<typename T>
    typename LambdaRetriveType<decltype(&T::operator())>::Type Lambda(T functionObject)
    {
      return functionObject;
    }

    /// <summary>Create a function reference to a function pointer, with all type information 
    /// automatically inferred. You can use the macro called "FUNCTION" to refer to this function.</summary>
    /// <typeparam name="T">Type of the function pointer.</typeparam>
    /// <returns>The function reference.</returns>
    /// <param name="functionObject">The function pointer.</param>
    template<typename T>
    typename FunctionObjectRetriveType<T>::Type ConvertToFunction(T functionObject)
    {
      return functionObject;
    }

    #define LAMBDA vl::function_lambda::Lambda
    #define FUNCTION vl::function_lambda::ConvertToFunction
    #define FUNCTION_TYPE(T) typename vl::function_lambda::FunctionObjectRetriveType<T>::Type
    #define FUNCTION_RESULT_TYPE(T) typename vl::function_lambda::FunctionObjectRetriveType<T>::ResultType
  }   // namespace function_lambda

  /***********************************************************************
    vl::function_binding::Binding<R(TArgs...)>
  ***********************************************************************/

  namespace function_binding
  {
    template<typename T>
    struct Binding
    {
    };

    template<typename T>
    struct CR 
    {
      typedef const T& Type;
    };
    
    template<typename T>
    struct CR<T&> 
    {
      typedef T& Type;
    };
    
    template<typename T>
    struct CR<const T> 
    {
      typedef const T& Type;
    };
    
    template<typename T>
    struct CR<const T&> 
    {
      typedef const T& Type;
    };

    template<typename R, typename T0, typename ...TArgs>
    struct Binding<R(T0, TArgs...)>
    {
      typedef R FunctionType(T0, TArgs...);
      typedef R CurriedType(TArgs...);
      typedef T0 FirstParameterType;

      class Binder : public Object
      {
        protected:
        Func<FunctionType>				target;
        T0								firstArgument;
        public:
        Binder(const Func<FunctionType>& _target, T0 _firstArgument)
        : target(_target), firstArgument(ForwardValue<T0>(_firstArgument))
        {
        }

        R operator()(TArgs ...args)const
        {
          return target(firstArgument, args...);
        }
      };

      class Currier : public Object
      {
        protected:
        Func<FunctionType>		target;
        public:
        Currier(const Func<FunctionType>& _target)
        : target(_target)
        {
        }

        Func<CurriedType> operator()(T0 firstArgument)const
        {
          return Binder(target, firstArgument);
        }
      };
    };
  }   // namespace function_binding

  /// <summary>Currize a function. Currizing means to create a new function whose argument is the first argument 
  /// of the original function. Calling this function will return another function reference whose arguments is all 
  /// remain arguments of the original function. Calling the returned function will call the original function.</summary>
  /// <typeparam name="T">Type of the function.</typeparam>
  /// <returns>The currized function.</returns>
  /// <param name="function">The function pointer to currize.</param>
  template<typename T>
  Func<Func<typename function_binding::Binding<T>::CurriedType>(typename function_binding::Binding<T>::FirstParameterType)>
  Curry(T* function)
  {
    return typename function_binding::Binding<T>::Currier(function);
  }

  /// <summary>Currize a function. Currizing means to create a new function whose argument is the first argument 
  /// of the original function. Calling this function will return another function reference whose arguments is all 
  /// remain arguments of the original function. Calling the returned function will call the original function.</summary>
  /// <typeparam name="T">Type of the function.</typeparam>
  /// <returns>The currized function.</returns>
  /// <param name="function">The function reference to currize.</param>
  template<typename T>
  Func<Func<typename function_binding::Binding<T>::CurriedType>(typename function_binding::Binding<T>::FirstParameterType)>
  Curry(const Func<T>& function)
  {
    return typename function_binding::Binding<T>::Currier(function);
  }

  /***********************************************************************
    vl::function_combining::Combining<R1(TArgs...), R2(TArgs...), R(R1,R2)>
  ***********************************************************************/

  namespace function_combining
  {
    template<typename A, typename B, typename C>
    class Combining
    {
    };

    template<typename R1, typename R2, typename R, typename ...TArgs>
    class Combining<R1(TArgs...), R2(TArgs...), R(R1, R2)> : public Object
    {
      protected:
        Func<R1(TArgs...)>		function1;
        Func<R2(TArgs...)>		function2;
        Func<R(R1, R2)>				converter;
      public:
        typedef R1 FirstFunctionType(TArgs...);
        typedef R2 SecondFunctionType(TArgs...);
        typedef R ConverterFunctionType(R1, R2);
        typedef R FinalFunctionType(TArgs...);

        Combining(const Func<R1(TArgs...)>& _function1, const Func<R2(TArgs...)>& _function2, const Func<R(R1, R2)>& _converter)
          : function1(_function1), function2(_function2), converter(_converter)
        {
        }

        R operator()(TArgs&& ...args)const
        {
          return converter(function1(ForwardValue<TArgs>(args)...), function2(ForwardValue<TArgs>(args)...));
        }
    };
  }   // namespace function_combining

  /// <summary>Combine two functions with a converter function. The two functions to combine should have the same argument types. 
  /// The converter function will use the return values of the two function to calculate the final value.</summary>
  /// <typeparam name="F1">Type of the first function.</typeparam>
  /// <typeparam name="F2">Type of the second function.</typeparam>
  /// <typeparam name="C">Type of the converter function.</typeparam>
  /// <returns>A new function whose argument list are the same of the two functions to provide. Calling this function 
  /// will call function1, function2 and converter in order to calculate the final value.</returns>
  /// <param name="converter">The converter function.</param>
  /// <param name="function1">The first function.</param>
  /// <param name="function2">The second function.</param>
  template<typename F1, typename F2, typename C>
  Func<typename function_combining::Combining<F1, F2, C>::FinalFunctionType>
  Combine(Func<C> converter, Func<F1> function1, Func<F2> function2)
  {
    return function_combining::Combining<F1, F2, C>(function1, function2, converter);
  }

  /// <summary>Use the converter function to create a combiner, who will receive two function and use <see cref="Combine"/> to create 
  /// a combined function. This function assumes the result types of the two provided function in the future are the same, 
  /// and the converter function will not change the result type.</summary>
  /// <typeparam name="T">Type of the two functions to combine.</typeparam>
  /// <returns>The combiner.</returns>
  /// <param name="converter">The converter function.</param>
  template<typename T>
  Func<Func<T>(Func<T>, Func<T>)> Combiner(const Func<typename Func<T>::ResultType(typename Func<T>::ResultType, typename Func<T>::ResultType)>& converter)
  {
    typedef typename Func<T>::ResultType R;
    return Curry<Func<T>(Func<R(R, R)>, Func<T>, Func<T>)>(Combine)(converter);
  }
}         // namespace vl

#endif    // VCZH_FUNCTION

