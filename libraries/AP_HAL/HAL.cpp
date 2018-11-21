#include <assert.h>

#include "HAL.h"
//assert的作用是先计算表达式 expression ，如果其值为假（即为0），
//那么它先向stderr打印一条出错信息，然后通过调用 abort 来终止程序运行。请看下面的程序清单badptr.c：
namespace AP_HAL
{

HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    : _setup(setup_fun)
    , _loop(loop_fun)
{
    assert(setup_fun);
    assert(loop_fun);
}

}
