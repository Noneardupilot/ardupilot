/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_ChibiOS.h"
#include "Scheduler.h"
#include "Util.h"

#include <AP_HAL_ChibiOS/UARTDriver.h>
#include <AP_HAL_ChibiOS/AnalogIn.h>
#include <AP_HAL_ChibiOS/Storage.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include <AP_HAL_ChibiOS/RCInput.h>
#include <AP_HAL_ChibiOS/CAN.h>

#if CH_CFG_USE_DYNAMIC == TRUE

#include <DataFlash/DataFlash.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "hwdef/common/stm32_util.h"
#include "shared_dma.h"
#include "sdcard.h"

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;
THD_WORKING_AREA(_timer_thread_wa, TIMER_THD_WA_SIZE);
THD_WORKING_AREA(_rcin_thread_wa, RCIN_THD_WA_SIZE);
#ifndef HAL_USE_EMPTY_IO
THD_WORKING_AREA(_io_thread_wa, IO_THD_WA_SIZE);
#endif
#ifndef HAL_USE_EMPTY_STORAGE
THD_WORKING_AREA(_storage_thread_wa, STORAGE_THD_WA_SIZE);
#endif
Scheduler::Scheduler()
{
}


/************************************************************************************************************************************
*函数原型：void Scheduler::init()
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::init()
{
    chBSemObjectInit(&_timer_semaphore, false); //定时器二进制信号量，初始状态是0，没有被取用，所以后面的信号量就可以被使用
    chBSemObjectInit(&_io_semaphore, false);    //IO二进制信号量，初始状态是0，没有被取用，，所以后面的信号量就可以被使用
    //设置定时器线程-这将调用任务在1kHz---- setup the timer thread - this will call tasks at 1kHz
    _timer_thread_ctx = chThdCreateStatic(_timer_thread_wa,
                     sizeof(_timer_thread_wa),
                     APM_TIMER_PRIORITY,        /* Initial priority.181    */
                     _timer_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */

    //设置RCIN线程-这将调用任务在1kHz---- setup the RCIN thread - this will call tasks at 1kHz
    _rcin_thread_ctx = chThdCreateStatic(_rcin_thread_wa,
                     sizeof(_rcin_thread_wa),
                     APM_RCIN_PRIORITY,        /* Initial priority. 177   */
                     _rcin_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#ifndef HAL_USE_EMPTY_IO
    //IO线程以较低优先级运行--------- the IO thread runs at lower priority
    _io_thread_ctx = chThdCreateStatic(_io_thread_wa,
                     sizeof(_io_thread_wa),
                     APM_IO_PRIORITY,        /* Initial priority.  58    */
                     _io_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif

#ifndef HAL_USE_EMPTY_STORAGE
    //存储线程在IO优先级之上运行。-------- the storage thread runs at just above IO priority
    _storage_thread_ctx = chThdCreateStatic(_storage_thread_wa,
                     sizeof(_storage_thread_wa),
                     APM_STORAGE_PRIORITY,        /* Initial priority.      */
                     _storage_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif

}



/************************************************************************************************************************************
*函数原型：void Scheduler::delay_microseconds(uint16_t usec)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/

void Scheduler::delay_microseconds(uint16_t usec)
{
    if (usec == 0) { //chibios faults with 0us sleep
        return;
    }
    uint32_t ticks;
    ticks = chTimeUS2I(usec);
    if (ticks == 0) {
        // calling with ticks == 0 causes a hard fault on ChibiOS
        ticks = 1;
    }
    chThdSleep(ticks); //Suspends Thread for desired microseconds
}

/*
  wrapper around sem_post that boosts main thread priority
 */
static void set_high_priority()
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    hal_chibios_set_priority(APM_MAIN_PRIORITY_BOOST);
#endif
}

/*
  return the main thread to normal priority
 */
void Scheduler::boost_end(void)
{
#if APM_MAIN_PRIORITY_BOOST != APM_MAIN_PRIORITY
    if (in_main_thread() && _priority_boosted) {
        _priority_boosted = false;
        hal_chibios_set_priority(APM_MAIN_PRIORITY);
    }
#endif
}

/*
  a variant of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop
 */
void Scheduler::delay_microseconds_boost(uint16_t usec)
{
    if (in_main_thread()) {
        set_high_priority();
        _priority_boosted = true;
    }
    delay_microseconds(usec); //Suspends Thread for desired microseconds
    _called_boost = true;
}

/*如果上次检查后调用了Delay-MySudisSoBuST（），则返回true
  return true if delay_microseconds_boost() has been called since last check
 */
bool Scheduler::check_called_boost(void)
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

/***********************************************************************************************************************
*函数原型：void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
*函数功能：寄存器计时器进程
*修改日期：2018-9-12
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    chBSemWait(&_timer_semaphore);
    for (uint8_t i = 0; i < _num_timer_procs; i++)
    {
        if (_timer_proc[i] == proc)
        {
            chBSemSignal(&_timer_semaphore);
            return;
        }
    }

    if (_num_timer_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) //CHIBIOS_SCHEDULER_MAX_TIMER_PROCS=8
    {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else
    {
        hal.console->printf("Out of timer processes\n");
    }
    chBSemSignal(&_timer_semaphore);
}


/***********************************************************************************************************************
*函数原型：void Scheduler::register_io_process(AP_HAL::MemberProc proc)
*函数功能：寄存器计时器进程
*修改日期：2018-9-12
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{

    chBSemWait(&_io_semaphore);  //等待二进制信号量到来，如果来了，往下执行

//    hal.uartG->printf("GG1\r\n");
    for (uint8_t i = 0; i < _num_io_procs; i++)
    {
        if (_io_proc[i] == proc)
        {
            chBSemSignal(&_io_semaphore); //释放二进制信号量
            return;
        }
    }
    
    if (_num_io_procs < CHIBIOS_SCHEDULER_MAX_TIMER_PROCS) //最多8个
    {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else
    {
        hal.console->printf("Out of IO processes\n");
    }
    chBSemSignal(&_io_semaphore); //释放二进制信号量
}



void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();

#ifndef NO_DATAFLASH
    //stop logging
    DataFlash_Class::instance()->StopLogging();

    // stop sdcard driver, if active
    sdcard_stop();
#endif

#if defined(HAL_USE_RTC) && HAL_USE_RTC
    // setup RTC for fast reboot
    set_fast_reboot(hold_in_bootloader?RTC_BOOT_HOLD:RTC_BOOT_FAST);
#endif

    // disable all interrupt sources
    port_disable();
    
    // reboot
    NVIC_SystemReset();
}


/************************************************************************************************************************************
*函数原型：void Scheduler::_run_timers()
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_run_timers()
{
//	hal.uartG->printf("CHIBIOS\r\n");
//	hal.uartG->printf("_in_timer_proc=%d\r\n",_in_timer_proc);
    if (_in_timer_proc) //在定时器进程中？，第一次不会进入if
    {
        return;
    }
    _in_timer_proc = true;

    int num_procs = 0;
    chBSemWait(&_timer_semaphore);  //等待信号量，这里可以直接进去，不用再等待了，相当于共享资源保护，为什么？看上面对信号量的初始值的讲解
    num_procs = _num_timer_procs;   //运行保存这个值，只有该任务执行完毕，才能更改
  //  hal.uartG->printf("num_procs=%d\r\n",num_procs);
    chBSemSignal(&_timer_semaphore);
    //现在调用基于计时器的驱动程序----- now call the timer based drivers
    for (int i = 0; i < num_procs; i++) //这里主要判断有多少定时器驱动程序
    {
        if (_timer_proc[i])
        {
            _timer_proc[i](); //这个函数在哪，看下面的函数
        }
    }

    //故障安全，如果一个初始化---- and the failsafe, if one is setup
    if (_failsafe != nullptr)
    {
        _failsafe();
    }

#if HAL_USE_ADC == TRUE
    //处理模拟输入线程-----process analog input
    ((AnalogIn *)hal.analogin)->_timer_tick();
#endif

    _in_timer_proc = false;
}


/************************************************************************************************************************************
*函数原型：void Scheduler::_timer_thread(void *arg)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("apm_timer");

    while (!sched->_hal_initialized)     //没有完成初始化
    {
        sched->delay_microseconds(1000); //延迟1ms
    }
    while (true)
    {
        sched->delay_microseconds(1000);

        //运行注册计时器------run registered timers
        sched->_run_timers();

        //处理任何请求的的遥控器RC输出请求---- process any pending RC output requests
        hal.rcout->timer_tick();
    }
}



/************************************************************************************************************************************
*函数原型：void Scheduler::_rcin_thread(void *arg)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("apm_rcin"); //进程名字
    while (!sched->_hal_initialized)
    {
        sched->delay_microseconds(20000);
    }
    while (true)
    {
        sched->delay_microseconds(2500);
        ((RCInput *)hal.rcin)->_timer_tick(); //2.5ms运行一次
    }
}

/************************************************************************************************************************************
*函数原型：void Scheduler::_run_io(void)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_run_io(void)
{

    if (_in_io_proc)  //_in_io_proc=0，第一次刚开始时
    {
        return;
    }
    //由于第一次_in_io_proc=0，运行到这里被设置1后，如果进入二进制信号量等待的话，一直处于返回状态，因为_in_io_proc没有被设置成0，说明一次run_io还没有运行完
    _in_io_proc = true;

    int num_procs = 0;            //进程计数
    chBSemWait(&_io_semaphore);   //等待信号量的到来
    hal.uartG->printf("GG2\r\n");
    num_procs = _num_io_procs;
    chBSemSignal(&_io_semaphore); //释放二进制信号量
    //现在调用基于IO的驱动程序----- -now call the IO based drivers
    for (int i = 0; i < num_procs; i++)
    {
        if (_io_proc[i])
        {
            _io_proc[i]();//IO_TIMER()
        }
    }

    _in_io_proc = false;
}

/************************************************************************************************************************************
*函数原型：void Scheduler::_io_thread(void* arg)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_io_thread(void* arg)
{
	hal.uartG->printf("GGG\r\n");
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("apm_io");   //设置线程名称
    while (!sched->_hal_initialized) //没有初始化就等待1ms
    {
        sched->delay_microseconds(1000);
    }
    while (true)
    {
        sched->delay_microseconds(1000); //io线程的周期是1ms

        //运行注册IO进程--------------run registered IO processes
        sched->_run_io();
    }
}


/************************************************************************************************************************************
*函数原型：void Scheduler::_run_io(void)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::_storage_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    chRegSetThreadName("apm_storage");
    while (!sched->_hal_initialized)
    {
        sched->delay_microseconds(10000);
    }
    while (true)
    {
        sched->delay_microseconds(10000); //10ms

        // process any pending storage writes
        hal.storage->_timer_tick();
    }
}

/************************************************************************************************************************************
*函数原型：void Scheduler::_run_io(void)
*函数功能：进程初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
bool Scheduler::in_main_thread() const
{
    return get_main_thread() == chThdGetSelfX();
}


/************************************************************************************************************************************
*函数原型：void Scheduler::system_initialized()
*函数功能：系统初始化
*修改日期：2018-10-29
*备   注：
*************************************************************************************************************************************/
void Scheduler::system_initialized()
{
    if (_initialized)
    {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;

  //  hal.uartG->printf("_initialized=%d\r\n",_initialized);

}

/*
  disable interrupts and return a context that can be used to
  restore the interrupt state. This can be used to protect
  critical regions
*/
void *Scheduler::disable_interrupts_save(void)
{
    return (void *)(uintptr_t)chSysGetStatusAndLockX();
}

/*
  restore interrupt state from disable_interrupts_save()
*/
void Scheduler::restore_interrupts(void *state)
{
    chSysRestoreStatusX((syssts_t)(uintptr_t)state);
}

/*
  trampoline for thread create
*/
void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    uint8_t thread_priority = APM_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_MAIN_PRIORITY_BOOST},
        { PRIORITY_MAIN, APM_MAIN_PRIORITY},
        { PRIORITY_SPI, APM_SPI_PRIORITY},
        { PRIORITY_I2C, APM_I2C_PRIORITY},
        { PRIORITY_CAN, APM_CAN_PRIORITY},
        { PRIORITY_TIMER, APM_TIMER_PRIORITY},
        { PRIORITY_RCIN, APM_RCIN_PRIORITY},
        { PRIORITY_IO, APM_IO_PRIORITY},
        { PRIORITY_UART, APM_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_STORAGE_PRIORITY},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority, LOWPRIO, HIGHPRIO);
            break;
        }
    }
    thread_t *thread_ctx = chThdCreateFromHeap(NULL,
                                               THD_WORKING_AREA_SIZE(stack_size),
                                               name,
                                               thread_priority,
                                               thread_create_trampoline,
                                               tproc);
    if (thread_ctx == nullptr) {
        free(tproc);
        return false;
    }
    return true;
}

#endif // CH_CFG_USE_DYNAMIC
