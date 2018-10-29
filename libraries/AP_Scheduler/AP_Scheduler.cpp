/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */
#include "AP_Scheduler.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define SCHEDULER_DEFAULT_LOOP_RATE 400
#else
#define SCHEDULER_DEFAULT_LOOP_RATE  50
#endif

#define debug(level, fmt, args...)   do { if ((level) <= _debug.get()) { hal.console->printf(fmt, ##args); }} while (0)

extern const AP_HAL::HAL& hal;

int8_t AP_Scheduler::current_task = -1;

const AP_Param::GroupInfo AP_Scheduler::var_info[] = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),

    // @Param: LOOP_RATE
    // @DisplayName: Scheduling main loop rate
    // @Description: This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart. Values over 400 are considered highly experimental.
    // @Values: 50:50Hz,100:100Hz,200:200Hz,250:250Hz,300:300Hz,400:400Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),

    AP_GROUPEND
};

// constructor
AP_Scheduler::AP_Scheduler(scheduler_fastloop_fn_t fastloop_fn) :
    _fastloop_fn(fastloop_fn)
{
    if (_s_instance) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many schedulers");
#endif
        return;
    }
    _s_instance = this;

    AP_Param::setup_object_defaults(this, var_info);

    // only allow 50 to 2000 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz.set(50);
    } else if (_loop_rate_hz > 2000) {
        _loop_rate_hz.set(2000);
    }
    _last_loop_time_s = 1.0 / _loop_rate_hz;
}

/*
 * Get the AP_Scheduler singleton
 */
AP_Scheduler *AP_Scheduler::_s_instance = nullptr;
AP_Scheduler *AP_Scheduler::get_instance()
{
    return _s_instance;
}

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit)
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;

    // setup initial performance counters
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();

    _log_performance_bit = log_performance_bit;
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = AP_HAL::micros();
    uint32_t now = run_started_usec;

    if (_debug > 1 && _perf_counters == nullptr) {
        _perf_counters = new AP_HAL::Util::perf_counter_t[_num_tasks];
        if (_perf_counters != nullptr) {
            for (uint8_t i=0; i<_num_tasks; i++) {
                _perf_counters[i] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i].name);
            }
        }
    }
    
    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = _loop_rate_hz / _tasks[i].rate_hz;
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt < interval_ticks) {
            // this task is not yet scheduled to run again
            continue;
        }
        // this task is due to run. Do we have enough time to run it?
        _task_time_allowed = _tasks[i].max_time_micros;

        if (dt >= interval_ticks*2) {
            // we've slipped a whole run of this task!
            debug(2, "Scheduler slip task[%u-%s] (%u/%u/%u)\n",
                  (unsigned)i,
                  _tasks[i].name,
                  (unsigned)dt,
                  (unsigned)interval_ticks,
                  (unsigned)_task_time_allowed);
        }

        if (_task_time_allowed > time_available) {
            // not enough time to run this task.  Continue loop -
            // maybe another task will fit into time remaining
            continue;
        }

        // run it
        _task_time_started = now;
        current_task = i;
        if (_debug > 1 && _perf_counters && _perf_counters[i]) {
            hal.util->perf_begin(_perf_counters[i]);
        }
        _tasks[i].function();
        if (_debug > 1 && _perf_counters && _perf_counters[i]) {
            hal.util->perf_end(_perf_counters[i]);
        }
        current_task = -1;

        // record the tick counter when we ran. This drives
        // when we next run the event
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        now = AP_HAL::micros();
        uint32_t time_taken = now - _task_time_started;

        if (time_taken > _task_time_allowed) {
            // the event overran!
            debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                  (unsigned)i,
                  _tasks[i].name,
                  (unsigned)time_taken,
                  (unsigned)_task_time_allowed);
        }
        if (time_taken >= time_available) {
            time_available = 0;
            break;
        }
        time_available -= time_taken;
    }

    // update number of spare microseconds
    _spare_micros += time_available;

    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = AP_HAL::micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average()
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (_spare_micros/_spare_ticks);
    return used_time / (float)loop_us;
}


/***********************************************************************************************************************
*函数原型：void AP_Scheduler::loop()
*函数功能：核心循环函数
*修改日期：2018-10-16
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void AP_Scheduler::loop()
{
	//等待INS数据采集完毕-------------------------wait for an INS sample
	    AP::ins().wait_for_sample();

	    //获取系统采样时间
	    const uint32_t sample_time_us = AP_HAL::micros();

	    //开始时间等于0?
	    if (_loop_timer_start_us == 0)
	    {
	        _loop_timer_start_us = sample_time_us;
	        _last_loop_time_s = get_loop_period_s();
	    } else
	    {
	        _last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6; //得到执行整个loop函数的时间
	    }

	    //执行快速循环函数----Execute the fast loop
	    // ---------------------
	    if (_fastloop_fn)
	    {
	    	// hal.uartG->printf("_fastloop_fn=%d\r\n",_fastloop_fn); //loop_us=2500,这个单位是us
	    	// hal.uartG->printf("MMM\r\n"); //loop_us=2500,这个单位是us
	        _fastloop_fn(); //该函数会通过指针函数知识，调用fast_loop()函数
	       // hal.uartG->printf("NNN\r\n"); //loop_us=2500,这个单位是us
	    }

	    //高速度调度器，一个任务节拍已经执行------tell the scheduler one tick has passed
	    tick();
	    //运行所有要运行的任务。注意我们只是必须按每个循环调用一次，因为任务是按计划进行的，并且每个任务是主循环的倍数。
	    //所以如果在第一次运行主循环时，任务表中的任务有不运行的任务.将等待下一次任务调度运行到来，才有可能会被运行。
	    //这里我举个例子：主任务时间2.5ms，其中一个数组表任务时间是10ms，那么需要运行四次loop才可能运行数组表中的任务，前三次就是上面说的情况
	    // run all the tasks that are due to run. Note that we only
	    // have to call this once per loop, as the tasks are scheduled
	    // in multiples of the main loop tick. So if they don't run on
	    // the first call to the scheduler they won't run on a later
	    // call until scheduler.tick() is called again
	    const uint32_t loop_us = get_loop_period_us();  //这个是主循环时间，400HZ，
	    //这里增加串口打印函数，
	 //   hal.uartG->printf("loop_us=%d\r\n",loop_us); //loop_us=2500,这个单位是us

	    //获取执行完fast_loop()函数后，剩余多少时间给数组表任务使用=开始+2.5ms-运行完上面fast_loop（）到这的时间，获得剩余时间，主要给数组表任务使用
	    const uint32_t time_available = (sample_time_us + loop_us) - AP_HAL::micros();
	    //打印出剩余时间
	  //  hal.uartG->printf("time_available=%d\r\n",time_available); //time_available=2212..


	    //运行run（）函数，运行数组表任务
	    run(time_available > loop_us ? 0u : time_available); //运行函数，上面计算剩余的时间都留给任务表中的任务去用。

	#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
	    // move result of AP_HAL::micros() forward:
	    hal.scheduler->delay_microseconds(1);
	#endif

	    //检查loop时间-----check loop time
	    perf_info.check_loop_time(sample_time_us - _loop_timer_start_us);

	    //重新赋值loop起始时间，下次计算loop运行时间使用
	    _loop_timer_start_us = sample_time_us;

}





/***********************************************************************************************************************
*函数原型：void AP_Scheduler::loop()
*函数功能：核心循环函数
*修改日期：2018-10-16
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void AP_Scheduler::update_logging()
{
    if (debug_flags()) {
        perf_info.update_logging();
    }
    if (_log_performance_bit != (uint32_t)-1 &&
        DataFlash_Class::instance()->should_log(_log_performance_bit)) {
        Log_Write_Performance();
    }
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();
}

// Write a performance monitoring packet
void AP_Scheduler::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us          : AP_HAL::micros64(),
        num_long_running : perf_info.get_num_long_running(),
        num_loops        : perf_info.get_num_loops(),
        max_time         : perf_info.get_max_time(),
        mem_avail        : hal.util->available_memory(),
        load             : (uint16_t)(load_average() * 1000)
    };
    DataFlash_Class::instance()->WriteCriticalBlock(&pkt, sizeof(pkt));
}

namespace AP {

AP_Scheduler &scheduler()
{
    return *AP_Scheduler::get_instance();
}

};
