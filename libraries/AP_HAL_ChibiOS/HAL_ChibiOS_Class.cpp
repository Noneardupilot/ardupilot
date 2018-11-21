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
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <assert.h>

#include "HAL_ChibiOS_Class.h"
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Private.h>
#include "shared_dma.h"
#include "sdcard.h"
#include "hwdef/common/usbcfg.h"
#include "hwdef/common/stm32_util.h"

#include <hwdef.h>

#ifndef HAL_NO_UARTDRIVER
static HAL_UARTA_DRIVER;
static HAL_UARTB_DRIVER;
static HAL_UARTC_DRIVER;
static HAL_UARTD_DRIVER;
static HAL_UARTE_DRIVER;
static HAL_UARTF_DRIVER;
static HAL_UARTG_DRIVER;
#else
static Empty::UARTDriver uartADriver;
static Empty::UARTDriver uartBDriver;
static Empty::UARTDriver uartCDriver;
static Empty::UARTDriver uartDDriver;
static Empty::UARTDriver uartEDriver;
static Empty::UARTDriver uartFDriver;
static Empty::UARTDriver uartGDriver;
#endif

#if HAL_USE_I2C == TRUE
static ChibiOS::I2CDeviceManager i2cDeviceManager;
#else
static Empty::I2CDeviceManager i2cDeviceManager;
#endif

#if HAL_USE_SPI == TRUE
static ChibiOS::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif

#if HAL_USE_ADC == TRUE
static ChibiOS::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif

#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ChibiOS::Storage storageDriver;
#endif
static ChibiOS::GPIO gpioDriver;
static ChibiOS::RCInput rcinDriver;

#if HAL_USE_PWM == TRUE
static ChibiOS::RCOutput rcoutDriver;
#else
static Empty::RCOutput rcoutDriver;
#endif

static ChibiOS::Scheduler schedulerInstance;
static ChibiOS::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;


#if HAL_WITH_IO_MCU
HAL_UART_IO_DRIVER;
#include <AP_IOMCU/AP_IOMCU.h>
AP_IOMCU iomcu(uart_io);
#endif

HAL_ChibiOS::HAL_ChibiOS() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &uartFDriver,
        &uartGDriver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        nullptr
        )
{}

static bool thread_running = false;        /**< Daemon status flag */
static thread_t* daemon_task;              /**< Handle of daemon task / thread */

extern const AP_HAL::HAL& hal;


/*
  set the priority of the main APM task
 */
void hal_chibios_set_priority(uint8_t priority)
{
    chSysLock();
#if CH_CFG_USE_MUTEXES == TRUE
    if ((daemon_task->prio == daemon_task->realprio) || (priority > daemon_task->prio)) {
      daemon_task->prio = priority;
    }
    daemon_task->realprio = priority;
#endif
    chSchRescheduleS();
    chSysUnlock();
}

thread_t* get_main_thread()
{
    return daemon_task;
}

static AP_HAL::HAL::Callbacks* g_callbacks;



/***********************************************************************************************************************
*函数原型：static void main_loop()
*函数功能：main_loop函数
*修改日期：2018-10-29
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/

static void main_loop()
{

    daemon_task = chThdGetSelfX(); //返回当前线程

    /*
      把main loop的优先级切换到高优先级-----switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY); //180

#ifdef HAL_I2C_CLEAR_BUS //协处理器不使用

    //- Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
    //清除所有的I2C总线。这可能需要在一些板上，可以得到一个卡住的I2C外围设备启动。
    ChibiOS::I2CBus::clear_all();
#endif

#if STM32_DMA_ADVANCED

    ChibiOS::Shared_DMA::init(); //不使能DMA
#endif
    peripheral_power_enable();   //启用外围电源
        
    hal.uartA->begin(115200);   //初始化USB的波特率

#ifdef HAL_SPI_CHECK_CLOCK_FREQ

    //SPI时钟频率的可选测试---- optional test of SPI clock frequencies
    ChibiOS::SPIDevice::test_clock_freq();
#endif 

    //初始化SD卡和文件系统-----Setup SD Card and Initialise FATFS bindings
    sdcard_init();

    hal.uartB->begin(38400);  //GPS波特率38400
    hal.uartC->begin(57600);  //串口波特率设置
    hal.analogin->init();     //模拟输入初始化，主要测试ADC功能，检查电源电压


    hal.scheduler->init();   //初始化任务init线程

    /*
        run setup() at low priority to ensure CLI doesn't hang the system, and to allow initial sensor read loops to run
     */

    //以低优先级运行SETUP（）以确保CLI(命令行界面)不挂起系统，并允许初始传感器读取循环运行。
    hal_chibios_set_priority(APM_STARTUP_PRIORITY); //APM_STARTUP_PRIORITY=10

    schedulerInstance.hal_initialized(); //_hal_initialized = true

    g_callbacks->setup();                 //调用应用层的setup（）函数
    hal.scheduler->system_initialized(); //系统初始化

    thread_running = true;
    chRegSetThreadName(SKETCHNAME);
    
    /*
      main loop切换到低优先级-------switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

   // hal.uartG->printf("UARTG\r\n"); //自己添加打印函数


    while (true)
    {

        g_callbacks->loop();  //调用APP的loop线程

        /*
          give up 250 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
          如果INS回路回调Delay-MySudiSsBooSth（）函数没有响应，则放弃250微秒的时间。
         以确保低优先级。有机会运行。回调延迟函数delay_microseconds_boost意味着我们已经放弃了主回路循环，所以我们不需要再做一次。
         */
       // hal.uartG->printf("MMM\r\n"); //自己添加打印函数
        if (!schedulerInstance.check_called_boost())
        {
        	//hal.uartG->printf("NNN\r\n"); //自己添加打印函数
            hal.scheduler->delay_microseconds(250);
        }
    }
    thread_running = false;
}





/***********************************************************************************************************************
*函数原型：void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
*函数功能：run函数
*修改日期：2018-10-29
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/

void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    /*
     * -系统初始化-------------------------------------System initializations.
     * -Chibios 硬件抽象层初始化，也就是进行驱动配置--------ChibiOS HAL initialization, this also initializes the configured device drivers
     * -和执行特殊的板层配置-----------------------------and performs the board-specific initializations.
     * -内核初始化、主函数（）成为一个线程，然后激活RTOS------ Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */

#ifdef HAL_USB_PRODUCT_ID  //协处理器不使用
  setup_usb_strings(); //动态分配USB描述符字符串，建议先不要去研究这个

#endif
    
#ifdef HAL_STDOUT_SERIAL   //协处理器不使用
    //标准输出初始化--------STDOUT Initialistion

    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);
#endif

    assert(callbacks);       //用来让程序测试条件，如果条件正确继续执行，如果条件错误，报错。

    g_callbacks = callbacks; //函数定义传递

    //接管执行main------------Takeover main
    main_loop();
}



/***********************************************************************************************************************
*函数原型：void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
*函数功能：run函数
*修改日期：2018-10-29
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_ChibiOS hal_chibios;
    return hal_chibios;
}

#endif
