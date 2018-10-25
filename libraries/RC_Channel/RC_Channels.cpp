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
 *       RC_Channels.cpp - class containing an array of RC_Channel objects
 *
 */

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"

bool RC_Channels::has_new_overrides;
AP_Float *RC_Channels::override_timeout;
AP_Int32 *RC_Channels::options;

/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    override_timeout = &_override_timeout;
    options = &_options;

    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("RC_Channels must be singleton");
    }
    _singleton = this;
}




/***********************************************************************************************************************
*函数原型：void RC_Channels::init(void)
*函数功能：初始化遥控器
*修改日期：2018-10-24
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void RC_Channels::init(void)
{
    //初始化通道------setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++)
    {
        channel(i)->ch_in = i; //初始化1-16通道
    }

    init_aux_all(); //初始化外部开关
}


/***********************************************************************************************************************
*函数原型：uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
*函数功能：获取遥控器输入
*修改日期：2018-10-24
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/

uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
{
    memset(chans, 0, num_channels*sizeof(*chans));

    const uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    for (uint8_t i = 0; i < read_channels; i++) {
        chans[i] = channel(i)->get_radio_in();
    }

    return read_channels;
}

// update all the input channels
bool RC_Channels::read_input(void)
{
    if (!hal.rcin->new_input() && !has_new_overrides)
    {
        return false;
    }

    has_new_overrides = false;

    bool success = false;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++)
    {
        success |= channel(i)->update();
    }

    return success;
}

/***********************************************************************************************************************
*函数原型：bool RC_Channels::read_input_japan_arm(void)
*函数功能：读取遥控器输入数据---更新遥控器输入数据
*修改日期：2018-9-7
*修改作者：cihang_uav
*备注信息：update all the input channels
*************************************************************************************************************************/
bool RC_Channels::read_input_japan_arm(void)
{
	    if (!hal.rcin->new_input() && !has_new_overrides) //判断有数据到来了吗，has_new_overrides默认设置1,这里重点看hal.rcin->new_input()
	    {
	        return false;
	    }

	    has_new_overrides = false;

	    bool success = false; //设置这个标志位，就是配合下面的for循环使用

	 //这里判断采用美国手，还是日本手，来进行遥控器操作

	   for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) //NUM_RC_CHANNELS=16
	   {
			if((i==1)||(i==2))
			{
				 if(i==1)//本来是美国手，这里变成日本手
				 {

					 success |= channel(i+1)->update_japan_arm();
				 }
				 else if(i==2)
				 {

					 success |= channel(i-1)->update_japan_arm();
				 }

			}
			else
			{
				 success |= channel(i)->update(); //其他不变化

			}


		}

	    return success;
}











uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
}

void RC_Channels::clear_overrides(void)
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        _rc.channel(i)->clear_override();
    }
    // we really should set has_new_overrides to true, and rerun read_input from
    // the vehicle code however doing so currently breaks the failsafe system on
    // copter and plane, RC_Channels needs to control failsafes to resolve this
}

void RC_Channels::set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms)
{
    RC_Channels &_rc = rc();
    if (chan < NUM_RC_CHANNELS) {
        _rc.channel(chan)->set_override(value, timestamp_ms);
    }
}

bool RC_Channels::has_active_overrides()
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        if (_rc.channel(i)->has_override()) {
            return true;
        }
    }

    return false;
}

bool RC_Channels::receiver_bind(const int dsmMode)
{
    return hal.rcin->rc_bind(dsmMode);
}


// support for auxillary switches:
// read_aux_switches - checks aux switch positions and invokes configured actions
void RC_Channels::read_aux_all()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }

    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        c->read_aux();
    }
}


/***********************************************************************************************************************
*函数原型：void RC_Channels::init(void)
*函数功能：初始化遥控器
*修改日期：2018-10-24
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void RC_Channels::init_aux_all()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++)
    {
        RC_Channel *c = channel(i);
        if (c == nullptr)
        {
            continue;
        }
        c->init_aux(); //初始化
    }
    reset_mode_switch();
}




/***********************************************************************************************************************
*函数原型：void RC_Channels::init(void)
*函数功能：初始化遥控器
*修改日期：2018-10-24
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
//
// Support for mode switches
//
RC_Channel *RC_Channels::flight_mode_channel()
{
    const int8_t num = flight_mode_channel_number();
    if (num <= 0) {
        return nullptr;
    }
    if (num >= NUM_RC_CHANNELS) {
        return nullptr;
    }
    return channel(num-1);
}

void RC_Channels::reset_mode_switch()
{
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr)
    {
        return;
    }
    c->reset_mode_switch();
}

void RC_Channels::read_mode_switch()
{
    if (!has_valid_input())
    {
        // exit immediately when no RC input
        return;
    }
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr)
    {
        return;
    }
    c->read_mode_switch();
}


// singleton instance
RC_Channels *RC_Channels::_singleton;


RC_Channels &rc()
{
    return *RC_Channels::get_singleton();
}
