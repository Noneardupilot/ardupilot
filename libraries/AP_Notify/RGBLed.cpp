/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include "RGBLed.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

RGBLed::RGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim):
    _led_off(led_off),
    _led_bright(led_bright),
    _led_medium(led_medium),
    _led_dim(led_dim)
{

}    

bool RGBLed::init()
{
    return hw_init();
}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red != _red_curr ||
        green != _green_curr ||
        blue != _blue_curr) {
        // call the hardware update routine
        if (hw_set_rgb(red, green, blue)) {
            _red_curr = red;
            _green_curr = green;
            _blue_curr = blue;
        }
    }
}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (pNotify->_rgb_led_override) {
        // don't set if in override mode
        return;
    }
    _set_rgb(red, green, blue);
}

uint8_t RGBLed::get_brightness(void) const
{
    uint8_t brightness = _led_bright;

    switch (pNotify->_rgb_led_brightness) {
    case RGB_LED_OFF:
        brightness = _led_off;
        break;
    case RGB_LED_LOW:
        brightness = _led_dim;
        break;
    case RGB_LED_MEDIUM:
        brightness = _led_medium;
        break;
    case RGB_LED_HIGH:
        brightness = _led_bright;
        break;
    }

    // use dim light when connected through USB
    if (hal.gpio->usb_connected() && brightness > _led_dim) {
        brightness = _led_dim;
    }
    return brightness;
}



/***********************************************************************************************************************
*函数原型：uint32_t RGBLed::get_colour_sequence(void) const
*函数功能：更新LED
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息： _scheduled_update - updates _red, _green, _blue according to notify flags
*************************************************************************************************************************/

uint32_t RGBLed::get_colour_sequence(void) const
{
    // initialising pattern
    if (AP_Notify::flags.initialising)
    {
        return sequence_initialising;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                  实现Z型控制闪烁
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 记录用于植保的AB点信息
    	if(AP_Notify::flags.zigzag_record >1)//AP_Notify::flags.zigzag_record=16,记录A,//AP_Notify::flags.zigzag_record=81,记录B点信息
    	{

    		bool yellow = ((AP_Notify::flags.zigzag_record%2) == 0)?false:true;
    		switch(ab_point_step)  //1,2,3，4,5
    		{

    		case 0:
    		case 1:
    		case 2:
    		case 3:
    		case 4:

    			if(yellow)  //记录B点闪烁黄灯0，1,2,3，4,
    			{

    				return sequence_zigzag_b;
    				// yellow on

    			}
    			else  //记录A点闪烁蓝灯5,6,7,8,9
    			{
    				// blue on
    				return sequence_zigzag_a ;//闪烁blue灯
    			}
    			break;

    		case 5:
    		case 6:
    		case 7:
    		case 8:

    			// 亮一会，灭一会，实现闪烁
    			return sequence_zigzag_ab_off;
    			break;
    		case 9:
    			if(yellow)
    			{

    				AP_Notify::flags.zigzag_record /= 3;//黄灯持续闪烁4次
    			}
    			else
    			{

    				AP_Notify::flags.zigzag_record /= 2;//蓝灯闪烁四次
    			}
    			return sequence_zigzag_ab_off;
    			break;
    		}
    	}
        if((AP_Notify::flags.zigzag_record_mode)>1)
        {
        	switch(ab_point_mode_step)  //1,2,3，4,5
        	{

        		case 0:
        		case 1:
        		case 2:
        		case 3:
        		case 4:
        	           return sequence_zigzag_ab_mode; //这里主要实现闪烁两下
        	           break;
        		case 5:
        		case 6:
        		case 7:
        		case 8:
     	               return sequence_zigzag_ab_mode_off; //这里主要实现关闭
     	               break;
        		case 9:
        			   AP_Notify::flags.zigzag_record_mode/=2;
    	               return sequence_zigzag_ab_mode_off; //这里主要实现关闭
    	               break;
        	}

         }


        if((AP_Notify::flags.zigzag_record_mode_erro)>1)
        {
        	switch(ab_point_mode_step)  //1,2,3，4,5
        	{

        		case 0:
        		case 1:
        		case 2:
        		case 3:
        		case 4:
        	           return sequence_zigzag_ab_mode_erro; //这里主要实现闪烁两下
        	           break;
        		case 5:
        		case 6:
        		case 7:
        		case 8:
     	               return sequence_zigzag_ab_mode_erro_off; //这里主要实现关闭
     	               break;
        		case 9:
        			   AP_Notify::flags.zigzag_record_mode_erro/=2;
    	               return sequence_zigzag_ab_mode_erro_off; //这里主要实现关闭
    	               break;
        	}

         }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                  实现Z型控制闪烁
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                                                  实现U型控制闪烁
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // 记录用于植保的AB点信息
            	if(AP_Notify::flags.ushape_record >1)//AP_Notify::flags.zigzag_record=16,记录A,//AP_Notify::flags.zigzag_record=81,记录B点信息
            	{

            		bool yellow = ((AP_Notify::flags.ushape_record%2) == 0)?false:true;
            		switch(ab_point_step)  //1,2,3，4,5
            		{

            		case 0:
            		case 1:
            		case 2:
            		case 3:
            		case 4:

            			if(yellow)  //记录B点闪烁黄灯0，1,2,3，4,
            			{

            				return sequence_ushape_b;
            				// yellow on

            			}
            			else  //记录A点闪烁蓝灯5,6,7,8,9
            			{
            				// blue on
            				return sequence_ushape_a ;//闪烁blue灯
            			}
            			break;

            		case 5:
            		case 6:
            		case 7:
            		case 8:

            			// 亮一会，灭一会，实现闪烁
            			return sequence_ushape_ab_off;
            			break;
            		case 9:
            			if(yellow)
            			{

            				AP_Notify::flags.ushape_record /= 3;//黄灯持续闪烁4次
            			}
            			else
            			{

            				AP_Notify::flags.ushape_record /= 2;//蓝灯闪烁四次
            			}
            			return sequence_ushape_ab_off;
            			break;
            		}
            	}
                if((AP_Notify::flags.ushape_record_mode)>1)
                {
                	switch(ab_point_mode_step)  //1,2,3，4,5
                	{

                		case 0:
                		case 1:
                		case 2:
                		case 3:
                		case 4:
                	           return sequence_ushape_ab_mode; //这里主要实现闪烁两下
                	           break;
                		case 5:
                		case 6:
                		case 7:
                		case 8:
             	               return sequence_ushape_ab_mode_off; //这里主要实现关闭
             	               break;
                		case 9:
                			   AP_Notify::flags.ushape_record_mode/=2;
            	               return sequence_ushape_ab_mode_off; //这里主要实现关闭
            	               break;
                	}

                 }

                if((AP_Notify::flags.ushape_record_mode_erro)>1)
                {
                	switch(ab_point_mode_step)  //1,2,3，4,5
                	{

                		case 0:
                		case 1:
                		case 2:
                		case 3:
                		case 4:
                	           return sequence_ushape_ab_mode_erro; //这里主要实现闪烁两下
                	           break;
                		case 5:
                		case 6:
                		case 7:
                		case 8:
             	               return sequence_ushape_ab_mode_erro_off; //这里主要实现关闭
             	               break;
                		case 9:
                			   AP_Notify::flags.ushape_record_mode_erro/=2;
            	               return sequence_ushape_ab_mode_erro_off; //这里主要实现关闭
            	               break;
                	}

                 }





        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                                  实现u型控制闪烁
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // save trim and esc calibration pattern
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration)
    {
        return sequence_trim_or_esc;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio ||
        AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.ekf_bad ||
        AP_Notify::flags.gps_glitching ||
        AP_Notify::flags.leak_detected) {

        if (AP_Notify::flags.leak_detected) {
            // purple if leak detected
            return sequence_failsafe_leak;
        } else if (AP_Notify::flags.ekf_bad) {
            // red on if ekf bad
            return sequence_failsafe_ekf;
        } else if (AP_Notify::flags.gps_glitching) {
            // blue on gps glitch
            return sequence_failsafe_gps_glitching;
        }
        // all off for radio or battery failsafe
        return sequence_failsafe_radio_or_battery;
    }

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            return sequence_armed;
        }
        // solid blue if armed with no GPS lock
        return sequence_armed_nogps;
    }

    // double flash yellow if failing pre-arm checks
    if (!AP_Notify::flags.pre_arm_check) {
        return sequence_prearm_failing;
    }
    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_dgps;
    }

    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_gps;
    }

    return sequence_disarmed_bad_gps;
}




/***********************************************************************************************************************
*函数原型：void RGBLed::update_colours(void)
*函数功能：更新LED
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息： _scheduled_update - updates _red, _green, _blue according to notify flags
*************************************************************************************************************************/

void RGBLed::update_colours(void)
{
    const uint8_t brightness = get_brightness();



    //比较慢的频率--------slow rate from 50Hz to 10hz
     counter++;
     if (counter < 5)
     {

         return;
     }
     //开始复位计算------reset counter
     counter = 0;
 //    hal.uartG->printf("CHANG\r\n");

     ab_point_step++;
     ab_point_mode_step++;
     if (ab_point_step >= 10) //1,2,3，4,5, 6,7,8,9,0
     {
         ab_point_step = 0;
     }
     if (ab_point_mode_step >= 10) //1,2,3，4,5, 6,7,8,9,0
     {
     	ab_point_mode_step = 0;
     }

    const uint32_t current_colour_sequence = get_colour_sequence();

    const uint8_t step = (AP_HAL::millis()/100) % 10;

    const uint8_t colour = current_colour_sequence >> (step*3);

    _red_des = (colour & RED) ? brightness : 0;
    _green_des = (colour & GREEN) ? brightness : 0;
    _blue_des = (colour & BLUE) ? brightness : 0;
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void RGBLed::update()
{
//	hal.uartG->printf("MMM\r\n");
    if (!pNotify->_rgb_led_override)
    {
        update_colours();
//        hal.uartG->printf("NNN\r\n");
        set_rgb(_red_des, _green_des, _blue_des);
    } else
    {
        update_override();
    }
}

/*
  handle LED control, only used when LED_OVERRIDE=1
*/
void RGBLed::handle_led_control(mavlink_message_t *msg)
{
    if (!pNotify->_rgb_led_override) {
        // ignore LED_CONTROL commands if not in LED_OVERRIDE mode
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(msg, &packet);

    _led_override.start_ms = AP_HAL::millis();
    
    switch (packet.custom_len) {
    case 3:
        _led_override.rate_hz = 0;
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    case 4:
        _led_override.rate_hz = packet.custom_bytes[3];
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    default:
        // not understood
        break;
    }
}

/*
  update LED when in override mode
 */
void RGBLed::update_override(void)
{
    if (_led_override.rate_hz == 0) {
        // solid colour
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
        return;
    }
    // blinking
    uint32_t ms_per_cycle = 1000 / _led_override.rate_hz;
    uint32_t cycle = (AP_HAL::millis() - _led_override.start_ms) % ms_per_cycle;
    if (cycle > ms_per_cycle / 2) {
        // on
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
    } else {
        _set_rgb(0, 0, 0);
    }
}
