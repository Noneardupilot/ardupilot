#include "Copter.h"


/***********************************************************************************************************************
*函数原型：void Copter::init_capabilities(void)
*函数功能：初始化设备性能
*修改日期：2018-10-26
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::init_capabilities(void)
{
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                               MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION);
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_TERRAIN);
#endif
}
