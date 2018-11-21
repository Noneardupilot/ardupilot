# find the mk/ directory, which is where this makefile fragment
# lives. (patsubst strips the trailing slash.)
#将系统的的类型名称赋值给SYSTYPE（uname 是查找电脑全部信息，
#包括处理器信息、操作系统信息等等）
SYSTYPE			:=	$(shell uname)

$(warning $(SYSTYPE))
#判断系统类型是否是CYGWIN（Cygwin 是一个用于 Windows 的类 UNIX shell）
ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  MK_DIR := $(shell cygpath -m ../mk)
$(warning "***********")
else
  MK_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
$(warning $(SYSTYPE))
endif


$(warning $(MK_DIR))
#转去执行mk/environ.mk文件，执行完后再转回来继续执行apm.mk文件
include $(MK_DIR)/environ.mk


$(warning $(MAKECMDGOALS))
# short-circuit build for the configure target
ifeq ($(MAKECMDGOALS),configure)
include $(MK_DIR)/configure.mk
else

# short-circuit build for the help target
include $(MK_DIR)/help.mk

# common makefile components
include $(MK_DIR)/targets.mk
include $(MK_DIR)/sketch_sources.mk
include $(SKETCHBOOK)/modules/uavcan/libuavcan/include.mk

ifneq ($(MAKECMDGOALS),clean)

# board specific includes
ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
include $(MK_DIR)/board_native.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_LINUX)
include $(MK_DIR)/board_linux.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_PX4)
	
include $(MK_DIR)/board_px4.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_VRBRAIN)
include $(MK_DIR)/board_vrbrain.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_F4LIGHT)

include $(MK_DIR)/board_F4Light.mk
endif

endif

endif
