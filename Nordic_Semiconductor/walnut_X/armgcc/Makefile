PROJECT_NAME := ble_app_gls_s130_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ../../nRF5_SDK_11.0.0_89a8197/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/button/app_button.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/app_error.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/app_error_weak.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fifo/app_fifo.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/timer/app_timer.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/app_util_platform.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/crc16/crc16.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fds/fds.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fstorage/fstorage.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/nrf_assert.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/nrf_log.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/uart/retarget.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util/sdk_mapped_flags.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/sensorsim/sensorsim.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/external/segger_rtt/RTT_Syscalls_GCC.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/external/segger_rtt/SEGGER_RTT.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/external/segger_rtt/SEGGER_RTT_printf.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/uart/app_uart_fifo.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/delay/nrf_delay.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/pstorage/pstorage.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/twi/app_twi.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/twi_master/nrf_drv_twi.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/examples/bsp/bsp.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/examples/bsp/bsp_btn_ble.c) \
$(abspath ../main.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/common/ble_advdata.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_advertising/ble_advertising.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_bas/ble_bas.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/common/ble_conn_params.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/common/ble_conn_state.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_dis/ble_dis.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_gls/ble_gls.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_gls/ble_gls_db.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_racp/ble_racp.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_radio_notification/ble_radio_notification.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/common/ble_srv_common.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/gatt_cache_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/gattc_cache_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/gatts_cache_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/id_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/peer_data.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/peer_data_storage.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/peer_database.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/peer_id.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/peer_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/pm_buffer.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/pm_mutex.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/security_dispatcher.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager/security_manager.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/toolchain/system_nrf51.c) \
$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath ../sensors.c) \
$(abspath ../w_ess_advertising.c) \
$(abspath ../Si7020.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath ../../nRF5_SDK_11.0.0_89a8197/components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS  = -I$(abspath ../)
INC_PATHS += -I$(abspath ../config/ble_app_gls_s130_pca10028)
INC_PATHS += -I$(abspath ../config)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/config)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/timer)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fifo)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fstorage/config)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fds/config)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/softdevice/s130/headers)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/delay)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/crc16)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/util)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/twi)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/twi_master)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/uart)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/trace)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/common)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/sensorsim)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/peer_manager)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_dis)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/device)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/uart)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_gls)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/button)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fstorage)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/experimental_section_vars)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/external/segger_rtt)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/examples/bsp)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/toolchain/CMSIS/Include)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/hal)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_racp)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/toolchain/gcc)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/toolchain)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/libraries/fds)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/drivers_nrf/common)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_advertising)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/softdevice/s130/headers/nrf51)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_services/ble_bas)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath ../../nRF5_SDK_11.0.0_89a8197/components/ble/ble_radio_notification)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF_LOG_USES_RTT=1
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
CFLAGS += --specs=nano.specs -u _scanf_float -u _printf_float
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF_LOG_USES_RTT=1
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0

#default target - first one defined
default: clean nrf51422_xxac_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s130
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s130: OUTPUT_FILENAME := nrf51422_xxac_s130
nrf51422_xxac_s130: LINKER_SCRIPT=ble_app_walnutX_gcc_nrf51_AA.ld

nrf51422_xxac_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flash: nrf51422_xxac_s130
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset -f nrf51

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.0_softdevice.hex
	nrfjprog --program ../../../../../../components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex -f nrf51 --chiperase
	nrfjprog --reset -f nrf51