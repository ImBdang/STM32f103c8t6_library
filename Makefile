#project name
PROJECT	:= control

#Kien truc
MCU		:= cortex-m3

#linker and startup file
LD_SCRIPT	:= stm32f103c8.ld
STARTUP_FILE:= startup_stm32f103c8.c


###################################
#			TOOLCHAINS			
###################################
TOOLCHAINS_PATH 	:= /home/bdang/toolchains/gcc-arm-none-eabi-14.3
PREFIX_TOOLCHAINS	:= arm-none-eabi
GCC					:= $(PREFIX_TOOLCHAINS)-gcc
CP					:= $(PREFIX_TOOLCHAINS)-objcopy
ASM 				:= $(PREFIX_TOOLCHAINS)-as


####################################
#			THU MUC
####################################
SRC_DIR				:= src
INC_DIR 			:= inc 
BUILD_DIR			:= build
STARTUP_DIR		 	:= startup
LINKER_DIR			:= linker


####################################
#	  SOURCE & OBJECTS FILES
####################################
SRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJECTS	  := $(SRC_FILES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

####################################
#			FLAGS
####################################
CFLAGS	:= -mcpu=$(MCU) -mthumb -Wall -O2 -g -ffreestanding -nostdlib -I$(INC_DIR)
LDFLAGS := -T$(LINKER_DIR)/$(LD_SCRIPT) -nostdlib
 

####################################
#			RULES
####################################

#Rule main
main: $(BUILD_DIR)/$(PROJECT).elf
bin: $(BUILD_DIR)/$(PROJECT).bin
hex: $(BUILD_DIR)/$(PROJECT).hex


#ELF
$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS)
	$(TOOLCHAINS_PATH)/bin/$(GCC) $(CFLAGS) $^ $(LDFLAGS) -o $@

#Bien dich .c sang .o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p build
	$(TOOLCHAINS_PATH)/bin/$(GCC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(TOOLCHAINS_PATH)/bin/$(CP) -O binary $< $@

$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf
	$(TOOLCHAINS_PATH)/bin/$(CP) -O ihex $< $@


clean:
	@rm -rf $(BUILD_DIR)


flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash write build/$(PROJECT).bin 0x8000000