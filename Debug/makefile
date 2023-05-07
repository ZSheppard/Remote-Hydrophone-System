################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Remote_Hydrophone_Sys_Rtos
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Remote_Hydrophone_Sys_Rtos.elf \

MAP_FILES += \
Remote_Hydrophone_Sys_Rtos.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Remote_Hydrophone_Sys_Rtos.list \

OBJCOPY_BIN += \
RTOS_Hydrophone.bin \


# All Target
all: main-build

# Main-build Target
main-build: Remote_Hydrophone_Sys_Rtos.elf secondary-outputs

# Tool invocations
<<<<<<< HEAD
Remote_Hydrophone_Sys_Rtos.elf Remote_Hydrophone_Sys_Rtos.map: $(OBJS) $(USER_OBJS) /Users/faran/Local-Repo/STM32/Capstone/Remote_Hydrophone_Sys_Rtos/STM32H7A3ZITXQ_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Remote_Hydrophone_Sys_Rtos.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/Users/faran/Local-Repo/STM32/Capstone/Remote_Hydrophone_Sys_Rtos/STM32H7A3ZITXQ_FLASH.ld" --specs=nosys.specs -Wl,-Map="Remote_Hydrophone_Sys_Rtos.map" -Wl,--gc-sections -static -L"/Users/faran/Local-Repo/STM32/Capstone/Remote_Hydrophone_Sys_Rtos/Drivers/CMSIS/Lib/GCC" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
=======
RTOS_Hydrophone.elf RTOS_Hydrophone.map: $(OBJS) $(USER_OBJS) C:\Users\sfinl\STM32CubeIDE\workspace_1.11.0\Hydrophone\ Microcontroller\Remote_Hydrophone_Sys_Rtos\STM32H7A3ZITXQ_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "RTOS_Hydrophone.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\sfinl\STM32CubeIDE\workspace_1.11.0\Hydrophone Microcontroller\Remote_Hydrophone_Sys_Rtos\STM32H7A3ZITXQ_FLASH.ld" --specs=nosys.specs -Wl,-Map="RTOS_Hydrophone.map" -Wl,--gc-sections -static -L"C:\Users\sfinl\STM32CubeIDE\workspace_1.11.0\Hydrophone Microcontroller\Remote_Hydrophone_Sys_Rtos\Drivers\CMSIS\Lib\GCC" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
>>>>>>> b6a4c61421419d35c6a5e304560b93442d838763
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Remote_Hydrophone_Sys_Rtos.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Remote_Hydrophone_Sys_Rtos.list"
	@echo 'Finished building: $@'
	@echo ' '

RTOS_Hydrophone.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RTOS_Hydrophone.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
<<<<<<< HEAD
	-$(RM) Remote_Hydrophone_Sys_Rtos.elf Remote_Hydrophone_Sys_Rtos.list Remote_Hydrophone_Sys_Rtos.map default.size.stdout
=======
	-$(RM) RTOS_Hydrophone.bin RTOS_Hydrophone.elf RTOS_Hydrophone.list RTOS_Hydrophone.map default.size.stdout
>>>>>>> b6a4c61421419d35c6a5e304560b93442d838763
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
