################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include freeRTOS/src/subdir.mk
-include Example/src/subdir.mk
-include Acelerometro/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: FRTOS_ACELEROMETRO.axf

dependents:
	-cd D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug && $(MAKE) all

# Tool invocations
FRTOS_ACELEROMETRO.axf: $(OBJS) $(USER_OBJS) D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug/liblpc_chip_175x_6x.a
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\Facultad\Digitales II\Repositorio MCU\lpc_chip_175x_6x\Debug" -Xlinker -Map="FRTOS_ACELEROMETRO.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "FRTOS_Debug.ld" -o "FRTOS_ACELEROMETRO.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) FRTOS_ACELEROMETRO.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "FRTOS_ACELEROMETRO.axf"; # arm-none-eabi-objcopy -v -O binary "FRTOS_ACELEROMETRO.axf" "FRTOS_ACELEROMETRO.bin" ; # checksum -p LPC1769 -d "FRTOS_ACELEROMETRO.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug/liblpc_chip_175x_6x.a:

-include ../makefile.targets
