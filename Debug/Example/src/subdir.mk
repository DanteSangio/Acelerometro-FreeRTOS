################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Example/src/FRTOS.c \
../Example/src/cr_startup_lpc175x_6x.c \
../Example/src/crp.c \
../Example/src/sysinit.c 

OBJS += \
./Example/src/FRTOS.o \
./Example/src/cr_startup_lpc175x_6x.o \
./Example/src/crp.o \
./Example/src/sysinit.o 

C_DEPS += \
./Example/src/FRTOS.d \
./Example/src/cr_startup_lpc175x_6x.d \
./Example/src/crp.d \
./Example/src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
Example/src/%.o: ../Example/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -DNO_BOARD_LIB -D__LPC17XX__ -D__REDLIB__ -I"D:\Facultad\Digitales II\Repositorio MCU\FRTOS_ACELEROMETRO\Acelerometro\Inc" -I"D:\Facultad\Digitales II\Repositorio MCU\lpc_chip_175x_6x" -I"D:\Facultad\Digitales II\Repositorio MCU\FRTOS\Example\inc" -I"D:\Facultad\Digitales II\Repositorio MCU\lpc_chip_175x_6x\inc" -I"D:\Facultad\Digitales II\Repositorio MCU\FRTOS\freeRTOS\inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


