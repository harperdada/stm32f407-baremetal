################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/core.c \
../Src/delay.c \
../Src/fonts.c \
../Src/i2c.c \
../Src/main.c \
../Src/spi.c \
../Src/st7789.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/core.o \
./Src/delay.o \
./Src/fonts.o \
./Src/i2c.o \
./Src/main.o \
./Src/spi.o \
./Src/st7789.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/core.d \
./Src/delay.d \
./Src/fonts.d \
./Src/i2c.d \
./Src/main.d \
./Src/spi.d \
./Src/st7789.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE=STM32F411xE -c -I"/Users/hussamaldean/STM32CubeIDE/workspace_1.7.0/embeddedExpertIO_posts/ST7789_IPS_TFT/Inc" -I"/Users/hussamaldean/STM32CubeIDE/workspace_1.7.0/embeddedExpertIO_posts/ST7789_IPS_TFT/F4_chip_headers/CMSIS/Include" -I"/Users/hussamaldean/STM32CubeIDE/workspace_1.7.0/embeddedExpertIO_posts/ST7789_IPS_TFT/F4_chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/core.d ./Src/core.o ./Src/core.su ./Src/delay.d ./Src/delay.o ./Src/delay.su ./Src/fonts.d ./Src/fonts.o ./Src/fonts.su ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/st7789.d ./Src/st7789.o ./Src/st7789.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

