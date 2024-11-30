################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/tim/Documents/GitHub/IMU_Interface/Common/BNO086_SPI/BNO086_SPI.c 

OBJS += \
./Common/BNO086_SPI/BNO086_SPI.o 

C_DEPS += \
./Common/BNO086_SPI/BNO086_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Common/BNO086_SPI/BNO086_SPI.o: /home/tim/Documents/GitHub/IMU_Interface/Common/BNO086_SPI/BNO086_SPI.c Common/BNO086_SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I"/home/tim/Documents/GitHub/IMU_Interface/Common" -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tim/Documents/GitHub/IMU_Interface/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-BNO086_SPI

clean-Common-2f-BNO086_SPI:
	-$(RM) ./Common/BNO086_SPI/BNO086_SPI.cyclo ./Common/BNO086_SPI/BNO086_SPI.d ./Common/BNO086_SPI/BNO086_SPI.o ./Common/BNO086_SPI/BNO086_SPI.su

.PHONY: clean-Common-2f-BNO086_SPI

