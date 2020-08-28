################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main_task/main.c 

OBJS += \
./src/main_task/main.o 

C_DEPS += \
./src/main_task/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/main_task/%.o: ../src/main_task/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	i686-pc-linux-gnu-gcc -I"E:\8.CODE\work_space\trajectory_planner\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


