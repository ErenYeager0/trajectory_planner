################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/axis_planner/axis_state_machine.c 

OBJS += \
./src/axis_planner/axis_state_machine.o 

C_DEPS += \
./src/axis_planner/axis_state_machine.d 


# Each subdirectory must supply rules for building sources it contributes
src/axis_planner/%.o: ../src/axis_planner/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	i686-pc-linux-gnu-gcc -I"E:\8.CODE\work_space\trajectory_planner\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


