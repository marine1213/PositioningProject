################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../cmake/CMakeFiles/feature_tests.cxx 

C_SRCS += \
../cmake/CMakeFiles/feature_tests.c 

CXX_DEPS += \
./cmake/CMakeFiles/feature_tests.d 

OBJS += \
./cmake/CMakeFiles/feature_tests.o 

C_DEPS += \
./cmake/CMakeFiles/feature_tests.d 


# Each subdirectory must supply rules for building sources it contributes
cmake/CMakeFiles/%.o: ../cmake/CMakeFiles/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

cmake/CMakeFiles/%.o: ../cmake/CMakeFiles/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


