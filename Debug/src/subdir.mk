################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ImgProcessing.cpp \
../src/cameraFrameWork.cpp \
../src/main.cpp \
../src/my_tools.cpp 

OBJS += \
./src/ImgProcessing.o \
./src/cameraFrameWork.o \
./src/main.o \
./src/my_tools.o 

CPP_DEPS += \
./src/ImgProcessing.d \
./src/cameraFrameWork.d \
./src/main.d \
./src/my_tools.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -I/usr/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


