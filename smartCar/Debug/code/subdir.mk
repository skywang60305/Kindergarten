################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/BMX055.c \
../code/CarState.c \
../code/SAMURAI.c \
../code/VCAN_VisualScope.c \
../code/VOFA.c \
../code/accessibility.c \
../code/adrc.c \
../code/attitude_calculation.c \
../code/control.c \
../code/deal_img.c \
../code/go.c \
../code/img2sd.c \
../code/key.c \
../code/lcd.c \
../code/lcd_menu.c \
../code/motor.c \
../code/motorControlAlgorithm.c \
../code/servo.c \
../code/speed.c \
../code/spi_sd.c \
../code/stack.c \
../code/switch.c \
../code/wdog.c 

COMPILED_SRCS += \
./code/BMX055.src \
./code/CarState.src \
./code/SAMURAI.src \
./code/VCAN_VisualScope.src \
./code/VOFA.src \
./code/accessibility.src \
./code/adrc.src \
./code/attitude_calculation.src \
./code/control.src \
./code/deal_img.src \
./code/go.src \
./code/img2sd.src \
./code/key.src \
./code/lcd.src \
./code/lcd_menu.src \
./code/motor.src \
./code/motorControlAlgorithm.src \
./code/servo.src \
./code/speed.src \
./code/spi_sd.src \
./code/stack.src \
./code/switch.src \
./code/wdog.src 

C_DEPS += \
./code/BMX055.d \
./code/CarState.d \
./code/SAMURAI.d \
./code/VCAN_VisualScope.d \
./code/VOFA.d \
./code/accessibility.d \
./code/adrc.d \
./code/attitude_calculation.d \
./code/control.d \
./code/deal_img.d \
./code/go.d \
./code/img2sd.d \
./code/key.d \
./code/lcd.d \
./code/lcd_menu.d \
./code/motor.d \
./code/motorControlAlgorithm.d \
./code/servo.d \
./code/speed.d \
./code/spi_sd.d \
./code/stack.d \
./code/switch.d \
./code/wdog.d 

OBJS += \
./code/BMX055.o \
./code/CarState.o \
./code/SAMURAI.o \
./code/VCAN_VisualScope.o \
./code/VOFA.o \
./code/accessibility.o \
./code/adrc.o \
./code/attitude_calculation.o \
./code/control.o \
./code/deal_img.o \
./code/go.o \
./code/img2sd.o \
./code/key.o \
./code/lcd.o \
./code/lcd_menu.o \
./code/motor.o \
./code/motorControlAlgorithm.o \
./code/servo.o \
./code/speed.o \
./code/spi_sd.o \
./code/stack.o \
./code/switch.o \
./code/wdog.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/pxy/SmartCarProject/smartCar/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/BMX055.d ./code/BMX055.o ./code/BMX055.src ./code/CarState.d ./code/CarState.o ./code/CarState.src ./code/SAMURAI.d ./code/SAMURAI.o ./code/SAMURAI.src ./code/VCAN_VisualScope.d ./code/VCAN_VisualScope.o ./code/VCAN_VisualScope.src ./code/VOFA.d ./code/VOFA.o ./code/VOFA.src ./code/accessibility.d ./code/accessibility.o ./code/accessibility.src ./code/adrc.d ./code/adrc.o ./code/adrc.src ./code/attitude_calculation.d ./code/attitude_calculation.o ./code/attitude_calculation.src ./code/control.d ./code/control.o ./code/control.src ./code/deal_img.d ./code/deal_img.o ./code/deal_img.src ./code/go.d ./code/go.o ./code/go.src ./code/img2sd.d ./code/img2sd.o ./code/img2sd.src ./code/key.d ./code/key.o ./code/key.src ./code/lcd.d ./code/lcd.o ./code/lcd.src ./code/lcd_menu.d ./code/lcd_menu.o ./code/lcd_menu.src ./code/motor.d ./code/motor.o ./code/motor.src ./code/motorControlAlgorithm.d ./code/motorControlAlgorithm.o ./code/motorControlAlgorithm.src ./code/servo.d ./code/servo.o ./code/servo.src ./code/speed.d ./code/speed.o ./code/speed.src ./code/spi_sd.d ./code/spi_sd.o ./code/spi_sd.src ./code/stack.d ./code/stack.o ./code/stack.src ./code/switch.d ./code/switch.o ./code/switch.src ./code/wdog.d ./code/wdog.o ./code/wdog.src

.PHONY: clean-code

