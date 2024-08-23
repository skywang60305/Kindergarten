################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/VOFA.c \
../code/deal_img.c \
../code/img2sd.c \
../code/key.c \
../code/lcd.c \
../code/lcd_menu.c \
../code/spi_sd.c \
../code/switch.c 

COMPILED_SRCS += \
./code/VOFA.src \
./code/deal_img.src \
./code/img2sd.src \
./code/key.src \
./code/lcd.src \
./code/lcd_menu.src \
./code/spi_sd.src \
./code/switch.src 

C_DEPS += \
./code/VOFA.d \
./code/deal_img.d \
./code/img2sd.d \
./code/key.d \
./code/lcd.d \
./code/lcd_menu.d \
./code/spi_sd.d \
./code/switch.d 

OBJS += \
./code/VOFA.o \
./code/deal_img.o \
./code/img2sd.o \
./code/key.o \
./code/lcd.o \
./code/lcd_menu.o \
./code/spi_sd.o \
./code/switch.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/10603/Desktop/test15/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
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
	-$(RM) ./code/VOFA.d ./code/VOFA.o ./code/VOFA.src ./code/deal_img.d ./code/deal_img.o ./code/deal_img.src ./code/img2sd.d ./code/img2sd.o ./code/img2sd.src ./code/key.d ./code/key.o ./code/key.src ./code/lcd.d ./code/lcd.o ./code/lcd.src ./code/lcd_menu.d ./code/lcd_menu.o ./code/lcd_menu.src ./code/spi_sd.d ./code/spi_sd.o ./code/spi_sd.src ./code/switch.d ./code/switch.o ./code/switch.src

.PHONY: clean-code

