# Makefile for FreeRTOS Project

CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -Wall -Wextra -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IFreeRTOS/Config -IFreeRTOS/Source
LDFLAGS = -TSTM32F4xx.ld -Wl,--gc-sections
SRC = Core/Src/main.c Core/Src/freertos.c Core/Src/PID.c Core/Src/queue.c Drivers/STM32F4xx_HAL_Driver/Src/*.c FreeRTOS/Source/*.c
OBJ = $(SRC:.c=.o)
TARGET = freertos-project.elf

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean