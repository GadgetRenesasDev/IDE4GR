#/****************************************************************
#KPIT Cummins Infosystems Limited Pune, India. 24 Aug 2011.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#*****************************************************************/

PREFIX=rl78-elf

DEBUG_FLAGS=-gdwarf-2

# Specify compiler to be used
CC = $(PREFIX)-gcc

# Specify Assembler to be used
AS = $(PREFIX)-as

# Specify linker to be used
LD = $(PREFIX)-ld

# Specify CPU flag
CPU =

# Common compiler flags
CFLAGS =

ALL_FLAGS = $(CFLAGS) $(DEBUG_FLAGS) $(CPU)


# Application name
#APPNAME = 

# Specify all objects that you are going to link together
OBJS	=	 main.o
SOURCES =	 main.c

RL78G13_led_flasher.mot : RL78G13_led_flasher.out
	$(PREFIX)-objcopy -O srec RL78G13_led_flasher.out $@

# Define ROMSTART if compiling for ROM
RL78G13_led_flasher.out:$(SOURCES)  makefile
	$(CC) $(ALL_FLAGS) -Xlinker -Map -Xlinker RL78G13t_led_flasher.map -o $@ $(SOURCES) -lm
	
all:RL78G13_led_flasher.mot
rom:RL78G13_led_flasher.mot

clean:
	rm -f *.o *.out *.mot *.map



