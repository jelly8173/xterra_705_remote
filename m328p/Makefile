main.hex: main.elf
	avr-objcopy -O ihex -R .eeprom main.elf main.hex
main.o: main.c
	avr-gcc -Os -DF_CPU=8000000UL -mmcu=atmega328p -c -o main.o main.c
main.elf: main.o
	avr-gcc -mmcu=atmega328p main.o -o main.elf
upload: main.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 19200 -U flash:w:main.hex
	#avrdude -c usbtiny -pm328p -U flash:w:main.hex

.PHONY: clean boot lock
boot:
	avrdude -c usbtiny -pm328p -e
	avrdude -c usbtiny -pm328p -U flash:w:../boot/optiboot_atmega328_19200_8M_not_read.hex
	avrdude -c usbtiny -pm328p -U efuse:w:0xFD:m -U hfuse:w:0xDA:m -U lfuse:w:0xFF:m
lock:
#	avrdude -c usbtiny -pm328p -U lock:w:0xC7:m
	avrdude -c usbtiny -pm328p -U lock:w:0xC4:m
clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
