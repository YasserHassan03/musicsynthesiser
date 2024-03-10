
all:
	pio -f -c vim run

download:
	pio -f -c vim run --target upload

clean:
	pio -f -c vim run --target clean

program:
		pio -f -c vim run --target program

uploadfs:
		pio -f -c vim run --target uploadfs

update:
		pio -f -c vim update

refresh_lsp:
	rm -rf compile_commands.json
	pio run -t compiledb

disass:
	arm-linux-gnueabi-objdump -D .pio/build/nucleo_l432kc/firmware.elf > diass.s	
