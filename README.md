# STM32F746G-Discovery empty ARM GCC Template
This project is a clean slate for further development on the STM32F746G-Discovery board. With minimal files for linking and initialization of the board, no HAL.

## Build
Simply run the folowing commands to build or clean the project:
```
$ make
$ make clean
```

## Flashing

Use OPENOCD for flashing. It works with OpenOCD 0.10.0+dev-00587-g63fcef49 from [OpenOCD](https://repo.or.cz/w/openocd.git) and STLink v1.5.1 for linux [STLink - Linux](https://github.com/texane/stlink).

To flash the board issue the following:
```
$ OpenOCD -f interface/stlink.cfg -f board/stm32f7discovery.cfg \
	  -c "init" \
	  -c "reset init" \
	  -c "flash probe 0" \
	  -c "flash info 0" \
	  -c "flash write_image erase stm32746g-eval.bin 0x08000000" \
	  -c "reset run" \
	  -c "shutdown"
```

## Handboning functionalities

This project contain 7 types of handboning functionalities, use with caution!
