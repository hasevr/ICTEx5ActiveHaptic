#!/bin/sh
case "$1" in
    "app-run") 
    make app&&make app-flash&&make monitor
    ;;
    "monitor") 
	make monitor
    ;;
    "app-flash") 
	make app-flash
    ;;
    "openocd") 
    taskkill -f -im openocd.exe 
#    /mingw32.exe openocd.exe -s /mingw32/share/openocd/scripts -f /mingw32/share/openocd/scripts/interface/ftdi/um232h.cfg -f /mingw32/share/openocd/scripts/board/esp-wroom-32.cfg
    openocd.exe -s /mingw32/share/openocd/scripts -f /mingw32/share/openocd/scripts/interface/ftdi/um232h.cfg -f /mingw32/share/openocd/scripts/board/esp-wroom-32.cfg
    ;;
esac
