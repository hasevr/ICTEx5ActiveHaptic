#!/bin/sh
case "$1" in
    "app-build") 
    make app -j4
    ;;
    "app-run") 
    make app -j4&&make app-flash -j4&&make monitor -j4
    ;;
    "monitor") 
	make monitor -j4
    ;;
    "app-flash") 
	make app-flash -j4
    ;;
    "flash")
	make flash -j4
    ;;
esac
