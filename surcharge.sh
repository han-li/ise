#! /bin/bash

rm -rf build in_robot Makefile *_exe *.rxe
goil -t=arm/nxt --templates=$TRAMPOLINEPATH/goil/templates -g -i in_robot.oil
make -o Makefile
nexttool /COM=usb -download=in_robot_exe.rxe
