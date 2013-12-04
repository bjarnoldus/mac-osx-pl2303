#!/bin/sh
cp -r ./build/Release-10.4-universal/osx-pl2303.kext /tmp
cd /tmp
kextload osx-pl2303.kext
#/Users/Jeroen/Projects/Xcode/SerialTest/build/Release/SerialTest
#avrdude -p m128 -c avrisp2 -P /dev/tty.PL2303-151
#kextunload osx-pl2303.kext

