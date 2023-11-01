# FF_A6lib
Fully asynchronous SMS send/receive in PDU mode for A6/GA6 modem class

## What's for?
This class allows asynchronously sending/receiving SMS using an A6 or GA6 (and probably others) modem using PDU mode.

Messages are in UTF-8 format and automatically converted into GSM7 or UCS2, and split in multiple messages if needed.

A callback routine in your program will be called each time a SMS is received.

You also may send SMS directly.

By default, logging/debugging is done through FF_TRACE macros, allowing to easily change code.

It may also be used with FF_WebServer class, as containing routines to map with it.

You may have a look at https://github.com/FlyingDomotic/FF_SmsServer which shows how to use it

## Prerequisites

Can be used directly with Arduino IDE or PlatformIO.

## Installation

Clone repository somewhere on your disk.
```
cd <where_you_want_to_install_it>
git clone https://github.com/FlyingDomotic/FF_A6lib.git FF_A6lib
```

Note that <where_you_want_to_install_it> could ingeniously be your compiler libraries folder ;-)

## Update

Go to code folder and pull new version:
```
cd <where_you_installed_FF_A6lib>
git pull
```

Note: if you did any changes to files and `git pull` command doesn't work for you anymore, you could stash all local changes using:
```
git stash
```
or
```
git checkout <modified file>
```

## Documentation

Documentation could be built using doxygen, either using makedoc.sh (for Linux) or makedoc.bat (for Windows), or running
```
doxygen doxyfile
```

HTML and RTF versions will then be available in `documentation` folder.
