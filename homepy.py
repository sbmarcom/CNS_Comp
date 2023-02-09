#!/usr/bin/env python3
import spidev
import hal
import time
import sys
import traceback
import configparser
import linuxcnc
import struct
import os


h = linuxcnc.command()
h.home(1)
h.home(2)
h.home(0)

