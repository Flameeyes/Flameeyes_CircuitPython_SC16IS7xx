# SPDX-FileCopyrightText: 2021 Diego Elio Pettenò
#
# SPDX-License-Identifier: MIT

import time

import serial

with serial.Serial("/dev/ttyUSB0", 104) as ser:
    while True:
        for char in list(b'abcdefghijklmnopqrstuvwxyz'):
            ser.write([char])
            time.sleep(1)
