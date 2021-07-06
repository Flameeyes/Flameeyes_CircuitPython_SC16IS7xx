# SPDX-FileCopyrightText: 2021 Diego Elio Pettenò
#
# SPDX-License-Identifier: MIT

import time

import board

from flameeyes_sc16is7xx.sc16is741a import SC16IS741A

i2c = board.I2C()

uart = SC16IS741A(i2c, 0x4d, baudrate=104, fifo=True)

while True:
    print(uart.write(b'\x55'))
    print(uart.read(1))
    time.sleep(2)
    buf = b'\xAA\xEE\xAA'
    sent = 0
    while sent < len(buf):
        sent += uart.write(buf[sent:])
    print(uart.read(2))
    time.sleep(3)
