# SPDX-FileCopyrightText: 2021 Diego Elio Pettenò
#
# SPDX-License-Identifier: MIT
"""Example `code.py` to use SC16IS741A with a software defined clock.

This assumes that D3 of the host board is connected to the XTAL1 pin of the
SC16IS741A, with XTAL2 not connected.
"""

import time

import board
import pwmio

from flameeyes_sc16is7xx.sc16is741a import SC16IS741A

UART_CLOCK = 104_000 * 16

i2c = board.I2C()

uart_clock = pwmio.PWMOut(board.D3, frequency=UART_CLOCK, duty_cycle=0x7FFF)

uart = SC16IS741A(i2c, 0x4D, baudrate=104, fifo=True, crystal_clock=UART_CLOCK)

while True:
    print(uart.write(b"\x55"))
    print(uart.read(1))
    time.sleep(2)
    buf = b"\xAA\xEE\xAA"
    sent = 0
    while sent < len(buf):
        sent += uart.write(buf[sent:])
    print(uart.read(2))
    time.sleep(3)
