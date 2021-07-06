# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2019 Carter Nelson
# SPDX-FileCopyrightText: 2020 Facebook Inc.
# SPDX-FileCopyrightText: 2021 Diego Elio Pettenò
#
# SPDX-License-Identifier: MIT

import time

from micropython import const
from adafruit_bus_device import i2c_device

# Pre-shift register addresses by three bits. Both I2C and SPI addressing for
# the registers in this part number has three trailing zero bits.
#
# Beware! Dual-UART part numbers actually include the channel information as
# part of those three bits!
REG_RHR = const(0x00 << 3)
REG_THR = const(0x00 << 3)
REG_IER = const(0x01 << 3)
REG_FCR = const(0x02 << 3)
REG_LCR = const(0x03 << 3)
REG_MCR = const(0x04 << 3)
REG_LSR = const(0x05 << 3)
REG_MSR = const(0x06 << 3)
REG_SPR = const(0x07 << 3)
REG_TCR = const(0x06 << 3)
REG_TLR = const(0x07 << 3)
REG_TXLVL = const(0x08 << 3)
REG_RXLVL = const(0x09 << 3)
REG_RESET = const(0x0E << 3)
REG_EFCR = const(0x0F << 3)
REG_DLL = const(0x00 << 3)
REG_DLH = const(0x01 << 3)
REG_EFR = const(0x02 << 3)
REG_XON1 = const(0x04 << 3)
REG_XON2 = const(0x05 << 3)
REG_XOFF1 = const(0x06 << 3)
REG_XOFF2 = const(0x06 << 3)


# Global buffer for reading and writing registers with the devices.  This is
# shared between all implementation classes to reduce memory allocations.
# However this is explicitly not thread safe or re-entrant by design!
_BUFFER = bytearray(65)

PARITY_NONE = 0
PARITY_ODD = 1
PARITY_EVEN = 3


class SC16IS741A:
    def __init__(
        self,
        i2c,
        address: int,
        *,
        baudrate: int = 9600,
        bits: int = 8,
        parity: int = None,
        stop: int = 1,
        timeout: float = 1,
        crystal_clock: int = 1_843_200,
        fifo: bool = False,
    ):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)

        self._write_reg8(REG_FCR, 0x06)
        self._write_reg8(REG_IER, 0x80 | 0x40 | 0x20)

        if parity is None:
            parity = PARITY_NONE

        if parity not in (PARITY_NONE, PARITY_EVEN, PARITY_ODD):
            raise ValueError(
                "Value {parity:2x} is not valid for parity parameter".format(
                    parity=parity
                )
            )

        if stop not in (1, 2):
            raise ValueError(
                "Value {stop} is not valid for stop parameter".format(stop=stop)
            )

        if bits not in (5, 6, 7, 8):
            raise ValueError(
                "Value {bits} is not valid for bits parameter".format(bits=bits)
            )

        lcr_value = (parity << 3) | (stop - 1) << 2 | (bits - 5) << 0

        self.timeout = timeout

        self._fifo = fifo
        fcr_value = 0
        if self._fifo:
            fcr_value |= 0x01

        mcr_value = 0

        # Try first without having to set prescaler
        divisor = crystal_clock / (16 * baudrate)
        if divisor > 0xFFFF:
            mcr_value |= 0x80  # set prescaler = 4
            divisor /= 4

        divisor = int(divisor)
        if divisor > 0xFFFF or divisor == 0:
            raise ValueError(
                "Unable to configure baud rate to {baudrate} with a crystal of {crystal_clock}".format(
                    baudrate=baudrate, crystal_clock=crystal_clock
                )
            )

        self._write_reg8(REG_LCR, 0x80)
        self._write_reg8(REG_DLH, divisor >> 8)
        self._write_reg8(REG_DLL, divisor & 0xFF)
        self._write_reg8(REG_LCR, lcr_value)
        self._write_reg8(REG_MCR, mcr_value)
        self._write_reg8(REG_FCR, fcr_value)

    def write(self, buf: bytes) -> int:
        space = min(len(buf), self.txlvl)
        if space:
            self._write_reg_bytes(REG_THR, buf[:space])
        return space

    def read(self, nbytes=None) -> bytes:
        end_time = time.monotonic() + self.timeout
        read_buffer = b""
        while (nbytes is None or nbytes > 0) and time.monotonic() < end_time:
            rxlvl = self.rxlvl
            space = min(nbytes, rxlvl) if nbytes is not None else rxlvl
            if space > 0:
                new_read = self._read_reg_bytes(REG_RHR, space)
                read_buffer += new_read
                if nbytes is not None:
                    nbytes -= len(new_read)

        return read_buffer

    @property
    def txlvl(self) -> int:
        if self._fifo:
            return self._read_reg(REG_TXLVL)
        else:
            return 1 if self._read_reg(REG_LSR) & 0x20 else 0

    @property
    def rxlvl(self) -> int:
        return self._read_reg(REG_RXLVL)

    def _read_reg(self, register):
        # Read an unsigned 8 bit value from the specified 8-bit register.
        with self.i2c_device as i2c:
            _BUFFER[0] = register

            i2c.write_then_readinto(_BUFFER, _BUFFER, out_end=1, in_start=1, in_end=2)
            return _BUFFER[1]

    def _read_reg_bytes(self, register, nbytes):
        assert nbytes <= 64

        with self.i2c_device as i2c:
            _BUFFER[0] = register

            i2c.write_then_readinto(
                _BUFFER, _BUFFER, out_end=1, in_start=1, in_end=nbytes + 1
            )
            return _BUFFER[1 : nbytes + 1]

    def _write_reg8(self, register, val):
        # Write an 8 bit value to the specified 8-bit register.
        with self.i2c_device as i2c:
            _BUFFER[0] = register
            _BUFFER[1] = val & 0xFF
            i2c.write(_BUFFER, end=2)

    def _write_reg_bytes(self, register, buf):
        assert len(buf) <= 64

        with self.i2c_device as i2c:
            _BUFFER[0] = register
            _BUFFER[1 : len(buf) + 1] = buf
            i2c.write(_BUFFER, end=len(buf) + 1)
