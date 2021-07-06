# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2019 Carter Nelson
# SPDX-FileCopyrightText: 2020 Facebook Inc.
# SPDX-FileCopyrightText: 2021 Diego Elio Pettenò
#
# SPDX-License-Identifier: MIT

import time

from micropython import const

from adafruit_register import i2c_bit, i2c_bits
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
        self._crystal_clock = crystal_clock

        if parity is None:
            parity = PARITY_NONE

        self.parity = parity
        self.stop_bits = stop
        self.word_length = bits
        self.fifo_enable = fifo
        self.baudrate = baudrate
        self.timeout = timeout

        self.rx_fifo_reset = True
        self.tx_fifo_reset = True

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

    rxlvl = i2c_bits.ROBits(8, REG_RXLVL, 0)
    _txlvl = i2c_bits.ROBits(8, REG_TXLVL, 0)
    thr_empty = i2c_bit.ROBit(REG_LSR, 5)

    @property
    def txlvl(self) -> int:
        if self.fifo_enable:
            return self._txlvl
        else:
            return 1 if self.thr_empty else 0

    rx_trigger_level = i2c_bits.RWBits(2, REG_FCR, 6)  # WO
    tx_trigger_level = i2c_bits.RWBits(2, REG_FCR, 4)  # WO
    tx_fifo_reset = i2c_bit.RWBit(REG_FCR, 2)  # WO
    rx_fifo_reset = i2c_bit.RWBit(REG_FCR, 1)  # WO
    fifo_enable = i2c_bit.RWBit(REG_FCR, 0)  # WO

    _parity_mode = i2c_bits.RWBits(3, REG_LCR, 3)
    _stop_bits = i2c_bit.RWBit(REG_LCR, 2)
    _word_length = i2c_bits.RWBits(2, REG_LCR, 0)

    @property
    def parity(self):
        return self._parity_mode

    @parity.setter
    def parity(self, value):
        if value not in (PARITY_NONE, PARITY_EVEN, PARITY_ODD):
            raise ValueError(
                "Value {value:2x} is not valid for parity parameter".format(value=value)
            )

        self._parity = value

    @property
    def stop_bits(self):
        return int(self._stop_bits) + 1

    @stop_bits.setter
    def stop_bits(self, value):
        if value not in (1, 2):
            raise ValueError(
                "Value {value} is not valid for stop parameter".format(value=value)
            )

        self._stop_bits = bool(value - 1)

    @property
    def word_length(self):
        return int(self._word_length) + 5

    @word_length.setter
    def word_length(self, bits):
        if bits not in (5, 6, 7, 8):
            raise ValueError(
                "Value {bits} is not valid for bits parameter".format(bits=bits)
            )

        self._word_length = bits - 5

    _divisor_latch_enable = i2c_bit.RWBit(REG_LCR, 7)
    _dll = i2c_bits.RWBits(8, REG_DLL, 0)
    _dlh = i2c_bits.RWBits(8, REG_DLH, 0)
    _clock_prescaler = i2c_bit.RWBit(REG_MCR, 7)

    @property
    def _divisor(self):
        self._divisor_latch_enable = True
        divisor = self._dll | (self._dlh << 8)
        self._divisor_latch_enable = False
        return divisor

    @_divisor.setter
    def _divisor(self, value):
        if value > 0xFFFF or value == 0:
            raise ValueError(
                "Unable to configure divisor to {value}.".format(value=value)
            )

        self._divisor_latch_enable = True
        self._dll = int(value) & 0xFF
        self._dlh = int(value) >> 8
        self._divisor_latch_enable = False

    @property
    def baudrate(self):
        divisor = self._divisor
        if self._clock_prescaler:
            divisor *= 4

        return self._crystal_clock / (16 * divisor)

    @baudrate.setter
    def baudrate(self, value):
        divisor = self._crystal_clock / (16 * value)

        # This is not representable even with prescaler enabled.
        if divisor > 0x3FFFC:
            raise ValueError(
                "Unable to configure baud rate to {value} with a crystal of {crystal_clock}".format(
                    value=value, crystal_clock=self._crystal_clock
                )
            )
        # This can only be represented with prescaler enabled.
        elif divisor > 0xFFFF:
            divisor /= 4
            self._clock_prescaler = True

        self._divisor = divisor

    def _read_reg_bytes(self, register, nbytes):
        assert nbytes <= 64

        with self.i2c_device as i2c:
            _BUFFER[0] = register

            i2c.write_then_readinto(
                _BUFFER, _BUFFER, out_end=1, in_start=1, in_end=nbytes + 1
            )
            return _BUFFER[1 : nbytes + 1]

    def _write_reg_bytes(self, register, buf):
        assert len(buf) <= 64

        with self.i2c_device as i2c:
            _BUFFER[0] = register
            _BUFFER[1 : len(buf) + 1] = buf
            i2c.write(_BUFFER, end=len(buf) + 1)
