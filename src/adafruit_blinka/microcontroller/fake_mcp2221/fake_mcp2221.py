# SPDX-FileCopyrightText: 2021 Melissa LeBlanc-Williams for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""Chip Definition for MCP2221"""

import os
import time
import atexit

import hid

# TODO: Remove all the unnecessary calls to the hid device
# as we're faking the MCP2221A

# Here if you need it
MCP2221_HID_DELAY = float(os.environ.get("BLINKA_MCP2221_HID_DELAY", 0))
# Use to set delay between reset and device reopen. if negative, don't reset at all
MCP2221_RESET_DELAY = float(os.environ.get("BLINKA_MCP2221_RESET_DELAY", 0.5))

# from the C driver
# http://ww1.microchip.com/downloads/en/DeviceDoc/mcp2221_0_1.tar.gz
# others (???) determined during driver developement
RESP_ERR_NOERR = 0x00
RESP_ADDR_NACK = 0x25
RESP_READ_ERR = 0x7F
RESP_READ_COMPL = 0x55
RESP_READ_PARTIAL = 0x54  # ???
RESP_I2C_IDLE = 0x00
RESP_I2C_START_TOUT = 0x12
RESP_I2C_RSTART_TOUT = 0x17
RESP_I2C_WRADDRL_TOUT = 0x23
RESP_I2C_WRADDRL_WSEND = 0x21
RESP_I2C_WRADDRL_NACK = 0x25
RESP_I2C_WRDATA_TOUT = 0x44
RESP_I2C_RDDATA_TOUT = 0x52
RESP_I2C_STOP_TOUT = 0x62

RESP_I2C_MOREDATA = 0x43  # ???
RESP_I2C_PARTIALDATA = 0x41  # ???
RESP_I2C_WRITINGNOSTOP = 0x45  # ???

MCP2221_RETRY_MAX = 50
MCP2221_MAX_I2C_DATA_LEN = 60
MASK_ADDR_NACK = 0x40


class MCP2221:
    """MCP2221 Device Class Definition"""

    VID = 0x04D8
    PID = 0x00DD

    GP_GPIO = 0b000
    GP_DEDICATED = 0b001
    GP_ALT0 = 0b010
    GP_ALT1 = 0b011
    GP_ALT2 = 0b100

    def __init__(self):
        pass # This is a "fake" implementation
        """
        self._hid = hid.device()
        self._hid.open(MCP2221.VID, MCP2221.PID)
        # make sure the device gets closed before exit
        atexit.register(self.close)
        if MCP2221_RESET_DELAY >= 0:
            self._reset()
        self._gp_config = [0x07] * 4  # "don't care" initial value
        for pin in range(4):
            self.gp_set_mode(pin, self.GP_GPIO)  # set to GPIO mode
            self.gpio_set_direction(pin, 1)  # set to INPUT
        """

    def close(self):
        """Close the hid device. Does nothing if the device is not open."""
        # self._hid.close()
        pass

    def __del__(self):
        # try to close the device before destroying the instance
        # self.close()
        pass


    def _hid_xfer(self, report, response=True):
        """Perform HID Transfer"""
        # first byte is report ID, which =0 for MCP2221
        # remaing bytes = 64 byte report data
        # https://github.com/libusb/hidapi/blob/083223e77952e1ef57e6b77796536a3359c1b2a3/hidapi/hidapi.h#L185
        """
        self._hid.write(b"\0" + report + b"\0" * (64 - len(report)))
        time.sleep(MCP2221_HID_DELAY)
        if response:
            # return is 64 byte response report
            return self._hid.read(64)
        """
        return None

    # ----------------------------------------------------------------
    # MISC
    # ----------------------------------------------------------------
    def gp_get_mode(self, pin):
        """Get Current Pin Mode"""
        # return self._hid_xfer(b"\x61")[22 + pin] & 0x07
        pass

    def gp_set_mode(self, pin, mode):
        """Set Current Pin Mode"""
        pass
        """
        # already set to that mode?
        mode &= 0x07
        if mode == (self._gp_config[pin] & 0x07):
            return
        # update GP mode for pin
        self._gp_config[pin] = mode
        # empty report, this is safe since 0's = no change
        report = bytearray(b"\x60" + b"\x00" * 63)
        # set the alter GP flag byte
        report[7] = 0xFF
        # add GP setttings
        report[8] = self._gp_config[0]
        report[9] = self._gp_config[1]
        report[10] = self._gp_config[2]
        report[11] = self._gp_config[3]
        # and make it so
        self._hid_xfer(report)
        """

    def _pretty_report(self, register):
        pass
        """
        report = self._hid_xfer(register)
        print("     0  1  2  3  4  5  6  7  8  9")
        index = 0
        for row in range(7):
            print("{} : ".format(row), end="")
            for _ in range(10):
                print("{:02x} ".format(report[index]), end="")
                index += 1
                if index > 63:
                    break
            print()
        """

    def _status_dump(self):
        pass
        # self._pretty_report(b"\x10")

    def _sram_dump(self):
        pass
        # self._pretty_report(b"\x61")

    def _reset(self):
        pass
        """
        self._hid_xfer(b"\x70\xAB\xCD\xEF", response=False)
        self._hid.close()
        time.sleep(MCP2221_RESET_DELAY)
        start = time.monotonic()
        while time.monotonic() - start < 5:
            try:
                self._hid.open(MCP2221.VID, MCP2221.PID)
            except OSError:
                # try again
                time.sleep(0.1)
                continue
            return
        raise OSError("open failed")
        """

    # ----------------------------------------------------------------
    # GPIO
    # ----------------------------------------------------------------
    def gpio_set_direction(self, pin, mode):
        """Set Current GPIO Pin Direction"""
        pass
        """
        if mode:
            # set bit 3 for INPUT
            self._gp_config[pin] |= 1 << 3
        else:
            # clear bit 3 for OUTPUT
            self._gp_config[pin] &= ~(1 << 3)
        report = bytearray(b"\x50" + b"\x00" * 63)  # empty set GPIO report
        offset = 4 * (pin + 1)
        report[offset] = 0x01  # set pin direction
        report[offset + 1] = mode  # to this
        self._hid_xfer(report)
        """

    def gpio_set_pin(self, pin, value):
        """Set Current GPIO Pin Value"""
        pass
        """
        if value:
            # set bit 4
            self._gp_config[pin] |= 1 << 4
        else:
            # clear bit 4
            self._gp_config[pin] &= ~(1 << 4)
        report = bytearray(b"\x50" + b"\x00" * 63)  # empty set GPIO report
        offset = 2 + 4 * pin
        report[offset] = 0x01  # set pin value
        report[offset + 1] = value  # to this
        self._hid_xfer(report)
        """

    def gpio_get_pin(self, pin):
        """Get Current GPIO Pin Value"""
        pass
        """
        resp = self._hid_xfer(b"\x51")
        offset = 2 + 2 * pin
        if resp[offset] == 0xEE:
            raise RuntimeError("Pin is not set for GPIO operation.")
        return resp[offset]
        """

    # ----------------------------------------------------------------
    # I2C
    # ----------------------------------------------------------------
    def _i2c_status(self):
        pass
        """
        resp = self._hid_xfer(b"\x10")
        if resp[1] != 0:
            raise RuntimeError("Couldn't get I2C status")
        return resp
        """

    def _i2c_state(self):
        pass
        # return self._i2c_status()[8]

    def _i2c_cancel(self):
        pass
        """
        resp = self._hid_xfer(b"\x10\x00\x10")
        if resp[1] != 0x00:
            raise RuntimeError("Couldn't cancel I2C")
        if resp[2] == 0x10:
            # bus release will need "a few hundred microseconds"
            time.sleep(0.001)
        """

    # pylint: disable=too-many-arguments,too-many-branches
    def _i2c_write(self, cmd, address, buffer, start=0, end=None):
        pass


    def _i2c_read(self, cmd, address, buffer, start=0, end=None):
        pass

    # pylint: enable=too-many-arguments

    def _i2c_configure(self, baudrate=100000):
        """Configure I2C"""
        pass

    def i2c_writeto(self, address, buffer, *, start=0, end=None):
        """Write data from the buffer to an address"""
        pass
        # self._i2c_write(0x90, address, buffer, start, end)

    def i2c_readfrom_into(self, address, buffer, *, start=0, end=None):
        """Read data from an address and into the buffer"""
        pass
        # self._i2c_read(0x91, address, buffer, start, end)

    def i2c_writeto_then_readfrom(
        self,
        address,
        out_buffer,
        in_buffer,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None,
    ):
        """Write data from buffer_out to an address and then
        read data from an address and into buffer_in
        """
        pass
        #self._i2c_write(0x94, address, out_buffer, out_start, out_end)
        #self._i2c_read(0x93, address, in_buffer, in_start, in_end)

    def i2c_scan(self, *, start=0, end=0x79):
        """Perform an I2C Device Scan"""
        pass
        """
        found = []
        for addr in range(start, end + 1):
            # try a write
            try:
                self.i2c_writeto(addr, b"\x00")
            except RuntimeError:  # no reply!
                continue
            # store if success
            found.append(addr)
        return found
        """

    # ----------------------------------------------------------------
    # ADC
    # ----------------------------------------------------------------
    def adc_configure(self, vref=0):
        """Configure the Analog-to-Digital Converter"""
        pass
        """
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[5] = 1 << 7 | (vref & 0b111)
        self._hid_xfer(report)
        """

    def adc_read(self, pin):
        """Read from the Analog-to-Digital Converter"""
        pass
        """
        resp = self._hid_xfer(b"\x10")
        return resp[49 + 2 * pin] << 8 | resp[48 + 2 * pin]
        """

    # ----------------------------------------------------------------
    # DAC
    # ----------------------------------------------------------------
    def dac_configure(self, vref=0):
        """Configure the Digital-to-Analog Converter"""
        pass
        """
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[3] = 1 << 7 | (vref & 0b111)
        self._hid_xfer(report)
        """

    # pylint: disable=unused-argument
    def dac_write(self, pin, value):
        """Write to the Digital-to-Analog Converter"""
        pass
        """
        report = bytearray(b"\x60" + b"\x00" * 63)
        report[4] = 1 << 7 | (value & 0b11111)
        self._hid_xfer(report)
        """

    # pylint: enable=unused-argument


mcp2221 = MCP2221()
