#!/usr/bin/env python
#
# Bitbang'd SPI interface with an MCP3008 ADC device
# MCP3008 is 8-channel 10-bit analog to digital converter
#  Connections are:
#     CLK => SCLK
#     DOUT =>  MISO
#     DIN => MOSI
#     CS => CE0

import time
import sys
import spidev

class SPIAnalog():

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)

    def buildReadCommand(self, channel):
        startBit = 0x01
        singleEnded = 0x08

        # Return python list of 3 bytes
        #   Build a python list using [1, 2, 3]
        #   First byte is the start bit
        #   Second byte contains single ended along with channel #
        #   3rd byte is 0
        return [startBit, singleEnded|(channel<<4),0]

    def processAdcValue(self, result):
        '''Take in result as array of three bytes.
        Return the two lowest bits of the 2nd byte and
         all of the third byte'''
        byte2 = (result[1] & 0x03)
        return (byte2 << 8) | result[2]

    def readAdc(self, channel):
        if ((channel > 7) or (channel < 0)):
            return -1
        r = self.spi.xfer2(self.buildReadCommand(channel))
        return self.processAdcValue(r)

    def closeChannel(self):
        self.spi.close()

if __name__ == '__main__':
    analogsensors = SPIAnalog()
    try:
        while True:
            val = analogsensors.readAdc(0)
            print "ADC Result: ", str(val)
            time.sleep(1)
    except KeyboardInterrupt:
        analogsensors.close()
        sys.exit(0)