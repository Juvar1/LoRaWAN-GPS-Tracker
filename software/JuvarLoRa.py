# JuvarLoRa
#
# Copyright 2019, 2020 Juha-Pekka Varjonen
#
# Base code is derived from Adafruit_TinyLoRa.
# This program works only with EU frequencies.
#
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#
# Adafruit_TinyLoRa
# https://github.com/adafruit/Adafruit_CircuitPython_TinyLoRa
#
# Copyright 2015, 2016 Ideetron B.V.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#
# Modified by Brent Rubell for Adafruit Industries.

import time
from random import randint
from micropython import const
import adafruit_bus_device.spi_device
from JuvarLoRa_encryption import AES

# RFM Module Settings
_MODE_SLEEP = const(0x00)
_MODE_LORA = const(0x80)
_MODE_STDBY = const(0x01)
_MODE_TX = const(0x83)
_MODE_RX = const(0x85)
_MODE_RX_SINGLE = const(0x86)
_TRANSMIT_DIRECTION_UP = const(0x00)
# RFM Registers
_REG_PA_CONFIG = const(0x09)
_REG_PREAMBLE_MSB = const(0x20)
_REG_PREAMBLE_LSB = const(0x21)
_REG_FRF_MSB = const(0x06)
_REG_FRF_MID = const(0x07)
_REG_FRF_LSB = const(0x08)
_REG_FEI_LSB = const(0x1E)
_REG_FEI_MSB = const(0x1D)
_REG_MODEM_CONFIG = const(0x26)
_REG_PAYLOAD_LENGTH = const(0x22)
_REG_FIFO_POINTER = const(0x0D)
_REG_FIFO_BASE_ADDR = const(0x80)
_REG_OPERATING_MODE = const(0x01)
_REG_VERSION = const(0x42)
_REG_PREAMBLE_DETECT = const(0x1F)
_REG_TIMER1_COEF = const(0x39)
_REG_NODE_ADDR = const(0x33)
_REG_IMAGE_CAL = const(0x3B)
_REG_RSSI_CONFIG = const(0x0E)
_REG_RSSI_COLLISION = const(0x0F)
_REG_DIO_MAPPING_1 = const(0x40)

# Freq synth step
# _FSTEP = 32000000.0 / 524288


class TTN:
    """TTN Class
    """

    def __init__(self, dev_address, net_key, app_key, country="EU"):
        """Interface for TheThingsNetwork
        :param bytearray dev_address: TTN Device Address.
        :param bytearray net_key: TTN Network Key.
        :param bytearray app_key: TTN Application Key.
        :param string country: TTN Region.
        """
        self.dev_addr = dev_address
        self.net_key = net_key
        self.app_key = app_key
        self.region = country

    @property
    def country(self):
        """Returns the TTN Frequency Country.
        """
        return self.region

    @property
    def device_address(self):
        """Returns the TTN Device Address.
        """
        return self.dev_addr

    @property
    def application_key(self):
        """Returns the TTN Application Key.
        """
        return self.app_key

    @property
    def network_key(self):
        """Returns the TTN Network Key.
        """
        return self.net_key


# pylint: disable=too-many-instance-attributes
class JuvarLoRa:

    # SPI Write Buffer
    _BUFFER = bytearray(2)

    # pylint: disable=too-many-arguments
    def __init__(self, spi, cs, irq, rst, ttn_config, channel=None):
        """Interface for a HopeRF RFM95/6/7/8(w) radio module. 
        Sets module up for sending to The Things Network.
        :param ~busio.SPI spi: The SPI bus the device is on
        :param ~digitalio.DigitalInOut cs: Chip select pin (RFM_NSS)
        :param ~digitalio.DigitalInOut irq: RFM's DIO0 Pin (RFM_DIO0)
        :param ~digitalio.DigitalInOut rst: RFM's RST Pin (RFM_RST)
        :param TTN ttn_config: TTN Configuration.
        :param int channel: Frequency Channel.
        """
        self._irq = irq
        self._irq.switch_to_input()
        self._cs = cs
        self._cs.switch_to_output()
        self._rst = rst
        self._rst.switch_to_output()
        # Set up SPI Device on Mode 0
        self._device = adafruit_bus_device.spi_device.SPIDevice(
            spi, self._cs, baudrate=4000000, polarity=0, phase=0
        )
        self._rst.value = False
        time.sleep(0.0001)  # 100 us
        self._rst.value = True
        time.sleep(0.005)  # 5 ms
        # Verify the version of the RFM module
        self._version = self._read_u8(_REG_VERSION)
        if self._version != 18:
            raise TypeError("Can not detect LoRa Module. Please check wiring!")
        # Set Frequency registers
        self._rfm_msb = None
        self._rfm_mid = None
        self._rfm_lsb = None
        # Set datarate registers
        self._sf = None
        self._bw = None
        self._modemcfg = None
        self.set_datarate("SF7BW125")
        # Set regional frequency plan
        self._frequencies = {0: (0xd9, 0x06, 0x8b),  # 868.1 MHz
             1: (0xd9, 0x13, 0x58),  # 868.3 MHz
             2: (0xd9, 0x20, 0x24),  # 868.5 MHz
             3: (0xd8, 0xc6, 0x8b),  # 867.1 MHz
             4: (0xd8, 0xd3, 0x58),  # 867.3 MHz
             5: (0xd8, 0xe0, 0x24),  # 867.5 MHz
             6: (0xd8, 0xec, 0xf1),  # 867.7 MHz
             7: (0xd8, 0xf9, 0xbe)}  # 867.9 MHz
        # Set Channel Number
        self._channel = channel
        self._tx_random = randint(0, 7)
        if self._channel is not None:
            # set single channel
            self.set_channel(self._channel)
        # Init FrameCounters
        self.frame_counter_up = 0
        self.frame_counter_down = 0
        # Set up RFM9x for LoRa Mode
        for pair in (
                (_REG_OPERATING_MODE, _MODE_SLEEP),
                (_REG_OPERATING_MODE, _MODE_LORA),
                (_REG_PA_CONFIG, 0xFF),
                (_REG_PREAMBLE_DETECT, 0x25),
                (_REG_PREAMBLE_MSB, 0x00),
                (_REG_PREAMBLE_LSB, 0x08),
                (_REG_MODEM_CONFIG, 0x0C),
                (_REG_TIMER1_COEF, 0x34),
                (_REG_NODE_ADDR, 0x67),  # ? was 0x27, now inverted IQ
                (_REG_IMAGE_CAL, 0x19),  # ? was 0x1D, now inverted IQ
                (_REG_RSSI_CONFIG, 0x80),  # ? tx base address
                (_REG_RSSI_COLLISION, 0x00),  # ? rx base address
        ):
            self._write_u8(pair[0], pair[1])
        # Give the lora object ttn configuration
        self._ttn_config = ttn_config

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.deinit()

    def deinit(self):
        """Deinitializes the TinyLoRa object properties and pins."""
        self._irq = None
        self._rst = None
        self._cs = None
        self.frame_counter_up = 0
        self.frame_counter_down = 0
        self._rfm_msb = None
        self._rfm_mid = None
        self._rfm_lsb = None
        self._sf = None
        self._bw = None
        self._modemcfg = None

    def process_data(self, data, data_length, 
        frame_counter_up, frame_counter_down, timeout=2):
        """Function to assemble and send data and open two receiving window
           and process received data
           :param data: data to send
           :param data_length: length of data to send
           :param frame_counter_up: frame counter up variable
           :param frame_counter_down: frame counter down variable
           :param timeout: TxDone wait time, default is 2.
        """
        # data packet
        enc_data = bytearray(data_length)
        lora_pkt = bytearray(64)
        # copy bytearray into bytearray for encryption
        enc_data[0:data_length] = data[0:data_length]
        del data
        # encrypt data (enc_data is overwritten in this function)
        self.frame_counter_up = frame_counter_up
        aes = AES(
            self._ttn_config.device_address,
            self._ttn_config.app_key,
            self._ttn_config.network_key,
            self.frame_counter_up,
        )
        enc_data = aes.encrypt(enc_data)
        # append preamble to packet
        lora_pkt[0] = const(_REG_DIO_MAPPING_1)
        lora_pkt[1] = self._ttn_config.device_address[3]
        lora_pkt[2] = self._ttn_config.device_address[2]
        lora_pkt[3] = self._ttn_config.device_address[1]
        lora_pkt[4] = self._ttn_config.device_address[0]
        lora_pkt[5] = 0
        lora_pkt[6] = frame_counter_up & 0x00FF
        lora_pkt[7] = (frame_counter_up >> 8) & 0x00FF
        lora_pkt[8] = 0x01
        del frame_counter_up
        # set length of LoRa packet
        lora_pkt_len = 9
        # load encrypted data into lora_pkt
        lora_pkt[lora_pkt_len : lora_pkt_len + data_length] = enc_data[0:data_length]
        del enc_data
        # recalculate packet length
        lora_pkt_len += data_length
        del data_length
        # Calculate MIC
        mic = bytearray(4)
        mic = aes.calculate_mic(lora_pkt, lora_pkt_len, 0x00, mic)
        del aes
        # load mic in package
        lora_pkt[lora_pkt_len : lora_pkt_len + 4] = mic[0:4]
        del mic
        # recalculate packet length (add MIC length)
        lora_pkt_len += 4
        # ---------------------
        # Set RFM to standby
        self._write_u8(_MODE_STDBY, 0x81)
        # wait for RFM to enter standby mode
        time.sleep(0.01)
        # switch interrupt to txdone
        self._write_u8(0x40, 0x40)
        # set datarate configuration
        self.set_datarate("SF7BW125")
        # check for multi-channel configuration
        if self._channel is None:
            self._tx_random = randint(0, 7)
            self._rfm_lsb = self._frequencies[self._tx_random][2]
            self._rfm_mid = self._frequencies[self._tx_random][1]
            self._rfm_msb = self._frequencies[self._tx_random][0]
        # Set up frequency registers
        for pair in (
                (_REG_FRF_MSB, self._rfm_msb),
                (_REG_FRF_MID, self._rfm_mid),
                (_REG_FRF_LSB, self._rfm_lsb),
                (_REG_FEI_LSB, self._sf),
                (_REG_FEI_MSB, self._bw),
                (_REG_MODEM_CONFIG, self._modemcfg),
                (_REG_PAYLOAD_LENGTH, lora_pkt_len),
                (_REG_FIFO_POINTER, _REG_FIFO_BASE_ADDR),
        ):
            self._write_u8(pair[0], pair[1])
        # fill the FIFO buffer with the LoRa payload
        for k in range(lora_pkt_len):
            self._write_u8(0x00, lora_pkt[k])
        del lora_pkt
        del lora_pkt_len
        # switch RFM to TX operating mode
        self._write_u8(_REG_OPERATING_MODE, _MODE_TX)
        # wait for TxDone IRQ, poll for timeout.
        start = time.monotonic()
        while not self._irq.value:
            if (time.monotonic() - start) >= timeout:
                self._write_u8(_REG_OPERATING_MODE, _MODE_SLEEP)
                # raise RuntimeError("Timeout during packet send")
                del start
                return None
        del start
        # ---------------------
        self._write_u8(0x12, 0xFF)  # Clear all interrupts
        self._write_u8(0x40, 0x00)  # switch interrupt to rxdone
        self._write_u8(_REG_OPERATING_MODE, _MODE_RX)  # start listen
        # Wait for the rx done interrupt with RX1 frequency
        start = time.monotonic()
        print('receiving...')
        while not self._irq.value:
            if (time.monotonic() - start) >= 3:
                # Wait for the rx done interrupt with RX2 frequency
                # set datarate for RX2
                self.set_datarate("SF9BW125")
                # set RX2 frequency to 869.525MHz
                # RX2Freq = int((869.525*1000000)/61.035)
                # Set up frequency registers
                for pair in (
                        (_REG_FRF_MSB, 0xd9),
                        (_REG_FRF_MID, 0x61),
                        (_REG_FRF_LSB, 0xbe),
                        (_REG_FEI_LSB, self._sf),
                        (_REG_FEI_MSB, self._bw),
                        (_REG_MODEM_CONFIG, self._modemcfg),
                ):
                    self._write_u8(pair[0], pair[1])
                while not self._irq.value:
                    if (time.monotonic() - start) >= 6:
                        # switch RFM to sleep mode
                        self._write_u8(_REG_OPERATING_MODE, _MODE_SLEEP)
                        del start
                        return None
        del start
        # Payload ready is set, a packet is in the FIFO
        packet = None
        print('packet is in the FIFO')
        # Grab the length of the received packet
        length = self._read_u8(0x13)
        # Reset the FIFO read ptr to the beginning of the packet
        current_addr = self._read_u8(0x10)
        self._write_u8(_REG_FIFO_POINTER, current_addr)
        del current_addr
        packet = bytearray(length)
        del length
        # Read the packet from FIFO buffer
        self._read_into(0x00, packet)
        # Clear all interrupts
        self._write_u8(0x12, 0xFF)
        # switch RFM to sleep mode
        self._write_u8(_REG_OPERATING_MODE, _MODE_SLEEP)
        # start parsing payload
        # https://github.com/anthonykirby/lora-packet/blob/master/lib/packet.js
        MACPayload = packet[1:-4]
        micBytes = packet[:-4]
        MIC = packet[-4:]
        del packet
        FOptsLen = MACPayload[4] & 0x0F
        FHDR_length = 7 + FOptsLen
        del FOptsLen
        FHDR = MACPayload[0:FHDR_length]
        DevAddr = bytearray(reversed(FHDR[0:4]))   # reverse order!
        FCnt = (FHDR[6] << 8) | FHDR[5]   # reverse order!
        del FHDR
        FRMPayload = MACPayload[FHDR_length+1:]
        # check if there is at least 1 byte of user data
        if FHDR_length == len(MACPayload):
            print('no user data found')
            del FHDR_length
            return None
        del FHDR_length
        # check frame counter
        if FCnt != frame_counter_down:
            print('frame counter does not match')
            del FCnt
            return None
        del FCnt
        # check device address
        if DevAddr != self._ttn_config.device_address:
            print('device address does not match')
            del DevAddr
            return None
        del DevAddr
        # initialize AES module
        aes = AES(
            self._ttn_config.device_address,
            self._ttn_config.app_key,
            self._ttn_config.network_key,
            frame_counter_down,
        )
        # Check Message Integrity Code, MIC
        micBuffer = bytearray(4)
        micBuffer = aes.calculate_mic(micBytes, len(micBytes), 0x01, micBuffer)
        if MIC != micBuffer:
            print('mic does not match. TTN:{},Local:{}'.format(MIC,micBuffer))
            del MIC
            del micBuffer
            return None
        del MIC
        del micBuffer
        # save frame counter value
        self.frame_counter_down = frame_counter_down
        del frame_counter_down
        # decrypt payload
        # https://github.com/jieter/python-lora/blob/master/lora/crypto.py
        aBlock = bytearray([
            0x01,  # 0 always 0x01
            0x00,  # 1 always 0x00
            0x00,  # 2 always 0x00
            0x00,  # 3 always 0x00
            0x00,  # 4 always 0x00
            1,     # 5 dir, 0 for uplink, 1 for downlink
            self._ttn_config.device_address[3],  # 6 devaddr, lsb
            self._ttn_config.device_address[2],  # 7 devaddr
            self._ttn_config.device_address[1],  # 8 devaddr
            self._ttn_config.device_address[0],  # 9 devaddr, msb
            self.frame_counter_down   & 0xFF,    # 10 seq ctr, lsb
            (self.frame_counter_down >> 8)  & 0xFF,  # 11 seq ctr
            (self.frame_counter_down >> 16) & 0xFF,  # 12 seq ctr
            (self.frame_counter_down >> 24) & 0xFF,  # 13 seq ctr, msb
            0x00,  # 14 always 0x00
            0x00,  # 15 block counter
        ])
        ctr = 1  # block counter
        size = len(FRMPayload)
        bufferIndex = 0
        # initialize output buffer
        encBuffer = bytearray(size)
        # complete blocks
        while size >= 16:
            aBlock[15] = ctr & 0xFF
            ctr += 1
            aes._aes_encrypt(aBlock, self._ttn_config.app_key)
            for i in range(16):
                encBuffer[bufferIndex + i] = FRMPayload[bufferIndex + i] ^ aBlock[i]
            size -= 16
            bufferIndex += 16
        # partial blocks
        if size > 0:
            aBlock[15] = ctr & 0xFF
            aes._aes_encrypt(aBlock, self._ttn_config.app_key)
            for i in range(size):
                encBuffer[bufferIndex + i] = FRMPayload[bufferIndex + i] ^ aBlock[i]
        del FRMPayload
        del aes
        del aBlock
        del ctr
        del size
        del bufferIndex
        return encBuffer

    def set_datarate(self, datarate):
        """Sets the RFM Datarate
        :param datarate: Bandwidth and Frequency Plan
        """
        data_rates = {
            "SF7BW125": (0x74, 0x72, 0x04),
            "SF7BW250": (0x74, 0x82, 0x04),
            "SF8BW125": (0x84, 0x72, 0x04),
            "SF9BW125": (0x94, 0x72, 0x04),
            "SF10BW125": (0xA4, 0x72, 0x04),
            "SF11BW125": (0xB4, 0x72, 0x0C),
            "SF12BW125": (0xC4, 0x72, 0x0C),
        }
        try:
            self._sf, self._bw, self._modemcfg = data_rates[datarate]
        except KeyError:
            raise KeyError("Invalid or Unsupported Datarate.")

    def set_channel(self, channel):
        """Sets the RFM Channel (if single-channel)
        :param int channel: Transmit Channel (0 through 7).
        """
        self._rfm_msb, self._rfm_mid, self._rfm_lsb = self._frequencies[channel]

    def _read_into(self, address, buf, length=None):
        """Read a number of bytes from the specified address into the
        provided buffer. If length is not specified (default) the entire buffer
        will be filled.
        :param bytearray address: Register Address.
        :param bytearray buf: Data Buffer for bytes.
        :param int length: Buffer length.
        """
        if length is None:
            length = len(buf)
        with self._device as device:
            # Strip out top bit to set 0 value (read).
            self._BUFFER[0] = address & 0x7F
            # pylint: disable=no-member
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def _read_u8(self, address):
        """Read a single byte from the provided address and return it.
        :param bytearray address: Register Address.
        """
        self._read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def _write_u8(self, address, val):
        """Writes to the RFM register given an address and data.
        :param bytearray address: Register Address.
        :param val: Data to write.
        """
        with self._device as device:
            self._BUFFER[0] = address | 0x80  # MSB 1 to Write
            self._BUFFER[1] = val
            # pylint: disable=no-member
            device.write(self._BUFFER, end=2)

