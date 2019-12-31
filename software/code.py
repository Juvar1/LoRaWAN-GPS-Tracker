import gc
import microcontroller
from digitalio import DigitalInOut, Direction, Pull
import time
import board
import busio
from analogio import AnalogIn
gc.collect()
from minimal_gps import GPS
gc.collect()
from JuvarLoRa import TTN, JuvarLoRa

# Enable automatic garbage collection
gc.enable()

uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=30)
gps = GPS(uart, debug=False)
del uart
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Enter sleep mode (wake-up on any input)
gps.send_command(b'PMTK161,0')

# gc.collect()
# print('Initial free: {} allocated: {}'.format(gc.mem_free(), gc.mem_alloc()))

# button
button = DigitalInOut(board.D10)
button.direction = Direction.INPUT
button.pull = Pull.UP

# battery voltage measurement
battery = AnalogIn(board.BATTERY)

# LoRa radio pinouts
ena = DigitalInOut(microcontroller.pin.PA06)
irq = DigitalInOut(microcontroller.pin.PA09)
rst = DigitalInOut(microcontroller.pin.PA08)

# TTN Device Address, 4 Bytes, MSB
dev = bytearray([0x26, 0x01, 0x1E, 0xE0])

# TTN Network Key, 16 Bytes, MSB
net = bytearray([0x44, 0x73, 0x05, 0xA1, 0xB7, 0xC7, 0x8D,
    0xDE, 0x0F, 0xA4, 0x95, 0x88, 0x32, 0x7A, 0x0A, 0x1B])

# TTN Application Key, 16 Bytes, MSB
app = bytearray([0xB3, 0x91, 0x81, 0x85, 0xAD, 0xD2, 0x61,
    0x07, 0xF2, 0x0C, 0x20, 0x47, 0xDE, 0xD0, 0x61, 0x89])

ttn = TTN(dev, net, app, country='EU')
lora = JuvarLoRa(spi, ena, irq, rst, ttn)
del dev
del net
del app
del spi
del ena
del irq
del rst
del ttn

# python handles negative numbers automatically with two's complement
def getCoords():
    lat = 0
    lng = 0
    # Turn on the basic GGA and RMC info (typical settings)
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    # Set update rate to once a second (1Hz)
    gps.send_command(b'PMTK220,1000')
    start = time.monotonic()
    while True:
        gps.update()
        if gps.has_fix:
            print('Latitude: {0:.6f} degrees'.format(gps.latitude))
            print('Longitude: {0:.6f} degrees'.format(gps.longitude))
            lat = int(gps.latitude * 1e6)
            lng = int(gps.longitude * 1e6)
            break
        if (time.monotonic() - start) >= 90:  # 90s is absolute min
            print('No GPS satellites available.')
            break
    # Set sleep mode
    gps.send_command(b'PMTK161,0')
    buffer = bytearray([
        (lat >> 24) & 0xff,
        (lat >> 16) & 0xff,
        (lat >> 8)  & 0xff,
        lat & 0xff,
        (lng >> 24) & 0xff,
        (lng >> 16) & 0xff,
        (lng >> 8)  & 0xff,
        lng  & 0xff,
    ])
    return buffer

# default polling delay in minutes
delay = 30
# by default tracking is off
tracking = False

while True:
    gc.collect()
    polling_time = False
    start = time.monotonic()
    while button.value and not polling_time:
        if (time.monotonic() - start) >= 60 * delay:
            # every half hour poll messages from server
            # send position and battery voltage if message is received
            # tracking can be started/stopped with message
            # in tracking mode device sends position in 5min interval
            polling_time = True
    # add voltage to data packet
    data = bytearray([int((battery.value * 33) / 65536) & 0xff])
    # add coordinates to data packet
    if not polling_time or tracking:
        print('Waiting for GPS fix...')
        data += getCoords()
    # Send data packet
    print('Processing data...')
    packet = lora.process_data(data, len(data), lora.frame_counter_up,
        lora.frame_counter_down, timeout=2)
    print('Processing done!')
    print('Up counter: {0}'.format(lora.frame_counter_up))
    lora.frame_counter_up += 1
    if packet:
        print('Received (raw bytes): {0}'.format(packet))
        print('Down counter: {0}'.format(lora.frame_counter_down))
        lora.frame_counter_down += 1
        tracking = False
        delay = 30
    if packet == b'\x05':  # start tracking with 5min interval
        print('Tracking is set!')
        tracking = True
        delay = 5
    del data
    del packet
