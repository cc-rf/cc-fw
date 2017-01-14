"""Serial Framed RF Device Communication Protocol
"""
#!/usr/bin/env python

import os
import sys
import struct
import time
import cobs

SERF_CODE_PROTO_M    = 0b11100000
SERF_CODE_PROTO_VAL  = 0b10100000
SERF_CODE_M          = 0b00011111


def serf_encode(code, data):
    if (code & SERF_CODE_M) != code:
        print >>sys.stderr, "warning: code 0x%02X truncated to 0x%02X" % (code, code & SERF_CODE_M)

    frame = struct.pack("<B%ic" % len(data), SERF_CODE_PROTO_VAL | (code & SERF_CODE_M), *data)

    #sys.stderr.write('frame/raw:')
    #for byte in frame:
    #    sys.stderr.write(' %02X' % ord(byte))
    #sys.stderr.write('\n')

    frame = cobs.cobs_encode(frame) + '\0'

    #sys.stderr.write('frame/enc:')
    #for byte in frame:
    #    sys.stderr.write(' %02X' % ord(byte))
    #sys.stderr.write('\n')

    return frame


SERF_DECODE_ERROR = (0, None)

def serf_decode(data):
    if len(data) <= 1:
        return SERF_DECODE_ERROR

    data = cobs.cobs_decode(data)

    if not len(data):
        return SERF_DECODE_ERROR

    if (ord(data[0]) & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL:
        return SERF_DECODE_ERROR

    return data[0] & SERF_CODE_M, data[1:]


def next_frame():
    count = 0

    while 1:
        count = count + 1
        data = "Hello %i" % count
        print data
        yield make_frame(count, data)
        time.sleep(.1)


def main(args):
    tty = args[0]
    baud = 115200


    try:
        serial = get_serial(tty, baud)
        read = sys.stdin.read
        write = serial.write
        flush = serial.flush

        for frame in next_frame():
            write(frame)
            flush()

    except Exception, e:
        print e
    except KeyboardInterrupt:
        print


def get_serial(tty, baud):
    import serial
    ser = serial.Serial()
    ser.port = tty
    ser.baudrate = baud
    ser.open()
    return ser

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage:", os.path.basename(sys.argv[0]), "<tty>"
        raise SystemExit

    main(sys.argv[1:])
