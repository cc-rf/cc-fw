#!/usr/bin/env python
import os
import sys
import struct

pkt_length = 8
pkt_count = 100
pkt_flags = 0
pkt_channel = 15


def pkt_create(idx):
    return struct.pack("<BBBB%is" % (pkt_length - 1),
                       0x11, pkt_flags, pkt_channel, idx, ':' * (pkt_length - 1))


def main(args):
    tty = args[0]
    baud = 230400
    try:
        if len(args) > 1:
            baud = int(args[1])

        serial = get_serial(tty, baud)
        read = sys.stdin.read
        write = serial.write
        flush = serial.flush

        print >>sys.stderr, "tx: flags=%i channel=%i length=%i count=%i" % (pkt_flags, pkt_channel, pkt_length, pkt_count)

        for idx in range(pkt_count):
            data = pkt_create(idx)
            data_len = data_len_rem = len(data)
            frame_count = 0

            while data_len_rem:
                xfer_length = min(data_len_rem, 255)
                write(chr(xfer_length) + data[:xfer_length])
                flush()
                frame_count += 1
                data = data[xfer_length:]
                data_len_rem -= xfer_length

            # print >>sys.stderr, "write: %i bytes / %i frames" % (data_len, frame_count)
            sys.stderr.write('.')
            sys.stderr.flush()

        sys.stderr.write('\n')

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
        print "usage:", os.path.basename(sys.argv[0]), "<tty>", "[<baud>]"
        raise SystemExit

    main(sys.argv[1:])
