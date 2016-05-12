#!/usr/bin/env python
import os
import sys

data_prev = ''
dump_byte_width = 12

def main(args):
    tty = args[0]
    baud = 115200
    try:
        if len(args) > 1:
            baud = int(args[1])

        serial = get_serial(tty, baud)
        read = serial.read
        write = sys.stdout.write
        flush = sys.stdout.flush

        while True:
            dump(read(8))
            flush()
    except Exception, e:
        print e
    except KeyboardInterrupt:
        print


def dump(data):
    global data_prev

    data = data_prev + data

    #if len(data) < 8:
    #    data_prev = data
    #    return

    while len(data) >= dump_byte_width:
        line = data[:dump_byte_width]
        data = data[dump_byte_width:]
        print " ".join("{:02x}".format(ord(c)) for c in line), "\t",
        print "".join(c if ord(c) in range(33, 128) else "." for c in line)

    data_prev = data


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
