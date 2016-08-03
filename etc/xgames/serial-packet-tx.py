#!/usr/bin/env python
import os
import sys
import struct
import time

pkt_delay = 0
pkt_flags = 2

dfl_chan_idle = 10
dfl_chan_strm = 13

def cmd_create(mac_addr_str, starting):
    mac_addr =  ''.join(chr(int(x,16)) for x in reversed(mac_addr_str.split(':')))

    if starting:
        cmd_chan = dfl_chan_strm | 0x80
    else:
        cmd_chan = dfl_chan_idle

    cmd = '0AT' + mac_addr + chr(cmd_chan)
    return cmd

def pkt_create(mac_addr_str, starting):
    cmd = cmd_create(mac_addr_str, starting)
    pkt_length = len(cmd)

    if starting:
        tx_channel = dfl_chan_idle
        pkt_count = 1
    else:
        tx_channel = dfl_chan_strm
        pkt_count = 20

    return struct.pack("<BBBBB%is" % pkt_length,
             0x11, pkt_flags, tx_channel, pkt_count, pkt_delay,
             cmd)


def main(args):
    tty = args[0]
    mac = args[1]
    cmd = args[2]
    baud = 230400

    if cmd == 'on' or cmd == '1' or cmd == 'start':
        cmd = True
    else:
        cmd = False

    try:
        serial = get_serial(tty, baud)
        read = sys.stdin.read
        write = serial.write
        flush = serial.flush

        print >>sys.stderr, "tx: target=%s enable=%s" % (mac, "1" if cmd else "0")

        #print >>sys.stderr, "tx: flags=%i channel=%i length=%i count=%i" % (pkt_flags, pkt_channel, pkt_length, pkt_count)

        for idx in range(1):
            data = pkt_create(mac, cmd)
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
            #time.sleep(0.010)
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
