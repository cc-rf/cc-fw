#!/usr/bin/env python2
"""Serial Framed RF Device Communication Protocol
"""

import os
import sys
import struct
import time
import cobs
import threading

CODE_ID_ECHO = 0
CODE_ID_STATUS = 1
CODE_ID_SEND = 2
CODE_ID_RECV = 3
CODE_ID_RESET = 9

RESET_MAGIC = 0xD1E00D1E


def handle_code_echo(data):
    sys.stdout.write(data)


def handle_code_status(*args):
    handle_status(*args)


def handle_code_recv(node, peer, dest, size, data):
    if len(data) != size:
        print >>sys.stderr, "recv: size/{} != len(data)/{}".format(size, len(data))
        return

    handle_recv(node, peer, dest, data)


def decode_code_echo(data):
    return struct.unpack("%is" % len(data), data)


def decode_code_status(data):
    return struct.unpack("<LQLHLL%is" % (len(data) - 26), data)[:-1]


def decode_code_recv(data):
    return struct.unpack("<HHHB%is" % (len(data) - 7), data)


def encode_code_echo(data):
    return serf_encode(CODE_ID_ECHO, struct.pack("%is" % len(data), data))


NMAC_SEND_DGRM = 0
NMAC_SEND_MESG = 1
NMAC_SEND_TRXN = 2
NMAC_SEND_STRM = 3


def encode_code_reset():
    return serf_encode(CODE_ID_RESET, struct.pack("<L", RESET_MAGIC))


def encode_code_status():
    return serf_encode(CODE_ID_STATUS, '')


def encode_code_send(typ, dest, data):
    return serf_encode(CODE_ID_SEND, struct.pack("<BH%is" % len(data), typ & 0xFF, dest & 0xFFFF, data))


encode_map = {
    CODE_ID_ECHO: encode_code_echo,
    CODE_ID_SEND: encode_code_send,
    CODE_ID_STATUS: encode_code_status,
    CODE_ID_RESET: encode_code_reset
}


def encode_code(code, *args):
    return encode_map.get(code, encode_map[CODE_ID_ECHO])(*args)


decode_map = {
    CODE_ID_ECHO: (decode_code_echo, handle_code_echo),
    CODE_ID_RECV: (decode_code_recv, handle_code_recv),
    CODE_ID_STATUS: (decode_code_status, handle_code_status),
}


def decode_code(code, data):
    decoder, handler = decode_map.get(code, decode_map[CODE_ID_ECHO])
    params = decoder(data)
    # print("decode: {} handle: {} param: {}".format(decoder, handler, params))
    return handler(*params)


SERF_CODE_PROTO_M = 0b11100000
SERF_CODE_PROTO_VAL = 0b10100000
SERF_CODE_M = 0b00011111


def serf_encode(code, data):
    if (code & SERF_CODE_M) != code:
        print >> sys.stderr, "warning: code 0x%02X truncated to 0x%02X" % (code, code & SERF_CODE_M)

    frame = struct.pack("<B%is" % len(data), SERF_CODE_PROTO_VAL | (code & SERF_CODE_M), data)

    # sys.stderr.write('frame/raw:')
    # for byte in frame:
    #    sys.stderr.write(' %02X' % ord(byte))
    # sys.stderr.write('\n')

    frame = cobs.cobs_encode(frame) + '\0'

    # sys.stderr.write('frame/enc:')
    # for byte in frame:
    #    sys.stderr.write(' %02X' % ord(byte))
    # sys.stderr.write('\n')

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

    return ord(data[0]) & SERF_CODE_M, data[1:]


def handle_frame(code, data):
    # print("frame: code={:02X} data='{}'".format(code, data))
    return decode_code(code, data)


def next_frame():
    count = 0

    while 1:
        count += 1
        data = "Hello %i" % count
        print data
        yield encode_code_echo(data)
        yield encode_code_send(NMAC_SEND_DGRM, 0x4040, data)
        time.sleep(10)


def handle_recv(node, peer, dest, data):
    print("recv: @{:04X}:{:04X}->{:04X} #{:02X}".format(
        node, peer, dest, len(data)
    ))


def handle_status(version, serial, uptime, node, recv_total, send_total):
    print("status: version={:08X} serial={:016X} node={:04X} uptime={}ms recv={} send={}".format(
        version, serial, node, uptime, recv_total, send_total
    ))


def device_init(tty, baud):
    serial = get_serial(tty, baud)
    thr = input_start(serial)
    return thr, serial


def reset_device(serial, tty, baud):
    serial.write(encode_code_reset())
    time.sleep(.5)
    serial.reset_input_buffer()
    serial.close()
    time.sleep(3)
    return device_init(tty, baud)


def main(args):
    tty = args[0]
    baud = 115200

    try:
        thr, serial = device_init(tty, baud)

        serial.write(encode_code_status())
        thr, serial = reset_device(serial, tty, baud)
        serial.write(encode_code_status())

        # for frame in next_frame():
        #    write(frame)
        #    flush()

        thr.join()
        # sys.exit(0)

    # except Exception, e:
    #    print e
    except KeyboardInterrupt:
        print

    # sys.exit(1)


def input_thread(serial):
    try:
        data = ''

        while serial.isOpen():
            in_data = serial.read()

            if not in_data or not len(in_data):
                continue

            data = data + in_data

            if '\0' not in data:
                continue

            idx = data.index('\0')

            result = serf_decode(data[:idx])

            if result != SERF_DECODE_ERROR:
                handle_frame(*result)

            data = ''

    except Exception, e:
        # print e
        pass
    except KeyboardInterrupt:
        print

    # print "done"
    # sys.exit(1)


def input_start(serial):
    thr = threading.Thread(target=input_thread, args=(serial,))
    thr.setDaemon(True)
    thr.start()
    return thr


def get_serial(tty, baud):
    import serial
    ser = serial.Serial()
    ser.port = tty
    ser.baudrate = baud
    ser.timeout = .25
    ser.open()
    return ser


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage:", os.path.basename(sys.argv[0]), "<tty>"
        raise SystemExit

    main(sys.argv[1:])
