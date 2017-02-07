#!/usr/bin/env python2
"""Serial Framed RF Device Communication Protocol
"""

import os
import random
import sys
import struct
import time
import cobs
import threading
import traceback

CODE_ID_ECHO = 0
CODE_ID_STATUS = 1
CODE_ID_SEND = 2
CODE_ID_RECV = 3
CODE_ID_RESET = 9

RESET_MAGIC = 0xD1E00D1E

status_sem = threading.Semaphore(0)


def handle_code_echo(data):
    sys.stdout.write(data)


def handle_code_status(*args):
    handle_status(*args)


def handle_code_recv(node, peer, dest, size, rssi, lqi, data):
    if len(data) != size:
        print >>sys.stderr, "recv: size/{} != len(data)/{}".format(size, len(data))
        return

    handle_recv(node, peer, dest, rssi, lqi, data)


def decode_code_echo(data):
    return struct.unpack("%is" % len(data), data)


def decode_code_status(data):
    return struct.unpack("<LQLHLLLL%is" % (len(data) - 34), data)[:-1]


def decode_code_recv(data):
    return struct.unpack("<HHHHbB%is" % (len(data) - 10), data)


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
    flag = 0
    node = 0
    size = len(data)
    return serf_encode(
        CODE_ID_SEND,
        struct.pack(
            "<BBHHH%is" % size,
            typ & 0xFF, flag & 0xFF, node & 0xFFFF, dest & 0xFFFF, size, data
        )
    )


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
        print >> sys.stderr, "serf: empty data"
        return SERF_DECODE_ERROR

    if (ord(data[0]) & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL:
        print >> sys.stderr, "serf: bad proto val"
        return SERF_DECODE_ERROR

    return ord(data[0]) & SERF_CODE_M, data[1:]


def handle_frame(code, data):
    # print("frame: code={:02X} data='{}'".format(code, data))
    try:
        return decode_code(code, data)
    except:
        traceback.print_exc()


def serf_status(serial):
    serial.write(encode_code_status())


def serf_send(serial, typ, dest, data):
    serial.write(encode_code_send(typ, dest, data))


def serf_echo(serial, data):
    serial.write(encode_code_echo(data))

# -


class Stats:
    recv_count = 0
    recv_time = 0
    recv_size = 0
    rssi_sum = 0
    lqi_sum = 0

    _start_time = 0
    _lock = None

    def __init__(self):
        self._lock = threading.Lock()

    def start(self):
        self._start_time = time.time()
        self.run()

    def lock(self):
        self._lock.acquire()

    def unlock(self):
        self._lock.release()

    def run(self):
        thr = threading.Timer(5, self._run)
        thr.setDaemon(True)
        thr.start()

        if not self.recv_count:
            return

        self.lock()

        recv_count = self.recv_count
        recv_time = self.recv_time
        recv_size = self.recv_size
        rssi_sum = self.rssi_sum
        lqi_sum = self.lqi_sum

        self.recv_count = 0
        self.recv_size = 0
        self.rssi_sum = 0
        self.lqi_sum = 0

        self.unlock()

        now = time.time()

        diff = now - recv_time

        if diff:
            d_rate = int(round(float(recv_size) / diff))
            p_rate = int(round(float(recv_count) / diff))
        else:
            d_rate = 0
            p_rate = 0

        if recv_count:
            rssi_avg = rssi_sum / recv_count
            lqi_avg = lqi_sum / recv_count
        else:
            rssi_avg = 0
            lqi_avg = 0

        elapsed = int(round(now - self._start_time))
        elapsed_hour = elapsed / 3600
        elapsed_min = (elapsed / 60) % 60
        elapsed_sec = elapsed % 60

        print(
            "{:02d}:{:02d}:{:02d}  {:5d} Bps / {:3d} pps \t rssi {:<4d}  lqi {:<2d}".format(
                elapsed_hour, elapsed_min, elapsed_sec, d_rate, p_rate, rssi_avg, lqi_avg
            )
        )

        # TODO: Maybe also add totals to this output ^

    def _run(self):
        try:
            self.run()
        except KeyboardInterrupt:
            sys.exit(0)

        except:
            traceback.print_exc()
            sys.exit(1)

stats = Stats()


def handle_recv(node, peer, dest, rssi, lqi, data):
    # print("recv: @{:04X}:{:04X}->{:04X} #{:02X} \t {}".format(
    #     node, peer, dest, len(data), ' '.join('{:02X}'.format(ord(ch)) for ch in data)
    # ))

    stats.lock()
    if not stats.recv_count:
        stats.recv_time = time.time()

    stats.recv_size += len(data)
    stats.recv_count += 1
    stats.rssi_sum += rssi
    stats.lqi_sum += lqi
    stats.unlock()


def handle_status(version, serial, uptime, node, recv_count, recv_bytes, send_count, send_bytes):
    print("Cloud Chaser {:016X}@{:04X} up={}s rx={}/{} tx={}/{}".format(
        serial, node, uptime // 1000, recv_count, recv_bytes, send_count, send_bytes
    ))
    print

    status_sem.release()


def reset_device(serial, tty, baud):
    serial.timeout = .25
    serial.write(encode_code_reset())
    time.sleep(.5)
    serial.reset_input_buffer()
    serial.close()
    time.sleep(1.5)
    return get_serial(tty, baud)


def main(args):
    tty = args[0]
    baud = 115200

    reset = False
    tx = False

    if len(args) > 1:
        if args[1] == 'tx':
            tx = True
        elif args[1] == 'reset':
            reset = True

    try:
        if not reset:
            stats.start()

        serial = get_serial(tty, baud)

        if reset:
            print >>sys.stderr, "resetting...",
            reset_device(serial, tty, baud)
            print >>sys.stderr, "done."
            sys.exit(0)

        thr = input_start(serial)

        serf_status(serial)
        status_sem.acquire()

        if tx:
            send_frames(serial)

        time.sleep(0.25)

        while not thr.join(0.25):
            pass

        # sys.exit(0)

    except KeyboardInterrupt:
        sys.exit(0)

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)


def send_frames(serial):
    count = 0

    while 1:
        count += 1
        # data = '\x3A' * 48
        data = ''.join([chr(random.randrange(0, 0xff+1)) for _ in range(random.randrange(32, 110))])
        serf_send(serial, NMAC_SEND_MESG, 0x0000, data)
        # serf_send(serial, random.choice((1, 3)), 0x0000, data)
        # time.sleep(.1)


def input_thread(serial):
    try:
        data = ''

        while serial.isOpen():
            in_data = serial.read()

            if not in_data or not len(in_data):
                print >> sys.stderr, "input: empty"
                continue

            data = data + in_data

            if '\0' not in data:
                continue

            idx = data.index('\0')

            result = serf_decode(data[:idx])

            if result != SERF_DECODE_ERROR:
                handle_frame(*result)

            data = ''

    except KeyboardInterrupt:
        sys.exit(0)

    except:
        traceback.print_exc()


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
    # ser.timeout = .25
    ser.open()
    return ser


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage:", os.path.basename(sys.argv[0]), "<tty>"
        raise SystemExit

    main(sys.argv[1:])
