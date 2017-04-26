#!/usr/bin/env python2
"""Cloud Chaser Support Library
"""

import os
import sys
import struct
import time
import random
import threading
import traceback
import cleanup
from serf import Serf


class CloudChaser(Serf):
    CODE_ID_ECHO = 0
    CODE_ID_STATUS = 1
    CODE_ID_SEND = 2
    CODE_ID_RECV = 3
    CODE_ID_RESET = 9

    RESET_MAGIC = 0xD1E00D1E

    NMAC_SEND_DGRM = 0
    NMAC_SEND_MESG = 1
    NMAC_SEND_TRXN = 2
    NMAC_SEND_STRM = 3

    def __init__(self, stats=None, handler=None):
        super(CloudChaser, self).__init__()
        
        self.stats = stats
        self.handler = handler

        self.add(
            name='echo',
            code=CloudChaser.CODE_ID_ECHO,
            encode=lambda mesg: struct.pack("%is" % (len(mesg) + 1), mesg + '\x00'),
            decode=lambda data: struct.unpack("%is" % len(data), data),
            handle=lambda mesg: sys.stdout.write(mesg)
        )

        self.add(
            name='reset',
            code=CloudChaser.CODE_ID_RESET,
            encode=lambda: struct.pack("<L", CloudChaser.RESET_MAGIC)
        )

        self.add(
            name='status',
            code=CloudChaser.CODE_ID_STATUS,
            decode=lambda data: struct.unpack("<LQLHLLLL%is" % (len(data) - 34), data)[:-1],
            handle=self.handle_status,
            response=CloudChaser.CODE_ID_STATUS
        )

        self.add(
            name='recv',
            code=CloudChaser.CODE_ID_RECV,
            decode=lambda data: struct.unpack("<HHHHbB%is" % (len(data) - 10), data),
            handle=self.handle_recv
        )

        self.add(
            name='send',
            code=CloudChaser.CODE_ID_SEND,
            encode=lambda typ, dest, data, flag=0, node=0: struct.pack(
                "<BBHHH%is" % len(data), typ & 0xFF, flag & 0xFF, node & 0xFFFF, dest & 0xFFFF, len(data), data
            )
        )

    def reset(self, reopen=True):
        self.io.reset()

        if reopen:
            self.reopen()
        else:
            self.close()

    def handle_status(self, version, serial, uptime, node, recv_count, recv_bytes, send_count, send_bytes):
        print("Cloud Chaser {:016X}@{:04X} up={}s rx={}/{} tx={}/{}".format(
            serial, node, uptime // 1000, recv_count, recv_bytes, send_count, send_bytes
        ))
        print

    def handle_recv(self, node, peer, dest, size, rssi, lqi, data):
        # print("recv: @{:04X}:{:04X}->{:04X} #{:02X} \t {}".format(
        #     node, peer, dest, len(data), ''  # '' '.join('{:02X}'.format(ord(ch)) for ch in data)
        # ))

        if self.stats is not None:
            self.stats.lock()
            if not self.stats.recv_count:
                self.stats.recv_time = time.time()
    
            self.stats.recv_size += len(data)
            self.stats.recv_count += 1
            self.stats.rssi_sum += rssi
            self.stats.lqi_sum += lqi
            self.stats.unlock()

        if self.handler is not None:
            self.handler(self, node, peer, dest, rssi, lqi, data)

        # self.io.send(CloudChaser.NMAC_SEND_STRM, peer, data)

        # data = ''.join([chr(random.randrange(0, 0xff+1)) for _ in range(random.randrange(1, 114))])
        # self.io.send(CloudChaser.NMAC_SEND_MESG, peer, data)


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
        elapsed -= elapsed % 5
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

        except SystemExit:
            sys.exit(0)

        except:
            traceback.print_exc()
            sys.exit(1)


tx = False


def send_frames(cc):
    count = 0

    while 1:
        count += 1
        # data = '\x3A' * 37
        data = ''.join([chr(random.randrange(0, 0xff+1)) for _ in range(random.randrange(1, 114))])
        cc.io.send(CloudChaser.NMAC_SEND_MESG, 0x0000, data)
        # time.sleep(.010)


def main(args):
    tty = args[0]
    baud = 115200

    reset = False
    global tx

    if len(args) > 1:
        if args[1] == 'tx':
            tx = True
        elif args[1] == 'reset':
            reset = True

    try:
        stats = None

        if not reset:
            stats = Stats()
            stats.start()

        cc = CloudChaser(stats)
        cc.open(tty, baud)

        if reset:
            print >>sys.stderr, "resetting...",
            cc.reset(reopen=False)
            print >>sys.stderr, "done."
            sys.exit(0)

        cleanup.install(lambda: sys.exit(1))

        cc.io.status()

        if tx:
            send_frames(cc)

        while not cc.join(1):
            pass

    except KeyboardInterrupt:
        pass

    except SystemExit:
        pass

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "usage:", os.path.basename(sys.argv[0]), "<tty>"
        raise SystemExit

    main(sys.argv[1:])
