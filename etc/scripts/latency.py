#!/usr/bin/env python2
"""Cloud Chaser Latency Test
"""

import os
import sys
import time
import threading
import traceback
from cc import CloudChaser

time_tx = 0
time_rx = 0
sem = threading.Semaphore(0)


def handle_ping(cc, node, peer, dest, rssi, lqi, data):
    # global time_rx
    # time_rx = time.time()
    # sem.release()
    cc.io.send(CloudChaser.NMAC_SEND_STRM, peer, data)


def handle_pong(cc, node, peer, dest, rssi, lqi, data):
    global time_rx
    time_rx = time.time()
    sem.release()


def main(tty1, tty2, *args):
    baud = 115200

    cc1 = CloudChaser(handler=handle_pong)
    cc1.open(tty1, baud)

    cc2 = CloudChaser(handler=handle_ping)
    cc2.open(tty2, baud)

    global time_tx

    while 1:
        time_tx = time.time()
        cc1.io.send(CloudChaser.NMAC_SEND_MESG, 0x0000, 'hello')
        sem.acquire()

        time_elapsed = 1000 * (time_rx - time_tx) / 2
        print "{:05.03f}ms".format(time_elapsed)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "usage:", os.path.basename(sys.argv[0]), "<tty> <tty>"
        raise SystemExit

    try:
        main(*sys.argv[1:])

    except KeyboardInterrupt:
        print()

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)
