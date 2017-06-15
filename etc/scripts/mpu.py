#!/usr/bin/env python2
"""Cloud Chaser Support Library: MPU I/O Flavor
"""
from __future__ import print_function
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
    CODE_ID_RESET = 9
    CODE_ID_MPU_CONFIG = 11
    CODE_ID_MPU_CTRL = 12
    CODE_ID_MPU_CALIB = 13
    CODE_ID_MPU_STREAM = 14

    RESET_MAGIC = 0xD1E00D1E

    NMAC_SEND_DGRM = 0
    NMAC_SEND_MESG = 1
    NMAC_SEND_TRXN = 2
    NMAC_SEND_STRM = 3

    def __init__(self, handler=None):
        super(CloudChaser, self).__init__()

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
            name='mpu_config',
            code=CloudChaser.CODE_ID_MPU_CONFIG,
            encode=lambda op, mpu_id, accel, gyro: struct.pack("<BBffffff", op & 0xFF, mpu_id & 0xFF, *(accel + gyro)),
            decode=lambda data: struct.unpack("<BBffffff", data)[:1],
            response=CloudChaser.CODE_ID_MPU_CONFIG
        )

        self.add(
            name='mpu_ctrl',
            code=CloudChaser.CODE_ID_MPU_CTRL,
            encode=lambda stream: struct.pack("<B", stream & 0xFF)
        )

        self.add(
            name='mpu_calibrate',
            code=CloudChaser.CODE_ID_MPU_CALIB,
            encode=lambda auto_lowpass=0: struct.pack("<B", 2 if auto_lowpass else 1),
            decode=lambda data: struct.unpack("<B", data),
            response=CloudChaser.CODE_ID_MPU_CALIB
        )

        def decode_mpu_stream(data):
            count, timestamp = struct.unpack("<BL", data[0:5])
            data = data[5:]
            items = {}

            while count:
                item_data, data = data[:37], data[37:]
                item = struct.unpack("<Bfffffffff", item_data)
                item_id = item[0]
                item = item[1:]
                items[item_id] = item

                if 0 < len(data) < 37:
                    print("remaining junk data: {} bytes".format(len(data)), file=sys.stderr)
                    break

                count -= 1

            return timestamp, items,

        self.add(
            name='mpu_stream',
            code=CloudChaser.CODE_ID_MPU_STREAM,
            decode=decode_mpu_stream,
            handle=self.handle_mpu_stream
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
        print()

    def handle_mpu_stream(self, timestamp, items):

        print("@ {}:".format(timestamp))

        for item_id, item_data in items.items():
            print("  {} [{},{},{}] [{},{},{}]".format(item_id, *item_data))


def main(tty, *args):
    baud = 115200

    calibrate = False
    quickdump = False

    if args:
        if args[0] == "calibrate":
            calibrate = True
        elif args[0] == "dump":
            quickdump = True

    try:
        cc = CloudChaser()
        cc.open(tty, baud)

        cc.io.status()

        if calibrate:
            print("calibrating...", file=sys.stderr)
            result = cc.io.mpu_calibrate(auto_lowpass=0)
            print("result={}".format(result), file=sys.stderr)

            print("setting low pass...", file=sys.stderr)
            result = cc.io.mpu_config(op=1, mpu_id=0xFF, accel=[0.05, 0.05, 0.05], gyro=[0.7, 0.7, 0.7])
            print("result={}".format(result), file=sys.stderr)

        elif quickdump:
            print("starting", file=sys.stderr)

            cc.io.mpu_ctrl(stream=1)
            time.sleep(0.200)
            cc.io.mpu_ctrl(stream=0)

            # def cc_cleanup():
            #     cc.io.mpu_ctrl(stream=0)
            #     os._exit(0)

            # cleanup.install(cc_cleanup)

        # while not cc.join(1):
        #     pass

    except KeyboardInterrupt:
        pass

    except SystemExit:
        raise

    except:
        traceback.print_exc()
        sys.exit(1)

    sys.exit(0)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage:", os.path.basename(sys.argv[0]), "<tty>", file=sys.stderr)
        sys.exit(1)

    main(*sys.argv[1:])
