#!/usr/bin/env python
"""Dump xgames packet data.

Format from xgames.h in Curie code below. The tag field in the header
is always '0' 'A' 'T'.

struct __xyz {
	int16_t x;
	int16_t y;
	int16_t z;
} __packed;

struct __header {
	char tag[3];
	uint8_t dev_id[6];
	uint8_t id;
	uint8_t p_counter[3];
} __packed;

/* Packet Structs */
struct xgames_packet_imu {
	struct __header header;
	struct __xyz accel;
	struct __xyz gyro;
	uint8_t pressure[3];
} __packed;

struct xgames_packet_gps {
	struct xgames_packet_imu imu;
	uint8_t bat;
	int32_t lat;
	int32_t lng;
	uint16_t speed;
	uint8_t gps_status;
} __packed;

struct xgames_packet_status {
	struct __header header;
	uint8_t bat;
	uint8_t gps_status;
} __packed;

"""
import sys, os

PKT_ID_IMU          = 1
PKT_ID_GPS          = 6
PKT_ID_STATUS       = 7

PKT_DATA_LEN_MAP    = { PKT_ID_IMU: 15, PKT_ID_GPS: 27, PKT_ID_STATUS: 2 }

data_prev = ''
seq_only = True


def dump(data):
    global data_prev

    data = data_prev + data

    if len(data) < 13:
        data_prev = data
        return
    else:
        data_prev = ''

    print " ".join("{:02x}".format(ord(c)) for c in data), "\t",
    print "".join(c if ord(c) in range(33,128) else "." for c in data)
    #print " ".join(c.rjust(2) if ord(c) in range(33,128) else "  " for c in data)
    #print

def parse(data):
    global data_prev

    if len(data_prev):
        data = data_prev + data
        data_prev = ''


    while len(data):
        i = data.find('0AT')
        if i < 0:
            break

        if i > 0:
            print >>sys.stderr, "discarding %i bytes" % i
            data = data[i:]

        if len(data) < 10:
            #print >>sys.stderr, "not enough data remaining (%i)" % len(data)
            break

        dev_id = ':'.join("{:02x}".format(ord(c)) for c in data[3:9])
        pkt_id = ord(data[9])
        p_counter = sum(ord(c)<<(8*i) for i,c in enumerate(data[10:13]))

        data_len = PKT_DATA_LEN_MAP.get(pkt_id, 0)
        #print "data_len=%i len(data[13:])=%i" % (data_len, len(data[13:]))

        if data_len > len(data[13:]):
            break

        p_data = " ".join("{:02x}".format(ord(c)) for c in data[13:13+data_len+1])
        data = data[13+data_len+1:]

        if not seq_only:
            print "packet:"
            print "  dev_id  = %s" % dev_id
            print "  pkt_id  = %i" % pkt_id
            print "  p_cntr  = %i" % p_counter
            print "  p_data  = %s" % p_data
            print
        else:
            print str(p_counter)
            sys.stdout.flush()

    if len(data):
        data_prev = data


def main(args):
    tty = args[0]
    baud = 230400
    try:
        if len(args) > 1:
            baud = int(args[1])

        serial = get_serial(tty, baud)
        read = serial.read

        while True:
            parse(read())
    except Exception, e:
        print >>sys.stderr, e
    except KeyboardInterrupt:
        print >>sys.stderr

def get_serial(tty, baud):
    import serial
    ser = serial.Serial()
    ser.port = tty
    ser.baudrate = baud
    ser.stopbits = 1
    ser.parity = 'N'
    ser.open()
    return ser

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print >>sys.stderr, "usage:", os.path.basename(sys.argv[0]), "<tty>", "[<baud>]"
        raise SystemExit

    main(sys.argv[1:])
