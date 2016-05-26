"""
"""
import os
import sys
import fileinput

missed = 0
count = 0
total_seen = 0
prev = -1

for line in fileinput.input():
	seq = int(line)
        count += 1
        total_seen += 1
        if prev < 0:
            prev = seq
            continue

        gap = seq - prev - 1
        missed += gap
        prev = seq

        if count >= 120:
            total = count + missed
            rate = 120.0 * count / total
            print "%*i pkt/sec\t\tN=%i" % (3, int(rate), total_seen)
            count = 0
            missed = 0
