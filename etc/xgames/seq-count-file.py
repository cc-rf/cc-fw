"""
"""
import os
import sys

name = 'stream.txt'

f = file(name)
lines = f.readlines()
f.close()

search_pre = '; IMU '
search_post = ':'

seqs = []

for line in lines:
	idx_pre = line.find(search_pre)

	if idx_pre < 0:
		continue

	idx_pre += len(search_pre)
	line = line[idx_pre:]

	idx_post = line.find(search_post)

	if idx_post < 0:
		continue

	seq = int(line[:idx_post])
	seqs.append(seq)

missed = 0

for idx in xrange(1, len(seqs)):
	seq_prev = seqs[idx-1]
	seq_cur = seqs[idx]
	missed += seq_cur - seq_prev - 1

total = missed + len(seqs)

print "missed %i packets in stream of %i for a presumed total of %i" % (missed, len(seqs), total)

print "calculated receive rate: %i packets/sec" % (120*len(seqs)/total)