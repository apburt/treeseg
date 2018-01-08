#!/usr/bin/env python

import os
import sys

for i in xrange(1,len(sys.argv)):
	tmp1 = sys.argv[i].split('s')
	tmp2 = tmp1[1].split('.')
	out = tmp2[0] + '.dat'
	print sys.argv[i],out
	command = 'mv '+sys.argv[i]+' '+out
	os.system(command)
