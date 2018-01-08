#!/usr/bin/env python

import os
import glob

fnames = glob.glob("stem_*.txt")
for i in xrange(len(fnames)):
	tmp1 = fnames[i].split("_")
	tmp2 = tmp1[1].split(".")
	tmp3 = tmp2[0]
	cyl_name = "cylinder_" + tmp3 + ".txt"
	run_name = "/Applications/CloudCompare.app/Contents/MacOS/CloudCompare "+fnames[i] + " " + cyl_name
	print run_name
	os.system(run_name)
