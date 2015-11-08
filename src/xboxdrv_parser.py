#!/usr/bin/python

import sys
import re

while True:
	line = sys.stdin.readline().rstrip()
	line = re.sub(r":\s*", ":", line)
	data = line.split(' ')
	for item in data:
		pair = item.split(":")
		if pair[0] == "LT":
			print pair[1]
