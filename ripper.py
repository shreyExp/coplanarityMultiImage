#!/usr/bin/python3
import sys
import json as js
f = open(sys.argv[1], 'r')
l = js.loads(f.read())
nl = []
for d in l:
	nl.append({"left": d["centre"], "right": d["right"]})
print(js.dumps(nl))
