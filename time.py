#!/usr/bin/env python3
import datetime
now = datetime.datetime.now()
time = now.timestamp()
print(time)
dt1 = datetime.datetime.fromtimestamp(time)
print(dt1.strftime('%Y-%m-%d - %H:%M:%S.%f'))
# print(dt1)
