from __future__ import print_function

import time

from pymavlink import mavutil
#from pymavlink import mavlinkv10 as mavlink

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("srcport", type=int)
parser.add_argument("dstport", type=int)

args = parser.parse_args()

msrc = mavutil.mavlink_connection('udp:localhost:14550', planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)

mdst = mavutil.mavlink_connection('udp:localhost:14450', planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)


# simple basic byte pass through, no logging or viewing of packets, or analysis etc
msrc.logfile_raw = mdst
mdst.logfile_raw = msrc

while True:
  # L -> R
    l = msrc.recv_match();
    if l is not None:
       l_last_timestamp = 0
       if  l.get_type() != 'BAD_DATA':
           l_timestamp = getattr(l, '_timestamp', None)
           if not l_timestamp:
               l_timestamp = l_last_timestamp
           l_last_timestamp = l_timestamp

       print("--> %s.%02u: %s\n" % (
           time.strftime("%Y-%m-%d %H:%M:%S",
                         time.localtime(l._timestamp)),
           int(l._timestamp*100.0)%100, l))

  # R -> L
    r = mdst.recv_match();
    if r is not None:
       r_last_timestamp = 0
       if r.get_type() != 'BAD_DATA':
           r_timestamp = getattr(r, '_timestamp', None)
           if not r_timestamp:
               r_timestamp = r_last_timestamp
           r_last_timestamp = r_timestamp

       print("<-- %s.%02u: %s\n" % (
           time.strftime("%Y-%m-%d %H:%M:%S",
                         time.localtime(r._timestamp)),
           int(r._timestamp*100.0)%100, r))
