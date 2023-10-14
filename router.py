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

mdst = mavutil.mavlink_connection('udpout:192.168.0.105:14450', planner_format=False,
                                  notimestamps=True,
                                  robust_parsing=True)


# simple basic byte pass through, no logging or viewing of packets, or analysis etc
while True:
  # L -> R
    m = msrc.recv_match();
    if m is not None:
      mdst.mav.send(m);
  # R -> L
    m2 = mdst.recv_match();
    if m2 is not None:
      msrc.mav.send(m2);
