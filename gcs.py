from pymavlink import mavutil
import time


# PyMAVLink has an issue that received messages which contain strings
# cannot be resent, because they become Python strings (not bytestrings)
# This converts those messages so your code doesn't crash when
# you try to send the message again.
def fixMAVLinkMessageForForward(msg):
    msg_type = msg.get_type()
    if msg_type in ('PARAM_VALUE', 'PARAM_REQUEST_READ', 'PARAM_SET'):
        if type(msg.param_id) == str:
            msg.param_id = msg.param_id.encode()
    elif msg_type == 'STATUSTEXT':
        if type(msg.text) == str:
            msg.text = msg.text.encode()
    return msg


# Modified from the snippet in your question
# UDP will work just as well or better
gcs_conn = mavutil.mavlink_connection('udp:localhost:14450', input=False)
gcs_conn.wait_heartbeat()
print(f'Heartbeat from system (system {gcs_conn.target_system} component {gcs_conn.target_system})')
