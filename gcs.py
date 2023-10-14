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
gcs_conn = mavutil.mavlink_connection('udp:192.168.0.105:14450', input=False)


vehicle = mavutil.mavlink_connection('udp:localhost:14550')
vehicle.wait_heartbeat()
print(f'Heartbeat from system (system {vehicle.target_system} component {vehicle.target_system})')

while True:
    # Don't block for a GCS message - we have messages
    # from the vehicle to get too
    gcs_msg = gcs_conn.recv_match(blocking=False)
    if gcs_msg is None:
        pass
    elif gcs_msg.get_type() != 'BAD_DATA':
        if gcs_msg.get_type() == 'HEARTBEAT':
            print(gcs_msg)
        # We now have a message we want to forward. Now we need to
        # make it safe to send
        gcs_msg = fixMAVLinkMessageForForward(gcs_msg)

        # Finally, in order to forward this, we actually need to
        # hack PyMAVLink so the message has the right source
        # information attached.
        vehicle.mav.srcSystem = gcs_msg.get_srcSystem()
        vehicle.mav.srcComponent = gcs_msg.get_srcComponent()

        # Only now is it safe to send the message
        vehicle.mav.send(gcs_msg)
        
    vcl_msg = vehicle.recv_match(blocking=False)
    if vcl_msg is None:
        pass
    elif vcl_msg.get_type() != 'BAD_DATA':
        # We now have a message we want to forward. Now we need to
        # make it safe to send
        if vcl_msg.get_type() == 'HEARTBEAT':
            print(vcl_msg)
        vcl_msg = fixMAVLinkMessageForForward(vcl_msg)

        # Finally, in order to forward this, we actually need to
        # hack PyMAVLink so the message has the right source
        # information attached.
        gcs_conn.mav.srcSystem = vcl_msg.get_srcSystem()
        gcs_conn.mav.srcComponent = vcl_msg.get_srcComponent()

        gcs_conn.mav.send(vcl_msg)
        

    # Don't abuse the CPU by running the loop at maximum speed
    time.sleep(0.001)
