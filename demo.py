import MavManager

mavrouter = MavManager.MavManager()
mavrouter.connectGCS('udp:192.168.0.99:14450',True)
mavrouter.connectInput("/dev/ttyACM0")