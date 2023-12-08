import MavManager

mavrouter = MavManager.MavManager(None)
mavrouter.connectGCS('udp:192.168.0.99:14450',True)
mavrouter.connectVehicle("/dev/ttyACM0")