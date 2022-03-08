import serial
import json
arduino = serial.Serial('/dev/cu.usbmodem112201', 115200, timeout=.1)
while True:
	data = arduino.readline().strip()
	if data:
		data = data.decode()
		try:
			dic_data = json.loads(data)
			print("timestamp", dic_data["timestamp"],
				  "bpm", dic_data["bpm"],
				  "spO2", dic_data["spO2"])

		except (UnicodeDecodeError, Exception):
			print("Not data", data)


