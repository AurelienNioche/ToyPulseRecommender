import serial
import json


import matplotlib.pyplot as plt


def main():

	with serial.Serial('/dev/cu.usbmodem112201', 115200, timeout=.1) as arduino:
		while True:
			data = arduino.readline().strip()

			if data:
				data = data.decode()
				print("drop", data)
				break

		fig, ax = plt.subplots()
		lines, = ax.plot([], [])

		timestamp = []
		bpm = []
		spO2 = []
		while True:
			data = arduino.readline().strip()
			if data:
				data = data.decode()
				try:
					dic_data = json.loads(data)
					print("timestamp", dic_data["timestamp"],
						  "bpm", dic_data["bpm"],
						  "spO2", dic_data["spO2"])
					timestamp.append(dic_data["timestamp"])
					bpm.append(dic_data["bpm"])
					spO2.append(dic_data["spO2"])
					#ax.plot(timestamp, bpm)
					#fig.canvas.draw_idle()
					lines.set_xdata(timestamp)
					lines.set_ydata(bpm)

					# Need both of these in order to rescale
					ax.relim()
					ax.autoscale_view()

					fig.canvas.draw_idle()
					fig.canvas.flush_events()

					plt.pause(0.05)

				except (UnicodeDecodeError, json.decoder.JSONDecodeError):
					print("Not data", data)


if __name__ == "__main__":
	main()


