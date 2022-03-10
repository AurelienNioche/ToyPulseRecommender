import os

frequency = 500 # Hertz
duration  = 2000 # milliseconds

os.system('play -v 2.0 -n synth %s sin %s' % (duration/1000, frequency))