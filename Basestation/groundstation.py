#!/usr/bin/python
import signal, sys, time, os, datetime
import serial, curses, pickle
import bitstring, numpy
import matplotlib.pyplot as plt

ser = serial.Serial(port='com5', baudrate=57600, timeout=0.1)
ser.flushInput()

data_dict = {'altitude':0, 'altitude_starting':0, 'altitude_max':0,'mode':0}
data_dict['gps'] = {'altitude':0, 'lat':0, 'long':0, 'speed':0, 'sats':0, 'mode':0, 'recent':0}
data_dict['ground_stats'] = {'num_pks_basic':0, 'num_pks_gps':0, 'num_pks_extra':0, 'num_pks_unknown':0}

#logfile naming
dt = datetime.datetime.now()
fileName = 'Data-%s-%s-%s_%s-%s-%s.csv' % (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
log_file = open(fileName, "w")

last_basic   = time.time()
last_gps     = time.time()
last_extra   = time.time()
last_unknown = time.time()

#adjust these values based on your location and map, lat and long are in decimal degrees
#TRX = -123.33515659247273 #top right longitude
#TRY = 44.983171093204625  #top right latitude
#BLX = -123.33956613486257 #bottom left longitude
#BLY = 44.98096497612233   #bottom left latitude
TRX = -123.3367622         #top right longitude
TRY = 45.0113628           #top right latitude
BLX = -123.3450571         #bottom left longitude 0.006611
BLY = 45.0073493           #bottom left latitude  0.0027238
last_lat = TRY

#Set up the Map:
plt.ion #interactive mode
#im = plt.imread('home.png') #lay the image under the graph
im = plt.imread('buhler.png') #lay the image under the graph
implot = plt.imshow(im,extent=[BLX, TRX, BLY, TRY]) #read a png file to map on
#Joshua:
path_collection = []
plt.draw()
plt.pause(0.05) #is necessary for the plot to update for some reason

def update_screen(screen):
	screen.clear()
	screen.border(0)
	
	#Header
	screen.addstr(2, 4, "RocketDuino Ground Control Station", curses.color_pair(1))
	screen.addstr(3, 16, "v 0.2.1", curses.color_pair(1))
	
	#Mode
	screen.addstr(6, 6, "MODE:      ", curses.color_pair(2))
	screen.addstr(6, 17, str(data_dict['mode']))
	if   data_dict['mode'] == 0:
		screen.addstr(6, 19, str("(Warmup)"), curses.color_pair(7))
	elif data_dict['mode'] == 1:
		screen.addstr(6, 19, str("(Ready)"), curses.color_pair(6))
	elif data_dict['mode'] == 2:
		screen.addstr(6, 19, str("(Flying)"), curses.color_pair(8))
	elif data_dict['mode'] == 3:
		screen.addstr(6, 19, str("(Recovery)"), curses.color_pair(7))
	
	screen.addstr(7, 6, "GPS MODE:  ", curses.color_pair(2))
	screen.addstr(7, 17, str(data_dict['gps']['mode']))
	#GpsMode:
	if   data_dict['gps']['mode'] == 0:
		screen.addstr(7, 19, str("(No Fix)"), curses.color_pair(7))
	elif data_dict['gps']['mode'] == 1:
		screen.addstr(7, 19, str("(2D/3D)"), curses.color_pair(8))
	elif data_dict['gps']['mode'] == 2:
		screen.addstr(7, 19, str("(DGPS)"), curses.color_pair(6))
	elif data_dict['gps']['mode'] == 3:
		screen.addstr(7, 19, str("(Bad Fix)"), curses.color_pair(7))
	screen.addstr(" (" + str(data_dict['gps']['sats']) + " sats) ", curses.color_pair(3))
	
	#GpsRecent:
	if   data_dict['gps']['recent'] == 0:
		screen.addstr(str("(GPS Disconnected)"), curses.color_pair(7))
	
	#Altitude measurements:
	screen.addstr(10, 6, "max alt:   ", curses.color_pair(2))
	screen.addstr(11, 6, "altitude:  ", curses.color_pair(2))
	screen.addstr(12, 6, "start alt: ", curses.color_pair(2))
	
	screen.addstr(10, 17, str(data_dict['altitude_max']))
	screen.addstr(" (m)", curses.color_pair(3))
	screen.addstr(11, 17, str(data_dict['altitude']))
	screen.addstr(" (m)", curses.color_pair(3))
	screen.addstr(12, 17, str(data_dict['altitude_starting']))
	screen.addstr(" (m)", curses.color_pair(3))
	
	screen.addstr(10, 26, str(round(data_dict['altitude_max'] * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	screen.addstr(11, 26, str(round(data_dict['altitude'] * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	screen.addstr(12, 26, str(round(data_dict['altitude_starting'] * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	
	#Now above ground level:
	screen.addstr(10, 37, "agl: ", curses.color_pair(2))
	screen.addstr(11, 37, "agl: ", curses.color_pair(2))
	
	screen.addstr(10, 42, str(data_dict['altitude_max'] - data_dict['altitude_starting']))
	screen.addstr(" (m)", curses.color_pair(3))
	screen.addstr(11, 42, str(data_dict['altitude'] - data_dict['altitude_starting']))
	screen.addstr(" (m)", curses.color_pair(3))
	
	screen.addstr(10, 51, str(round((data_dict['altitude_max'] - data_dict['altitude_starting']) * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	screen.addstr(11, 51, str(round((data_dict['altitude'] - data_dict['altitude_starting']) * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	
	#GPS DATA
	screen.addstr(14, 6, "GPS DATA: ", curses.color_pair(2))
	screen.addstr(15, 6, "altitude: ", curses.color_pair(2))
	screen.addstr(16, 6, "speed:    ", curses.color_pair(2))
	screen.addstr(17, 6, "lat:      ", curses.color_pair(2))
	screen.addstr(18, 6, "long:     ", curses.color_pair(2))
	
	screen.addstr(15, 17, str(data_dict['gps']['altitude']))
	screen.addstr(" (m)", curses.color_pair(3))
	screen.addstr(16, 17, str(data_dict['gps']['speed']))
	screen.addstr(" (m/s)", curses.color_pair(3))
	screen.addstr(17, 17, str(round(data_dict['gps']['lat'], 7)))
	screen.addstr(18, 17, str(round(data_dict['gps']['long'], 7)))
	
	screen.addstr(15, 28, str(round(data_dict['gps']['altitude'] * 3.28)))
	screen.addstr(" (ft)", curses.color_pair(3))
	screen.addstr(16, 28, str(round(data_dict['gps']['speed'] * 2.237)))
	screen.addstr(" (mph)", curses.color_pair(3))
	
	#Received Packets
	screen.addstr(20, 6, "Basic Packet: ", curses.color_pair(1))
	screen.addstr(str(data_dict['ground_stats']['num_pks_basic']))
	if ((time.time() - last_basic) > 1.5):
		screen.addstr(" (" + str(round(time.time() - last_basic, 1)) + "s ago)")
	
	screen.addstr(21, 6, "  GPS Packet: ", curses.color_pair(1))
	screen.addstr(str(data_dict['ground_stats']['num_pks_gps']))
	if ((time.time() - last_gps) > 3):
		screen.addstr(" (" + str(round(time.time() - last_gps, 1)) + "s ago)")
	
	screen.addstr(22, 6, "     Unknown: ", curses.color_pair(5))
	screen.addstr(str(data_dict['ground_stats']['num_pks_unknown']))
	if ((time.time() - last_unknown) < 60):
		screen.addstr(" (" + str(round(time.time() - last_unknown, 1)) + "s ago)")
	
	#screen.addstr(19, 6, "Last Packet:")
	#screen.addstr(19, 6, "Last Packet:")
	#screen.addstr(19, 6, "Last Packet:")
	
	screen.addstr(25, 2, "Please enter a command...", curses.color_pair(1))
	screen.addstr(26, 4, "1 - Dump all current data as pickle", curses.color_pair(1))
	screen.addstr(27, 4, "q - Quit Ground Control", curses.color_pair(1))
	
	#Return after 1ms and get more data
	screen.timeout(1)

def update_map():
	global path_collection
	
	#calc position
	pos_y = data_dict['gps']['lat']
	pos_x = data_dict['gps']['long'] #longitude is negative
	
	new_point = plt.scatter([pos_x], [pos_y])
	#new_point = plt.plot([pos_x], [pos_y], 'rx')
	
	#Save the point so we can remove it later
	path_collection.append(new_point)
	
	#If we have enough points, remove the oldest
	if len(path_collection) > 60:
		path_collection[0].remove() #Remove the first point from the map
		path_collection.pop(0)      #Remove the first point from the list
	
	#Joshua
	plt.draw()
	plt.pause(0.05) #is necessary for the plot to update for some reason

def handle_key(screen):
	x = screen.getch()
	if x == ord('q'):
		signal_handler(1,1)
	elif x == ord('1'):
		dump_history_pickle()

def signal_handler(signum, frame):
	curses.endwin()
	print("Closing sockets...")
	log_file.close()
	sys.exit(1)

def dump_history_pickle():
	pickle.dump(data_stream, open('backup.p', 'wb'))

def write_log():
	print(str(round(time.time(), 1)), end =",", file=log_file)
	print(str(data_dict['mode']), end =",", file=log_file)
	print(str(data_dict['altitude']), end =",", file=log_file)
	print(str(data_dict['altitude_starting']), end =",", file=log_file)
	print(str(data_dict['altitude_max']), end =",", file=log_file)
	print(str(data_dict['gps']['altitude']), end =",", file=log_file)
	print(str(round(data_dict['gps']['lat'], 8)), end =",", file=log_file)
	print(str(round(data_dict['gps']['long'], 8)), end =",", file=log_file)
	print(str(data_dict['gps']['speed']), end =",", file=log_file)
	print(str(data_dict['gps']['sats']), end =",", file=log_file)
	print(str(data_dict['gps']['mode']), end =",", file=log_file)
	print(str(data_dict['gps']['recent']), file=log_file)

def decode_basic():
	global last_basic, data_dict

	packet = ser.read(5)
	bits = bitstring.BitArray(packet)
	#print (bits.bin)
	altitude, altitude_max, mode = bits.unpack('intle:16, intle:16, uint:8')
	data_dict['altitude'] = altitude
	data_dict['mode'] = mode
	data_dict['altitude_max'] = altitude_max
	
	#Update Packet Number & history
	data_dict['ground_stats']['num_pks_basic'] += 1
	last_basic = time.time()
	write_log()

def decode_gps():
	global last_gps, data_dict
	
	packet = ser.read(13)
	bits = bitstring.BitArray(packet)
	#print (bits.bin)
	lat, long, gpsAlt, gpsSpeedMPS, lastByte = bits.unpack('floatle:32, floatle:32, uintle:16, uintle:16, bits:8')
	padd, gpsRecent, gpsMode, sats = lastByte.unpack('uint:1, uint:1, uint:2, uint:4') #Parse the last byte seperately
	data_dict['gps']['lat'] = lat
	data_dict['gps']['long'] = long
	data_dict['gps']['altitude'] = gpsAlt
	data_dict['gps']['speed'] = gpsSpeedMPS
	data_dict['gps']['sats'] = sats
	data_dict['gps']['mode'] = gpsMode
	data_dict['gps']['recent'] = gpsRecent
	
	#Update Packet Number & history
	data_dict['ground_stats']['num_pks_gps'] += 1
	last_gps = time.time()
	write_log()
	log_file.flush()

def decode_extra():
	global data_dict
	
	packet = ser.read(2)
	bits = bitstring.BitArray(packet)
	#print (bits.bin)
	starting = bits.unpack('intle:16')
	data_dict['altitude_starting'] = starting[0]

#set up signal handler(s)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGABRT, signal_handler)

screen = curses.initscr()
curses.start_color()
curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
curses.init_pair(3, curses.COLOR_BLUE, curses.COLOR_BLACK)
curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLACK)
curses.init_pair(5, curses.COLOR_RED, curses.COLOR_BLACK)
curses.init_pair(6, curses.COLOR_BLACK, curses.COLOR_GREEN)
curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_RED)
curses.init_pair(8, curses.COLOR_BLACK, curses.COLOR_CYAN)

while True:
	bytes_avail = ser.in_waiting
	
	#Basic Packet
	if bytes_avail % 5 == 0 and bytes_avail > 4:
		decode_basic()
		continue
	
	#gps packet
	elif bytes_avail == 13:
		decode_gps()
		continue
	
	#extra packet
	elif bytes_avail == 2:
		decode_extra()
		continue
		
	# Invalid data (too much?)
	elif bytes_avail > 13:
		print("extra data in buffer: dropping", bytes_avail, "bytes")
		ser.reset_input_buffer()
		data_dict['ground_stats']['num_pks_unknown'] += 1
		last_unknown = time.time()
	
	#Update the screen
	update_screen(screen)
	handle_key(screen)
	
	#Update the map
	plt.draw()
	plt.pause(0.01)
	
	if (last_lat is not data_dict['gps']['lat']):
		if (data_dict['gps']['lat'] < BLY):
			continue
		if (data_dict['gps']['lat'] > TRY):
			continue
		if (data_dict['gps']['long'] < BLX):
			continue
		if (data_dict['gps']['long'] > TRX):
			continue
		update_map()
		last_lat = data_dict['gps']['lat']