from Tkinter import *					# Import Tkinter GUI module
import ttk 								# Import ttk module
import serial 							# Import pySerial module
import time								# Import time module
import glob								# Import glob module
import collections						# Import collections module
from collections import OrderedDict		# Import OrderedDict

# Define application
class App(Frame):

	# Initialize procedure
	def __init__(self):

		# Define title of box
		self.root = Tk()
		self.root.title("Pathfinder Control")

		# Define main application frame
		self.frame = ttk.Frame(self.root)
		self.frame.grid(column=0, row=0, ipady=12, ipadx=12)
		self.frame.bind_all('<Key>', self.keypress)

		# Define paned windows
		self.control_panes = ttk.Panedwindow(self.frame, orient=HORIZONTAL)
		self.control_panes.pack()
		self.control_panes.pack_propagate(0)

		self.status_panes = ttk.Panedwindow(self.frame, orient=HORIZONTAL)
		self.status_panes.pack()
		self.status_panes.pack_propagate(0)

		# Define the label frames in each paned window
		self.setup = ttk.Labelframe(self.control_panes, text="Setup", padding=(12, 12, 12, 12))
		self.control_panes.add(self.setup)
		self.control = ttk.Labelframe(self.control_panes, text="Control", padding=(12, 12, 12, 12))
		self.control_panes.add(self.control)

		self.status = ttk.Labelframe(self.status_panes, text="Status", padding=(12, 12, 12, 12))
		self.status_panes.add(self.status)

		# Run create widgets procedure
		self.create_widgets()

		# Run sensor status update procedure
		self.update_status()

		# Execute main loop
		self.root.mainloop()

	# Define create widgets procedure
	# Called by __init__ to initially create widgets
	def create_widgets(self):

		# Setup Widgets

		# List all ports of acceptable type
		self.ports = glob.glob('/dev/tty.[A-Za-z]*')

		# Port name label
		self.port_label = ttk.Label(self.setup, text="Port", anchor=W, justify=LEFT)
		self.port_label.grid(column=0, row=0, columnspan=2, sticky=(N, W))

		# Port name select combobox
		self.port_select_var = StringVar()
		self.port_select = ttk.Combobox(self.setup, textvariable=self.port_select_var, state="readonly")
		self.port_select['values'] = self.ports
		self.port_select.current(2)
		self.port_select.bind('<<ComboboxSelected>>')
		self.port_select.grid(column=1, row=0, columnspan=2, sticky=(N, S, E, W))

		# Port rate label
		self.rate_label = ttk.Label(self.setup, text="Baud Rate", anchor=W, justify=LEFT)
		self.rate_label.grid(column=0, row=1, sticky=(N, W))

		# Port rate entry 
		self.port_rate = ttk.Entry(self.setup)
		self.port_rate.insert(0,115200)				# Insert default value
		self.port_rate.config(width=10)
		self.port_rate.grid(column=1, row=1, columnspan=2, sticky=(N, S, E, W))

		# "Connect"/"Disconnect" button, executes connect_bluetooth procedure
		self.port_button = ttk.Button(self.setup, text="Connect", command=self.connect_bluetooth)
		self.port_button.grid(column=0, row=2, columnspan=3, sticky=(N, S, E, W))

		# Connection results label
		self.port_result = ttk.Label(self.setup, wraplength=250)
		self.port_result_var = StringVar()
		self.port_result['textvariable'] = self.port_result_var
		self.port_result.grid(column=0, row=3, columnspan=3, sticky=(N, S, E, W))

		# Port connection boolean
		self.port_connected = None

		# Control Widgets

		# Autonomous/remote control radiobuttons
		self.control_select_var = IntVar()			# Variable of control selection
		self.auto_control_select = ttk.Radiobutton(self.control, text="Auto", variable=self.control_select_var, value=1, command=self.switch_control)
		self.auto_control_select.grid(column=0, row=0, sticky=(N, S, E, W))
		self.remote_control_select = ttk.Radiobutton(self.control, text="Remote", variable=self.control_select_var, value=0, command=self.switch_control)
		self.remote_control_select.grid(column=2, row=0, sticky=(N, S, E, W))

		# Separator between control mode and remote control buttons
		self.control_sep = ttk.Separator(self.control, orient=HORIZONTAL)
		self.control_sep.grid(column=0, row=1, columnspan=3, sticky=(N, S, E, W), pady=12)

		# Drive "Forward" button, run drive_forward prcoedure
		self.drive_forward_button = ttk.Button(self.control, text="Forward", command=self.drive_forward, state="disabled")
		self.drive_forward_button.grid(column=1, row=2, sticky=(N, S, E, W))
		
		# Drive "Backward" button, run drive_backward prcoedure
		self.drive_backward_button = ttk.Button(self.control, text="Backward", command=self.drive_backward, state="disabled")
		self.drive_backward_button.grid(column=1, row=3, sticky=(N, S, E, W))
		
		# Turn "Left" button, run turn_left prcoedure
		self.turn_left_button = ttk.Button(self.control, text="Left", command=self.turn_left, state="disabled")
		self.turn_left_button.grid(column=0, row=3, sticky=(N, S, E, W))
		
		# Turn "Right" button, run turn_right prcoedure
		self.turn_right_button = ttk.Button(self.control, text="Right", command=self.turn_right, state="disabled")
		self.turn_right_button.grid(column=2, row=3, sticky=(N, S, E, W))

		# Status Widgets

		# Initialize double variables for IR status
		self.ir_forward_var = DoubleVar()
		self.ir_left_var = DoubleVar()
		self.ir_right_var = DoubleVar()

		# Initialize double variables for time
		self.time_var = DoubleVar()

		# Define status dictionary
		self.status_vars = OrderedDict([('Forward IR', self.ir_forward_var),
										('Left IR', self.ir_left_var), 
										('Right IR', self.ir_right_var), 
										('Time', self.time_var)])

		# Forward IR label
		self.ir_forward_label = ttk.Label(self.status, text="Forward IR", justify=CENTER)
		self.ir_forward_label.grid(column=1, row=0)

		# Forward IR value
		self.ir_forward = ttk.Label(self.status,  justify=CENTER)
		self.ir_forward['textvariable'] = self.status_vars['Forward IR']
		self.ir_forward.grid(column=1, row=1)

		# Left IR label
		self.ir_left_label = ttk.Label(self.status, text="Left IR", justify=CENTER)
		self.ir_left_label.grid(column=0, row=2)

		# Left IR value
		self.ir_left = ttk.Label(self.status, justify=CENTER)
		self.ir_left['textvariable'] = self.status_vars['Left IR']
		self.ir_left.grid(column=0, row=3)

		# Right IR label
		self.ir_right_label = ttk.Label(self.status, text="Right IR", justify=CENTER)
		self.ir_right_label.grid(column=2, row=2)

		# Right IR value
		self.ir_right = ttk.Label(self.status, justify=CENTER)
		self.ir_right['textvariable'] = self.status_vars['Right IR']
		self.ir_right.grid(column=2, row=3)
		
		# Separator between IR and other status
		self.control_sep = ttk.Separator(self.status, orient=HORIZONTAL)
		self.control_sep.grid(column=0, row=4, columnspan=3, sticky=(N, S, E, W), pady=12)

		# Time label
		self.time_label = ttk.Label(self.status, text="Time", justify=LEFT)
		self.time_label.grid(column=0, row=5)

		# Time value
		self.time = ttk.Label(self.status,  justify=RIGHT)
		self.time['textvariable'] = self.status_vars['Time']
		self.time.grid(column=2, row=5)

	# Define bluetooth connection procedure
	# called when user hits "Connect" or "Disconnect"
	def connect_bluetooth(self):
				
		# If connect_bluetooth procedure is run with intention to disconnect	
		if (self.port_connected):

			self.ser.flush()
			time.sleep(1)
			self.ser.close()
			self.blue_port = self.port_select.get()
			self.port_result_var.set('Disconected from ' + self.blue_port)
			self.port_select.configure(state="readonly")
			self.port_rate.configure(state="normal")
			self.port_button.configure(text="Connect")
			self.port_connected = False
			self.remote_control(False)

		# If connect_bluetooth procedure is run with intention to connect
		else:

			# Attempt connection with the designated bluetooth serial port
			try:

				# Set serial settings and connect
				self.blue_port = self.port_select.get()
				self.blue_rate = self.port_rate.get()
				self.ser = serial.Serial(self.blue_port, self.blue_rate, timeout=5)
				self.ser.flushInput()
				self.ser.flushOutput()
				time.sleep(1)

				# Assume connected after 1 second, change widget states, initialize time
				self.port_result_var.set('Connected to ' + self.blue_port)
				self.port_select.configure(state="disabled")
				self.port_rate.configure(state="disabled")
				self.port_button.configure(text="Disconnect")
				self.port_connected = True
				self.remote_control(True)
				self.initial_time = time.time()

			# Return error if connection is not possible
			except Exception, err:

				self.port_result_var.set('Failed to connect. Try again! \n' + str(err))
				return None

	# Define switch control procedure
	# Called when user switches between autonomous and remote control
	def switch_control(self):

		if (self.port_connected):
			
			# Determine control mode selected
			mode = self.control_select_var.get()

			# Transmit control mode selected
			self.ser.write(bytes(mode))

			if (mode == 1):
				# Disable remote control buttons when autonomous mode is selected
				self.remote_control(True)
			else:
				self.remote_control(False)

	# Define remote control procedure
	# Called when remote control button states need to be changed
	def remote_control(self, state):

		# If under remote control, enable buttons
		if state == True:

			self.drive_forward_button.state(["!disabled"])
			self.drive_backward_button.state(["!disabled"])
			self.turn_left_button.state(["!disabled"])
			self.turn_right_button.state(["!disabled"])

		# If under autonomous control, disable button
		else: 

			self.drive_forward_button.state(["disabled"])
			self.drive_backward_button.state(["disabled"])
			self.turn_left_button.state(["disabled"])
			self.turn_right_button.state(["disabled"])

	# Define update status procedure
	# Runs continuously after port has been connected
	def update_status(self):

		if (self.port_connected):
			
			# Send serial "read" byte, read line of sensor data, and create data list
			self.ser.write('r')
			data = self.ser.readline()
			status = data.split(',')
			status = map(int, status)

			# Calculate elapsed time and append to status list
			elapsed_time = round(time.time() - self.initial_time, 2)
			status.append(elapsed_time)
			
			# Update status values
			for i in range(len(self.status_vars)):

				self.status_vars.values()[i].set(status[i])

		# Wait for dt before running again
		self.frame.after(200, self.update_status)

	# Define drive forward procedure
	# called when user presses "Forward" or types 'w'
	def drive_forward(self):

		if (self.port_connected):

			self.ser.write('w')

	# Define drive backward procedure
	# called when user presses "Backward" or types 's'
	def drive_backward(self):

		if (self.port_connected):

			self.ser.write('s')

	# Define turn left procedure
	# called when user presses "Left" or types 'a'
	def turn_left(self):

		if (self.port_connected):

			self.ser.write('a')

	# Define turn right procedure
	# called when user presses "Right" or types 'd'
	def turn_right(self):

		if (self.port_connected):

			self.ser.write('d')

	# Define keystroke procedure
	def keypress(self, event):

		# 'w' is forward
		if event.char is 'w':
			self.drive_forward_button.focus()
			self.drive_forward()

		# 'a' is forward
		elif event.char is 'a':
			self.turn_left_button.focus()
			self.turn_left()

		# 's' is forward
		elif event.char is 's':
			self.drive_backward_button.focus()
			self.drive_backward()

		# 'd' is forward
		elif event.char is 'd':
			self.turn_right_button.focus()
			self.turn_right()

		else:
			return None

# Execute App
if __name__ == '__main__':
	app = App()