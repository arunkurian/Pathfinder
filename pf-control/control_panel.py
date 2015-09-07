from Tkinter import *	# Import Tkinter GUI module
import ttk
import serial 			# Import pySerial module
import time				# Import time module
import glob				# Import glob module

class App(Frame):

	# Initialize procedure
	def __init__(self):

		# Define title of box
		self.root = Tk()
		self.root.title("Pathfinder Control")
		self.root.resizable(width=FALSE, height=FALSE)

		# Define frame
		self.content = ttk.Frame(self.root, padding=(12, 12, 12, 12), width=400)
		self.content.pack_propagate(0)
		self.content.pack()
		self.content.bind_all('<Key>', self.keypress)

		self.frame = ttk.Frame(self.content)
		self.frame.grid(column=0, row=4, columnspan=3, rowspan=3, pady=12)

		# Run create widgets procedure
		self.create_widgets()

		# Execute main loop
		self.root.mainloop()

	# Define create widgets procedure
	def create_widgets(self):

		# List all ports of acceptable type
		self.ports = glob.glob('/dev/tty.[A-Za-z]*')

		# Port name label
		self.port_label = ttk.Label(self.content, text="Port", anchor=W, justify=LEFT)
		self.port_label.grid(column=0, row=0, columnspan=2, sticky=(N, W))

		# Port name select combobox
		self.port_select_var = StringVar()
		self.port_select = ttk.Combobox(self.content, textvariable=self.port_select_var, state="readonly")
		self.port_select['values'] = self.ports
		self.port_select.current(2)
		self.port_select.bind('<<ComboboxSelected>>')
		self.port_select.grid(column=1, row=0, columnspan=2, sticky=(N, S, E, W))

		# Port rate label
		self.rate_label = ttk.Label(self.content, text="Baud Rate", anchor=W, justify=LEFT)
		self.rate_label.grid(column=0, row=1, sticky=(N, W))

		# Port rate entry 
		self.port_rate = ttk.Entry(self.content)
		self.port_rate.insert(0,115200)				# Insert default value
		self.port_rate.config(width=10)
		self.port_rate.grid(column=1, row=1, columnspan=2, sticky=(N, S, E, W))

		# "Connect"/"Disconnect" button
		# Run connect_bluetooth procedure
		self.port_button = ttk.Button(self.content, text="Connect", command=self.connect_bluetooth)
		self.port_button.grid(column=0, row=2, columnspan=3, sticky=(N, S, E, W))

		# Connection results label
		self.port_result = ttk.Label(self.content)
		self.port_result_var = StringVar()
		self.port_result['textvariable'] = self.port_result_var
		self.port_result.grid(column=0, row=3, columnspan=3, sticky="nsew")

		# Drive "Forward" button
		# Run drive_forward procedure
		self.drive_forward_button = ttk.Button(self.frame, text="Forward", command=self.drive_forward)
		self.drive_forward_button.grid(column=1, row=4, sticky=(N, S, E, W))
		
		# Drive "Backward" button
		# Run drive_backward procedure
		self.drive_backward_button = ttk.Button(self.frame, text="Backward", command=self.drive_backward)
		self.drive_backward_button.grid(column=1, row=6, sticky=(N, S, E, W))
		
		# Turn "Left" button
		# Run turn_left procedure
		self.turn_left_button = ttk.Button(self.frame, text="Left", command=self.turn_left)
		self.turn_left_button.grid(column=0, row=5, sticky=(N, S, E, W))
		
		# Turn "Right" button
		# Run turn_right procedure
		self.turn_right_button = ttk.Button(self.frame, text="Right", command=self.turn_right)
		self.turn_right_button.grid(column=2, row=5, sticky=(N, S, E, W))

	# Define bluetooth connection procedure
	# called when user hits "Connect" or "Disconnect"
	def connect_bluetooth(self):

		# If connect_bluetooth procedure is run with intention to connect
		if self.port_button.config('text')[-1] == "Connect":

			# Attempt connection with the designated bluetooth serial port
			try:

				self.blue_port = self.port_select.get()
				self.blue_rate = self.port_rate.get()
				self.ser = serial.Serial(self.blue_port, self.blue_rate)
				time.sleep(1)
				self.port_result_var.set('Connected to ' + self.blue_port)
				self.port_select.configure(state="disabled")
				self.port_rate.configure(state="disabled")
				self.port_button.configure(text="Disconnect")

			# Return error if connection is not possible
			except Exception, err:

				self.port_result_var.set('Failed to connect. Try again! \n' + str(err))
				return None
		
		# If connect_bluetooth procedure is run with intention to disconnect	
		else:

			self.ser.flush()
			time.sleep(1)
			self.ser.close()
			self.blue_port = self.port_select.get()
			self.port_result_var.set('Disconected from ' + self.blue_port)
			self.port_select.configure(state="readonly")
			self.port_rate.configure(state="normal")
			self.port_button.configure(text="Connect")
	
	# Define drive forward procedure
	# called when user presses "Forward" or types 'w'
	def drive_forward(self):

		if self.port_button.config('text')[-1] != "Connect":

			self.ser.write('w')
			data = self.ser.readline()
			print data

	# Define drive backward procedure
	# called when user presses "Backward" or types 's'
	def drive_backward(self):

		if self.port_button.config('text')[-1] != "Connect":

			self.ser.write('s')
			data = self.ser.readline()
			print data

	# Define turn left procedure
	# called when user presses "Left" or types 'a'
	def turn_left(self):

		if self.port_button.config('text')[-1] != "Connect":

			self.ser.write('a')
			data = self.ser.readline()
			print data

	# Define turn right procedure
	# called when user presses "Right" or types 'd'
	def turn_right(self):

		if self.port_button.config('text')[-1] != "Connect":

			self.ser.write('d')
			data = self.ser.readline()
			print data

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