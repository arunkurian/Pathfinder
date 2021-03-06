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

		# Define paned windows
		self.controlPanes = ttk.Panedwindow(self.frame, orient=HORIZONTAL)
		self.controlPanes.pack()
		self.controlPanes.pack_propagate(0)

		self.commandPanes = ttk.Panedwindow(self.frame, orient=HORIZONTAL)
		self.commandPanes.pack()
		self.commandPanes.pack_propagate(0)

		# Define the label frames in each paned window
		self.setup = ttk.Labelframe(self.controlPanes, text="Setup", padding=(12, 12, 12, 12))
		self.controlPanes.add(self.setup)

		self.queueShapes = ttk.Labelframe(self.controlPanes, text="Shapes Queue", padding=(12, 12, 12, 12))
		self.controlPanes.add(self.queueShapes)

		self.pickShapes = ttk.Labelframe(self.commandPanes, text="Pick Shapes", padding=(12, 12, 12, 12))
		self.commandPanes.add(self.pickShapes)

		# Run create widgets procedure
		self.createWidgets()

		# Run shapes update procedure
		self.updateShapes()

		# Execute main loop
		self.root.mainloop()

	# Define create widgets procedure
	# Called by __init__ to initially create widgets
	def createWidgets(self):

		# Setup Widgets

		# List all ports of acceptable type
		self.ports = glob.glob('/dev/tty.[A-Za-z]*')

		# Port name label
		self.portLabel = ttk.Label(self.setup, text="Port", anchor=W, justify=LEFT)
		self.portLabel.grid(column=0, row=0, columnspan=2, sticky=(N, W))

		# Port name select combobox
		self.portSelectVar = StringVar()
		self.portSelect = ttk.Combobox(self.setup, textvariable=self.portSelectVar, state="readonly")
		self.portSelect['values'] = self.ports
		self.portSelect.current(2)
		self.portSelect.bind('<<ComboboxSelected>>')
		self.portSelect.grid(column=1, row=0, columnspan=2, sticky=(N, S, E, W))

		# Port rate label
		self.portRateLabel = ttk.Label(self.setup, text="Baud Rate", anchor=W, justify=LEFT)
		self.portRateLabel.grid(column=0, row=1, sticky=(N, W))

		# Port rate entry 
		self.portRate = ttk.Entry(self.setup)
		self.portRate.insert(0,115200)				# Insert default value
		self.portRate.config(width=10)
		self.portRate.grid(column=1, row=1, columnspan=2, sticky=(N, S, E, W))

		# "Connect"/"Disconnect" button, executes connectBluetooth procedure
		self.portButton = ttk.Button(self.setup, text="Connect", command=self.connectBluetooth)
		self.portButton.grid(column=0, row=2, columnspan=3, sticky=(N, S, E, W))

		# Connection results label
		self.portResult = ttk.Label(self.setup, wraplength=250)
		self.portResultVar = StringVar()
		self.portResult['textvariable'] = self.portResultVar
		self.portResult.grid(column=0, row=3, columnspan=3, sticky=(N, S, E, W))

		# Port connection boolean
		self.portConnected = None

		# Status Widgets

		# Circle button
		self.circleButton = ttk.Button(self.pickShapes, text="Circle", command=lambda: self.sendDesiredShape('c'))
		self.circleButton.grid(column=0, row=0, sticky=(N, S, E, W))

		# Square button
		self.squareButton = ttk.Button(self.pickShapes, text="Square", command=lambda: self.sendDesiredShape('s'))
		self.squareButton.grid(column=1, row=0, sticky=(N, S, E, W))
		
		# Triangle button
		self.triangleButton = ttk.Button(self.pickShapes, text="Triangle", command=lambda: self.sendDesiredShape('t'))
		self.triangleButton.grid(column=2, row=0, sticky=(N, S, E, W))
		
		# Cross button
		self.crossButton = ttk.Button(self.pickShapes, text="Cross", command=lambda: self.sendDesiredShape('x'))
		self.crossButton.grid(column=3, row=0, sticky=(N, S, E, W))

		# Shape status label
		self.shapeStatus = ttk.Label(self.queueShapes, wraplength=150)
		self.shapeStatusVar = StringVar()
		self.shapeStatus['textvariable'] = self.shapeStatusVar
		self.shapeStatus.grid(column=0, row=2, columnspan=3, sticky=(N, S, E, W))

		# Shape List
		self.shapesList = Listbox(self.queueShapes, height=5)
		self.shapesList.grid(column=0, row=0, sticky=(N, S, E, W))

	# Define bluetooth connection procedure
	# called when user hits "Connect" or "Disconnect"
	def connectBluetooth(self):
				
		# If connectBluetooth procedure is run with intention to disconnect	
		if (self.portConnected):

			self.arduino.flush()
			time.sleep(1)
			self.arduino.close()
			self.btPort = self.portSelect.get()
			self.portResultVar.set('Disconected from ' + self.btPort)
			self.portSelect.configure(state="readonly")
			self.portRate.configure(state="normal")
			self.portButton.configure(text="Connect")
			self.portConnected = False

		# If connectBluetooth procedure is run with intention to connect
		else:

			# Attempt connection with the designated bluetooth serial port
			try:

				# Set serial settings and connect
				self.btPort = self.portSelect.get()
				self.btRate = self.portRate.get()
				self.arduino = serial.Serial(self.btPort, self.btRate, timeout=0.1)
				time.sleep(1)
				self.arduino.write('1')

				# Verify connection with handshake
				if (self.arduino.read(1) == '1'):

					self.portResultVar.set('Connected to ' + self.btPort)
					self.portSelect.configure(state="disabled")
					self.portRate.configure(state="disabled")
					self.portButton.configure(text="Disconnect")
					self.arduino.flushInput()
					self.arduino.flushOutput()
					self.portConnected = True

			# Return error if connection is not possible
			except Exception, err:

				self.portResultVar.set('Failed to connect. Try again! \n' + str(err))
				return None

	# Define update shapes procedure
	# Runs continuously after port has been connected
	def updateShapes(self):

		if (self.portConnected):

			if (self.shapesList.size() > 0):

				incomingCmd = self.arduino.read(4)

				if (incomingCmd == 'pfcv'):

					shape = self.arduino.read(1)

					shapeType = self.shapesList.get(0)
					self.deleteShape()

					if (shape == '0'):
						self.shapeStatusVar.set('Error: ' + shapeType + ' not found!')
					elif (shape == '1'):
						self.shapeStatusVar.set(shapeType + ' found. Turning left!')
					elif (shape == '2'):
						self.shapeStatusVar.set(shapeType + ' found. Straight ahead!')
					elif (shape == '3'):
						self.shapeStatusVar.set(shapeType + ' found. Turning right!')

			else:

				self.shapeStatusVar.set('Pick a shape!')

		else:

			self.shapeStatusVar.set('Connect to Pathfinder!')

		# Wait for dt before running again
		self.frame.after(200, self.updateShapes)

	def deleteShape(self):

		last = self.shapesList.size()
		self.shapesList.selection_clear(0, last)

		self.shapesList.delete(0)
		self.shapesList.select_set(0)

	def sendDesiredShape(self, shape):

		shapeType = ''

		if (shape == 'c'):
			shapeType = 'Circle'
		elif (shape == 's'):
			shapeType = 'Square'
		elif (shape == 't'):
			shapeType = 'Triangle'
		elif (shape == 'x'):
			shapeType = 'Cross'

		self.shapesList.insert(END, shapeType)
		self.shapesList.select_set(0)

		if (self.portConnected):

			self.arduino.write(shape)

# Execute App
if __name__ == '__main__':
	app = App()