
# import os

# if os.name == "nt":
# 	import ctypes

# 	class _XinputGamepad(ctypes.Structure):
# 		_fields_ = [('Buttons', ctypes.c_ushort),
# 			('LeftTrigger', ctypes.c_ubyte),
# 			('RightTrigger', ctypes.c_ubyte),
# 			('LeftX', ctypes.c_short),
# 			('LeftY', ctypes.c_short),
# 			('RightX', ctypes.c_short),
# 			('RightY', ctypes.c_short)]

# 	class _XinputState(ctypes.Structure):
# 			_fields_ = [('Packet', ctypes.c_ulong), ('Gamepad', _XinputGamepad)]
# else:
# 	import struct

# class XboxController:
# 	ButtonNames = ('A', 'B', 'X', 'Y', 'LB', 'RB', 'Start', 'Select')

# 	def __init__(self, userIndex = 0):
# 		self.userIndex = userIndex
# 		self._clearState()
# 		self._onButton = [lambda: None] * 2 * len(XboxController.ButtonNames)

# 	def onButtonReleased(self, button, function):
# 		self._onButton[2 * XboxController.ButtonNames.index(button)] = function

# 	def onButtonPressed(self, button, function):
# 		self._onButton[2 * XboxController.ButtonNames.index(button) + 1] = function

# 	def _clearState(self):
# 		self.Connected = False
# 		self.LeftX = 0
# 		self.LeftY = 0
# 		self.RightX = 0
# 		self.RightY = 0
# 		self.LeftTrigger = 0
# 		self.RightTrigger = 0
# 		self.DpadX = 0
# 		self.DpadY = 0

# 		for button in XboxController.ButtonNames:
# 			self.__setattr__(button, int(0))
		
# 		self._previousButtons = int(0)

# 	def printState(self):
# 		print("\nDisconnected" if not self.Connected else (
# 			f"\nLeftX: {self.LeftX:.3f}\t\tLeftY: {self.LeftY:.3f}"
# 			f"\nRightX: {self.RightX:.3f}\t\tRightY: {self.RightY:.3f}"
# 			f"\nLeftTrigger: {self.LeftTrigger:.3f}\tRightTrigger: {self.RightTrigger:.3f}"
# 			f"\nDpadX: {self.DpadX}\t\tDpadY: {self.DpadY}"
# 			f"\nA: {self.A} \t\t\tB: {self.B}"
# 			f"\nX: {self.X} \t\t\tY: {self.Y}"
# 			f"\nLB: {self.LB}\t\t\tRB: {self.RB}"
# 			f"\nStart: {self.Start}\t\tSelect: {self.Select}"))

# 	if os.name == "nt":
# 		_xInputDll = getattr(ctypes.windll, "XInput1_4.dll")

# 		def __enter__(self):
# 			return self

# 		def __exit__(self, *args):
# 			return

# 		def update(self):
# 			state = _XinputState()
# 			if XboxController._xInputDll.XInputGetState(ctypes.c_uint32(self.userIndex), ctypes.byref(state)):
# 				self._clearState()
# 			else:
# 				self.Connected = True
# 				self.LeftX = state.Gamepad.LeftX / 0x8000
# 				self.LeftY = state.Gamepad.LeftY / 0x8000
# 				self.RightX = state.Gamepad.RightX / 0x8000
# 				self.RightY = state.Gamepad.RightY / 0x8000
# 				self.LeftTrigger = state.Gamepad.LeftTrigger / 0x100
# 				self.RightTrigger = state.Gamepad.RightTrigger / 0x100

# 				buttons = int(state.Gamepad.Buttons)
# 				self.DpadX = (0, -1, 1, 0)[buttons >> 2 & 0x03]
# 				self.DpadY = (0, 1, -1, 0)[buttons & 0x03]
# 				self.A = buttons >> 12 & 1
# 				self.B = buttons >> 13 & 1
# 				self.X = buttons >> 14 & 1
# 				self.Y = buttons >> 15 & 1
# 				self.LB = buttons >> 8 & 1
# 				self.RB = buttons >> 9 & 1
# 				self.Start = buttons >> 5 & 1
# 				self.Select = buttons >> 4 & 1

# 				for i, mask in enumerate((0x1000, 0x2000, 0x4000, 0x8000, 0x100, 0x200, 0x20, 0x10)):
# 					if ~self._previousButtons & buttons & mask:
# 						self._onButton[2 * i + 1]()
# 					elif self._previousButtons & ~buttons & mask:
# 						self._onButton[2 * i]()
# 				self._previousButtons = buttons

# 	else:
# 		def __enter__(self):
# 			self._connect()
# 			return self

# 		def __exit__(self, *args):
# 			os.close(self._fid)

# 		def _connect(self):
# 			try:
# 				self._fid = os.open(f"/dev/input/js{self.userIndex}", os.O_RDONLY | os.O_NONBLOCK)
# 			except (FileNotFoundError, PermissionError):
# 				return
# 			else:
# 				self.Connected = True

# 		def update(self):
# 			while self.Connected:
# 				try:
# 					eventValue, eventType, eventIndex = struct.unpack_from("hBB", os.read(self._fid, 8), offset = 4)
# 				except BlockingIOError:
# 					return
# 				except FileNotFoundError:
# 					self._clearState()
# 					return

# 				if eventType == 1:
# 					buttonValue = eventValue & 1
# 					self.__setattr__(XboxController.ButtonNames[eventIndex], buttonValue)
# 					self._onButton[2 * eventIndex + buttonValue]()
# 				elif eventType == 2:
# 					if eventIndex < 6:
# 						if eventIndex < 3:
# 							if eventIndex == 0:
# 								self.LeftX = eventValue / 0x8000
# 							elif eventIndex == 1:
# 								self.LeftY = -eventValue / 0x8000
# 							else:
# 								self.LeftTrigger = (eventValue + 0x7FFF) / 0x10000
# 						elif eventIndex == 3:
# 							self.RightX = eventValue / 0x8000
# 						elif eventIndex == 4:
# 							self.RightY = -eventValue / 0x8000
# 						else:
# 							self.RightTrigger = (eventValue + 0x7FFF) / 0x10000
# 					elif eventIndex == 6:
# 						self.DpadX = -1 if eventValue < 0 else 1 if eventValue > 0 else 0
# 					elif eventIndex == 7:
# 						self.DpadY = 1 if eventValue < 0 else -1 if eventValue > 0 else 0
# 			else:
# 				self._connect()
	
# 	@staticmethod
# 	def _deadband(x, db, sat):
# 		if x < 0: return -XboxController._deadband(-x, db, sat)
# 		return 0 if x <= db else 1 if x >= sat else (x - db) / (sat - db)






from threading import Thread
from inputs import get_gamepad
from sys import platform
from time import sleep

class XboxController:
	def __init__(self):
		self._clearState()
		self._updateThread = Thread(target = self._update)
		self._updateThread.start()
		self._onButtonPressed = {b: [] for b in ('A', 'B', 'X', 'Y', 'LB', 'RB')}

	def onButtonPressed(self, button, function):
		self._onButtonPressed[button].append(function)

	def _clearState(self):
		self.LeftX = 0
		self.LeftY = 0
		self.RightX = 0
		self.RightY = 0
		self.LeftTrigger = 0
		self.RightTrigger = 0
		self.DpadX = 0
		self.DpadY = 0
		self.A = 0
		self.B = 0
		self.X = 0
		self.Y = 0
		self.LB = 0
		self.RB = 0
		self.Select = 0
		self.Start = 0

	def _buttonEvent(self, button, state):
		if state != 0:
			for function in self._onButtonPressed[button]:
				function()

	def _update(self):
		win = platform == "win32"
		while True:
			try:
				events = get_gamepad()
				for event in events:
					if event.ev_type == "Absolute":
						stickRange = 0x8000
						triggerRange = 0x100 if win else 0x400
						if event.code == "ABS_X":
							self.LeftX = event.state / stickRange
						elif event.code == "ABS_Y":
							self.LeftY = -event.state / stickRange
						elif event.code == "ABS_RX":
							self.RightX = event.state / stickRange
						elif event.code == "ABS_RY":
							self.RightY = -event.state / stickRange
						elif event.code == "ABS_Z":
							self.LeftTrigger = event.state / triggerRange
						elif event.code == "ABS_RZ":
							self.RightTrigger = event.state / triggerRange
						elif event.code == "ABS_HAT0X":
							self.DpadX = event.state
						elif event.code == "ABS_HAT0Y":
							self.DpadY = -event.state
					elif event.ev_type == "Key":
						if event.code == "BTN_SOUTH":
							self.A = event.state
							self._buttonEvent('A', event.state)
						elif event.code == "BTN_EAST":
							self.B = event.state
							self._buttonEvent('B', event.state)
						elif event.code == "BTN_WEST":
							if win:
								self.X = event.state
								self._buttonEvent('X', event.state)
							else:
								self.Y = event.state
								self._buttonEvent('Y', event.state)
						elif event.code == "BTN_NORTH":
							if win:
								self.Y = event.state
								self._buttonEvent('Y', event.state)
							else:
								self.X = event.state
								self._buttonEvent('X', event.state)
						elif event.code == "BTN_TL":
							self.LB = event.state
							self._buttonEvent('LB', event.state)
						elif event.code == "BTN_TR":
							self.RB = event.state
							self._buttonEvent('RB', event.state)
						elif event.code == "BTN_START":
							if win:
								self.Start = event.state
							else:
								self.Select = event.state
								return
						elif event.code == "BTN_SELECT":
							if win:
								self.Select = event.state
								return
							else:
								self.Start = event.state
			except:
				self._clearState()
				sleep(0.2)
