"""
RBE 550: Motion Planning
Final Project

Xbox Controller
Author: Daniel Moyer, dmoyer@wpi.edu

December 15, 2022

"""

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
