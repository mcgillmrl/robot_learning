#! /usr/bin/env python

from sensor_msgs.msg import Joy


class JoyState:
  def __init__(self):
    self.type = ""
    self.LX = None
    self.LY = None
    self.RX = None
    self.RY = None
    self.A = None
    self.B = None
    self.X = None
    self.Y = None
    self.L1 = None
    self.L2 = None
    self.L3 = None
    self.R1 = None
    self.R2 = None
    self.R3 = None
    self.Select = None
    self.Start = None
    self.DL = None
    self.DR = None
    self.DU = None
    self.DD = None
  
  
  def fromJoyMsg(self, joy, deadzone=0.0):
    if not self.fromDualshock3USBJoyMsg(joy, deadzone):
      if not self.fromDualshock3BTSixAdMsg(joy, deadzone):
        if not self.fromLogitechXInputJoyMsg(joy, deadzone):
          if not self.fromLogitechDirectInputJoyMsg(joy, deadzone):
            if not self.fromDualshock3BTJoyMsg(joy, deadzone):
              return False
    return True


  def fromLogitechXInputJoyMsg(self, joy, deadzone=0.0):
    if len(joy.axes) == 8 and len(joy.buttons) == 11:
      self.type = "logitech"

      self.LX = -joy.axes[0]
      self.LY = joy.axes[1]
      self.RX = -joy.axes[3]
      self.RY = joy.axes[4]
      if (self.LX >= -deadzone and self.LX <= deadzone): self.LX = 0
      if (self.LY >= -deadzone and self.LY <= deadzone): self.LY = 0
      if (self.RX >= -deadzone and self.RX <= deadzone): self.RX = 0
      if (self.RY >= -deadzone and self.RY <= deadzone): self.RY = 0

      self.A = joy.buttons[0]
      self.B = joy.buttons[1]
      self.X = joy.buttons[2]
      self.Y = joy.buttons[3]
      self.L1 = joy.buttons[4]
      self.L2 = (joy.axes[2] < 0)
      self.L3 = joy.buttons[9]
      self.R1 = joy.buttons[5]
      self.R2 = (joy.axes[5] < 0)
      self.R3 = joy.buttons[10]
      self.Start = joy.buttons[7]
      self.Select = joy.buttons[6]

      self.DL = (joy.axes[6] == 1)
      self.DR = (joy.axes[6] == -1)
      self.DU = (joy.axes[7] == 1)
      self.DD = (joy.axes[7] == -1)
      return True
      
    return False  
    
    
  def fromLogitechDirectInputJoyMsg(self, joy, deadzone=0.0):
    if len(joy.axes) == 6 and len(joy.buttons) == 12:
      self.type = "logitech"

      self.LX = -joy.axes[0]
      self.LY = joy.axes[1]
      self.RX = -joy.axes[2]
      self.RY = joy.axes[3]
      if (self.LX >= -deadzone and self.LX <= deadzone): self.LX = 0
      if (self.LY >= -deadzone and self.LY <= deadzone): self.LY = 0
      if (self.RX >= -deadzone and self.RX <= deadzone): self.RX = 0
      if (self.RY >= -deadzone and self.RY <= deadzone): self.RY = 0

      self.A = joy.buttons[1]
      self.B = joy.buttons[2]
      self.X = joy.buttons[0]
      self.Y = joy.buttons[3]
      self.L1 = joy.buttons[4]
      self.L2 = joy.buttons[6]
      self.L3 = joy.buttons[10]
      self.R1 = joy.buttons[5]
      self.R2 = joy.buttons[7]
      self.R3 = joy.buttons[11]
      self.Start = joy.buttons[9]
      self.Select = joy.buttons[8]

      self.DL = (joy.axes[4] == 1)
      self.DR = (joy.axes[4] == -1)
      self.DU = (joy.axes[5] == 1)
      self.DD = (joy.axes[5] == -1)
      return True
      
    return False
    
    
  def fromDualshock3BTJoyMsg(self, joy, deadzone=0.0):
    if len(joy.axes) == 20 and len(joy.buttons) == 17:
      self.type = "ps3_bt_joy"

      self.LX = -joy.axes[0]
      self.LY = joy.axes[1]
      self.RX = -joy.axes[2]
      self.RY = joy.axes[3]
      if (self.LX >= -deadzone and self.LX <= deadzone): self.LX = 0
      if (self.LY >= -deadzone and self.LY <= deadzone): self.LY = 0
      if (self.RX >= -deadzone and self.RX <= deadzone): self.RX = 0
      if (self.RY >= -deadzone and self.RY <= deadzone): self.RY = 0

      self.A = joy.buttons[14]
      self.B = joy.buttons[13]
      self.X = joy.buttons[15]
      self.Y = joy.buttons[12]
      self.L1 = joy.buttons[10]
      self.L2 = joy.buttons[8]
      self.L3 = joy.buttons[1]
      self.R1 = joy.buttons[11]
      self.R2 = joy.buttons[9]
      self.R3 = joy.buttons[2]
      self.Start = joy.buttons[3]
      self.Select = joy.buttons[0]

      self.DL = joy.buttons[7]
      self.DR = joy.buttons[5]
      self.DU = joy.buttons[4]
      self.DD = joy.buttons[6]
      return True

    return False
    
    
  def fromDualshock3BTSixAdMsg(self, joy, deadzone=0.0):
    if len(joy.axes) == 29 and len(joy.buttons) == 17:
      self.type = "ps3_bt_sixad"

      self.LX = -joy.axes[0]
      self.LY = joy.axes[1]
      self.RX = -joy.axes[2]
      self.RY = joy.axes[3]
      if (self.LX >= -deadzone and self.LX <= deadzone): self.LX = 0
      if (self.LY >= -deadzone and self.LY <= deadzone): self.LY = 0
      if (self.RX >= -deadzone and self.RX <= deadzone): self.RX = 0
      if (self.RY >= -deadzone and self.RY <= deadzone): self.RY = 0

      self.A = joy.buttons[14]
      self.B = joy.buttons[13]
      self.X = joy.buttons[15]
      self.Y = joy.buttons[12]
      self.L1 = joy.buttons[10]
      self.L2 = joy.buttons[8]
      self.L3 = joy.buttons[1]
      self.R1 = joy.buttons[11]
      self.R2 = joy.buttons[9]
      self.R3 = joy.buttons[2]
      self.Start = joy.buttons[3]
      self.Select = joy.buttons[0]

      self.DL = joy.buttons[7]
      self.DR = joy.buttons[5]
      self.DU = joy.buttons[4]
      self.DD = joy.buttons[6]
      return True
      
    return False


  def fromDualshock3USBJoyMsg(self, joy, deadzone=0.0):
    if len(joy.axes) == 27 and len(joy.buttons) == 19:
      self.type = "ps3_usb"

      self.LX = -joy.axes[0]
      self.LY = joy.axes[1]
      self.RX = -joy.axes[2]
      self.RY = joy.axes[3]
      if (self.LX >= -deadzone and self.LX <= deadzone): self.LX = 0
      if (self.LY >= -deadzone and self.LY <= deadzone): self.LY = 0
      if (self.RX >= -deadzone and self.RX <= deadzone): self.RX = 0
      if (self.RY >= -deadzone and self.RY <= deadzone): self.RY = 0

      self.A = joy.buttons[14]
      self.B = joy.buttons[13]
      self.X = joy.buttons[15]
      self.Y = joy.buttons[12]
      self.L1 = joy.buttons[10]
      self.L2 = joy.buttons[8]
      self.L3 = joy.buttons[1]
      self.R1 = joy.buttons[11]
      self.R2 = joy.buttons[9]
      self.R3 = joy.buttons[2]
      self.Start = joy.buttons[3]
      self.Select = joy.buttons[0]

      self.DL = joy.buttons[7]
      self.DR = joy.buttons[5]
      self.DU = joy.buttons[4]
      self.DD = joy.buttons[6]
      return True

    return False
