/******************************************************************
   Super amazing BT controller Arduino Library v1.8
      http://www.7gp.cn

   

  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  <http://www.gnu.org/licenses/>

******************************************************************/

#include "BTJoystick.h"


BTJoystick::BTJoystick(
  Stream &stream
) :
  _stream(stream)
{
}


/****************************************************************************************/
boolean BTJoystick::NewButtonState() {
  return ((last_buttons ^ buttons) > 0);
}

/****************************************************************************************/
boolean BTJoystick::NewButtonState(unsigned int button) {
  return (((last_buttons ^ buttons) & button) > 0);
}

/****************************************************************************************/
boolean BTJoystick::ButtonPressed(unsigned int button) {
  return (NewButtonState(button) & Button(button));
}

/****************************************************************************************/
boolean BTJoystick::ButtonReleased(unsigned int button) {
  return ((NewButtonState(button)) & ((~last_buttons & button) > 0));
}

/****************************************************************************************/
boolean BTJoystick::Button(uint16_t button) {
  return ((~buttons & button) > 0);
}

/****************************************************************************************/
unsigned int BTJoystick::ButtonDataByte() {
  return (~buttons);
}

/****************************************************************************************/
byte BTJoystick::Analog(byte button) {
  return BT2data[button];
}


boolean BTJoystick::_decodeCommand() {
  _outputBuffer = "";

  if (_inputBuffer.startsWith("M=")) { //M=byte byte byte int
    evaluateCommand();
    return true;
  }

  return false;
}

void BTJoystick::readCommand() {
  int pkSize = 0;
  while (_stream.available()) {
    byte incomingChar = _stream.read();
    inBuf[CURRENTPORT++] = incomingChar;
    if (CURRENTPORT >= 12) {
      CURRENTPORT = 0;
      evaluateCommand();
    }
  }
}

void BTJoystick::evaluateCommand()

{
  for (int i = 0; i < 4; i++) {
    BT2data[i] =map( inBuf[i + 3],0,20,0,255);  
  }
  unsigned int _buttons = 0;
  last_buttons = buttons;
  _buttons = inBuf[7];
  _buttons = inBuf[8] | (_buttons << 8);
  _buttons = inBuf[9] | (_buttons << 8);
  _buttons = inBuf[10] | (_buttons << 8);
  buttons = ~_buttons;
}
