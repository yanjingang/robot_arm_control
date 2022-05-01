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



#ifndef BTJoystick_h
#define BTJoystick_h

#include "Arduino.h"

//These are our button constants
#define BTB_SELECT      0x0001
#define BTB_L3          0x0002
#define BTB_R3          0x0004
#define BTB_START       0x0008
#define BTB_PAD_UP      0x0010
#define BTB_PAD_RIGHT   0x0020
#define BTB_PAD_DOWN    0x0040
#define BTB_PAD_LEFT    0x0080
#define BTB_L2          0x0100
#define BTB_R2          0x0200
#define BTB_L1          0x0400
#define BTB_R1          0x0800
#define BTB_GREEN       0x1000
#define BTB_RED         0x2000
#define BTB_BLUE        0x4000
#define BTB_PINK        0x8000
#define BTB_TRIANGLE    0x1000
#define BTB_CIRCLE      0x2000
#define BTB_CROSS       0x4000
#define BTB_SQUARE      0x8000
#define BTB_A    0x1000
#define BTB_B    0x2000
#define BTB_C    0x4000
#define BTB_D    0x8000

//These are stick values
#define PSS_RXX 0
#define PSS_RYY 1
#define PSS_LXX 2
#define PSS_LYY 3

class BTJoystick {
  private:
    // Input string buffer
    String _inputBuffer;
    // Output stream buffer
    String _outputBuffer;
    // Stream to read commands and to write output.
    Stream &_stream;

    /**
       Parse input buffer and detect command.

       @return COMMAND - Returns detected command. Returns COMMAND_UNKNOWN if failed.
    */
    boolean _decodeCommand();
    void evaluateCommand();
    unsigned char BT2data[4];
    unsigned int last_buttons;
    unsigned int buttons=0xFFFF;
    uint8_t CURRENTPORT=0;
    byte inBuf[12];
  public:
    /**
       Constructor

       @param Command commands - Array of commands (pointer to array of pointers of objects with Command interface).
       @param Strem stream - Input and output stream.
    */
    BTJoystick(
      Stream &stream
    );
    /**
       Read command from stream and execute it.
    */
    void readCommand();

    boolean Button(uint16_t);                //will be TRUE if button is being pressed
    unsigned int ButtonDataByte();
    boolean NewButtonState();
    boolean NewButtonState(unsigned int);    //will be TRUE if button was JUST pressed OR released
    boolean ButtonPressed(unsigned int);     //will be TRUE if button was JUST pressed
    boolean ButtonReleased(unsigned int);    //will be TRUE if button was JUST released
    byte Analog(byte);

};

#endif
