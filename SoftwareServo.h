// SoftwareServo.h - Software timer driven Servo library for Esp8266
// 
// MIT License
// 
// Copyright (c) 2020 Björn Rennfanz <bjoernrennfanz@users.noreply.github.com>
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//   A servo is activated by creating an instance of the Servo class passing
//   the desired pin to the attach() method.
//   The servos are pulsed in the background using the value most recently
//   written using the write() method.
//
//   The methods are:
//
//   SoftwareServo - Class for manipulating servo motors connected to ESP8266 pins.
//
//   attach(pin)  - Attaches a servo motor to an i/o pin.
//   attach(pin, min, max) - Attaches to a pin setting min and max values in microseconds
//   default min is 1000, max is 2000
//
//   write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
//   writeMicroseconds() - Sets the servo pulse width in microseconds
//   read()      - Gets the last written servo pulse width as an angle between 0 and 180.
//   readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
//   attached()  - Returns true if there is a servo attached.
//   detach()    - Stops an attached servos from pulsing its i/o pin.

#ifndef SOFTWARESERVO_H
#define SOFTWARESERVO_H

#include <Arduino.h>

// The following values are in us (microseconds).
// Since the defaults can be overwritten in the new attach() member function,
// they were modified from the Arduino AVR defaults to be in the safe range
// of publically available specifications. While this implies that many 180°
// servos do not operate the full 0° to 180° sweep using these, it also prevents
// unsuspecting damage. For Arduino AVR, the same change is being discussed.
#define DEFAULT_MIN_PULSE_WIDTH      1000 // uncalibrated default, the shortest duty cycle sent to a servo
#define DEFAULT_MAX_PULSE_WIDTH      2000 // uncalibrated default, the longest duty cycle sent to a servo 
#define DEFAULT_NEUTRAL_PULSE_WIDTH  1500 // default duty cycle when servo is attached
#define REFRESH_INTERVAL                5 // classic default period to refresh servos in microseconds 
#define MAX_SERVOS                      4 // D0-D8

#if !defined(ESP8266)

#error "This library only supports esp8266 boards."

#endif

class SoftwareServo
{
public:
    SoftwareServo();
    ~SoftwareServo();

    // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure.
    // returns channel number or 0 if failure.
    uint8_t attach(int pin);
    
    // attach the given pin to the next free channel, sets pinMode, min, and max values for write().
    // returns channel number or 0 if failure.
    uint8_t attach(int pin, uint16_t min, uint16_t max);
    
    // attach the given pin to the next free channel, sets pinMode, min, and max values for write(),
    // and sets the initial value, the same as write().
    // returns channel number or 0 if failure.
    uint8_t attach(int pin, uint16_t min, uint16_t max, int value);
    void detach();
    void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
    void writeMicroseconds(int value); // Write pulse width in microseconds 
    int read();                        // returns current pulse width as an angle between 0 and 180 degrees
    int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
    bool attached();                   // return true if this servo is attached, otherwise false
private:
    int _id;
    bool _attached;
    uint16_t _minUs;                   
    uint16_t _maxUs;                   
};

#endif