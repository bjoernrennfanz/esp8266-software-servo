// SoftwareServo.cpp - Software timer driven Servo library for Esp8266
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

#if defined(ESP8266)

#include <SoftwareServo.h>
#include <Arduino.h>
extern "C" {
#include "user_interface.h"
}

// similiar to map but will have increased accuracy that provides a more
// symmetrical api (call it and use result to reverse will provide the original value)
int improved_map(int value, int minIn, int maxIn, int minOut, int maxOut)
{
    const int rangeIn = maxIn - minIn;
    const int rangeOut = maxOut - minOut;
    const int deltaIn = value - minIn;

    // fixed point math constants to improve accurancy of divide and rounding
    constexpr int fixedHalfDecimal = 1;
    constexpr int fixedDecimal = fixedHalfDecimal * 2;

    return ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + minOut;
}

// Internal timers
static os_timer_t software_servo_timer;
static int software_servo_pins[MAX_SERVOS] = {-1,-1,-1,-1};
static int software_servo_pulse_lenght[MAX_SERVOS] = {DEFAULT_NEUTRAL_PULSE_WIDTH,
    DEFAULT_NEUTRAL_PULSE_WIDTH,DEFAULT_NEUTRAL_PULSE_WIDTH,DEFAULT_NEUTRAL_PULSE_WIDTH
};

static unsigned int software_servo_current_servo = 0;
static int software_servo_map = 0;

static int software_servo_next_servo[MAX_SERVOS] = {1,2,3,0};
static int software_servo_time_to_next_pulse[MAX_SERVOS] = {REFRESH_INTERVAL,
    REFRESH_INTERVAL,REFRESH_INTERVAL,REFRESH_INTERVAL
};

// Find best time between generated pulses
int find_times_between_pulses(void)
{
    int tmp_time,find,i,ii,finded;

    finded = 0;
    for (int i = 0 ; i < MAX_SERVOS; i++)
    {
        find = 0;
        tmp_time = REFRESH_INTERVAL;
        for (int ii = 0 ; ii < MAX_SERVOS; ii++)
        {
            if (software_servo_pins[(ii+i+1) % MAX_SERVOS] == (-1) && find == 0)
            {
                tmp_time += REFRESH_INTERVAL;
            }
            else
            {
                find = 1;
                software_servo_next_servo[i] = (ii+i+1) % MAX_SERVOS;
                software_servo_time_to_next_pulse[i] = tmp_time;
                break;
            }
        }

        finded += find;
   }

  return finded;
}

void software_servo_timer_tick() // software servo timer function
{
    if ((software_servo_map != 0) && (software_servo_time_to_next_pulse[software_servo_current_servo] >= 0))
    {
        os_timer_disarm(&software_servo_timer); // dis_arm the timer
        os_timer_setfn(&software_servo_timer, (os_timer_func_t *)software_servo_timer_tick, NULL); // set the timer function, dot get os_timer_func_t to force function convert
        os_timer_arm(&software_servo_timer, software_servo_time_to_next_pulse[software_servo_current_servo], 1); // arm the timer every 5ms and repeat

        if (software_servo_pins[software_servo_current_servo] >= 0)
        {
            digitalWrite(software_servo_pins[software_servo_current_servo], HIGH);
            os_delay_us(software_servo_pulse_lenght[software_servo_current_servo]);
            digitalWrite(software_servo_pins[software_servo_current_servo], LOW);
      }

      software_servo_current_servo = software_servo_next_servo[software_servo_current_servo];
   }
   else
   {
      os_timer_disarm(&software_servo_timer); // dis_arm the timer
   }
}

//-------------------------------------------------------------------
// Servo class methods

SoftwareServo::SoftwareServo()
{
    _id = -1;
    _attached = false;
    _minUs = DEFAULT_MIN_PULSE_WIDTH;
    _maxUs = DEFAULT_MAX_PULSE_WIDTH;
}

SoftwareServo::~SoftwareServo() 
{
    detach();
}

uint8_t SoftwareServo::attach(int pin)
{
    return attach(pin, DEFAULT_MIN_PULSE_WIDTH, DEFAULT_MAX_PULSE_WIDTH);
}

uint8_t SoftwareServo::attach(int pin, uint16_t minUs, uint16_t maxUs)
{
    return attach(pin, minUs, maxUs, DEFAULT_NEUTRAL_PULSE_WIDTH);
}

uint8_t SoftwareServo::attach(int pin, uint16_t minUs, uint16_t maxUs, int value)
{
    if (!_attached) 
    {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);

        // Find first free pin
        for (int i = 0 ; i < MAX_SERVOS; i++)
        {
            // Search first free slot
            if (software_servo_pins[i] == -1)
            {
                software_servo_pins[i] = pin;
                software_servo_pulse_lenght[i] = value;
                
                _id = i;
                break;
            }
        }

        // Check free slot was found
        if (_id == -1)
        {
            return 0;
        }

        _attached = true;
    }

  // keep the min and max within 200-3000 us, these are extreme
  // ranges and should support extreme servos while maintaining
  // reasonable ranges
  _maxUs = max((uint16_t)250, min((uint16_t)3000, maxUs));
  _minUs = max((uint16_t)200, min(_maxUs, minUs));

  write(value);

  return pin;
}

void SoftwareServo::detach()
{
    if (_attached) 
    {
        software_servo_map &= ~(1 << _id);
        software_servo_pulse_lenght[_id] = DEFAULT_NEUTRAL_PULSE_WIDTH;

        _id = -1;
        _attached = false;
    }
}

void SoftwareServo::write(int value)
{
    // treat any value less than 200 as angle in degrees (values equal or larger are handled as microseconds)
    if (value < 200) 
    {
        // assumed to be 0-180 degrees servo
        value = constrain(value, 0, 180);
        value = improved_map(value, 0, 180, _minUs, _maxUs);
    }

    writeMicroseconds(value);
}

void SoftwareServo::writeMicroseconds(int value)
{
    value = constrain(value, _minUs, _maxUs);
    if (_attached) 
    {
        software_servo_pulse_lenght[_id] = value;
        if (find_times_between_pulses() > 0)
        {
            software_servo_map |= (1 << _id);
            os_timer_disarm(&software_servo_timer); // dis_arm the timer
            os_timer_setfn(&software_servo_timer, (os_timer_func_t *)software_servo_timer_tick, NULL); // set the timer function, dot get os_timer_func_t to force function convert
            os_timer_arm(&software_servo_timer, software_servo_time_to_next_pulse[software_servo_current_servo], 1); // arm the timer every 5ms and repeat
        }
        else
        {
            software_servo_map &= ~(1 << _id);
            os_timer_disarm(&software_servo_timer); // dis_arm the timer
        }
    }
}

int SoftwareServo::read() // return the value as degrees
{
  // read returns the angle for an assumed 0-180
  return improved_map(readMicroseconds(), _minUs, _maxUs, 0, 180);
}

int SoftwareServo::readMicroseconds()
{
    if (_attached)
    {
        return software_servo_pulse_lenght[_id];
    }

    return 0;
}

bool SoftwareServo::attached()
{
  return _attached;
}


#endif