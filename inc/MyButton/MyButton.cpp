/*

MyButton library

See MyButton.h


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/


#include "MyButton.h"
#define DEBOUNCE_TIME   50            // wait 50ms after each pin changes


#define  IS_PRESS    _BV(0)
#define  WAS_PRESS   _BV(1)
#define  ON_PRESS    _BV(2)
#define  ON_RELEASE  _BV(3)
#define  IS_HOLD     _BV(4)
#define  ON_HOLD     _BV(5)


uint64_t holdTime = 500;                  // default Hold time = 500 ms

MyButton::MyButton(GPIO_TypeDef* GPIOx, uint16_t pin) {
    _pin = pin;
    _gpio = GPIOx;
    BITMASK_SET(RCC->APB2ENR, RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC);
    F103_GPIO_pinMode_input (_gpio, _pin, GPIO_Mode_IPU);
    //_flags = 0;                       // led Arduino initialize _flags to 0
}

void MyButton::update() {
    uint64_t interval = millis() - _changeTime;
    bool state;
  // Read pin state to IS_PRESS
    if (interval > DEBOUNCE_TIME) {
        state = ((_gpio->IDR)&(1<<_pin));
        if ( state == LOW) {
            _flags |= IS_PRESS;
        }
        else {
            _flags &= ~IS_PRESS;
        }
    }

  // Detect changes
    switch (_flags & (WAS_PRESS | IS_PRESS)) {
        case WAS_PRESS:                   // onRelease event
          _flags = ON_RELEASE;
          _changeTime = millis();
          break;

        case IS_PRESS:                    // onPress event
          _flags = WAS_PRESS | IS_PRESS | ON_PRESS;
          _changeTime = millis();
          interval = 0;
          break;
    }

    // Checking Hold state
    if ((_flags & IS_PRESS)             // isPress state
     && (!(_flags & IS_HOLD))           // IS_HOLD not set
     && (interval >= holdTime))         // long Press interval
    {
      _flags |= IS_HOLD | ON_HOLD;      // set Hold flags
    }
}


bool MyButton::isPress() {
    update();
    return (_flags & IS_PRESS);
}


bool MyButton::isHold() {
    update();
    return (_flags & IS_HOLD);
}


bool MyButton::onEvent(uint8_t event) {
    update();
    bool result = _flags & event;
    _flags &= ~event;                    // Reset event flag after checking
    return result;
}


bool MyButton::onPress() {
    return onEvent(ON_PRESS);
}


bool MyButton::onHold() {
    return onEvent(ON_HOLD);
}


bool MyButton::onRelease() {
    return onEvent(ON_RELEASE);
}


bool MyButton::isRelease() {
    update();
    return !(_flags & IS_PRESS);
}


void MyButton::setHoldTime(uint64_t holdTime_ms) {
  holdTime = holdTime_ms;
}