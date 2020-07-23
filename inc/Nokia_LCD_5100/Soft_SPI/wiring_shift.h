/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_SHIFT_
#define _WIRING_SHIFT_

#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f10x_gpio.h"
/*
 * \brief
 */
#define LSBFIRST 0
#define MSBFIRST 1
extern uint32_t shiftIn(GPIO_TypeDef* GPIOx, uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder ) ;


/*
 * \brief
 */
extern void shiftOut(GPIO_TypeDef* GPIOx, uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint8_t ulVal ) ;


#ifdef __cplusplus
}
#endif

#endif /* _WIRING_SHIFT_ */
