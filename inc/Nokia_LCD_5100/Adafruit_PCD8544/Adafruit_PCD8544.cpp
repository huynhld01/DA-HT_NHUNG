#include "Adafruit_PCD8544.h"
#include <string.h>

/*!
  @brief Constructor for software SPI with explicit CS pin
  @param SCLK SCLK pin
  @param DIN  DIN pin
  @param DC   DC pin
  @param CS   CS pin
  @param RST  RST pin
 */
Adafruit_PCD8544::Adafruit_PCD8544(GPIO_TypeDef* GPIOx, uint16_t SCLK, uint16_t DIN, uint16_t DC, uint16_t CS, uint16_t RST):
    Adafruit_GFX(LCDWIDTH, LCDHEIGHT),
    _gpio(GPIOx),
    _sclk(SCLK),
    _din(DIN),
    _dc(DC),
    _cs(CS),
    _rst(RST)
{}
/*!
  @brief The most basic function, get a single pixel
  @param  x x coord
  @param  y y coord
  @return   color of the pixel at x,y
 */
uint8_t Adafruit_PCD8544::getPixel(int8_t x, int8_t y) {
  if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
    return 0;

  return (pcd8544_buffer[x+ (y/8)*LCDWIDTH] >> (y%8)) & 0x1;
}

/*!
  @brief Initialize the display. Set bias and contrast, enter normal mode.
 */
void Adafruit_PCD8544::initDisplay() {

    // toggle RST low to reset
    // if (_rst > 0) {
      // digitalWrite(_rst, LOW);
      // delay(500);
      // digitalWrite(_rst, HIGH);
    // }
    // digitalWrite(_rst, LOW);
    _gpio->ODR &= ~_BV(_rst);
    delay(500);
    // digitalWrite(_rst, HIGH);
    _gpio->ODR |= _BV(_rst);

    setBias(_bias);
    setContrast(_contrast);

    // normal mode
    command(PCD8544_FUNCTIONSET);

    // Set display to Normal
    command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

/*!
  @brief Set up SPI, initialize the display, set the bounding box
  @param contrast Initial contrast value
  @param bias     Initial bias value
 */
void Adafruit_PCD8544::begin(uint8_t contrast, uint8_t bias){
    // Setup software SPI.

    // Set software SPI specific pin outputs.
    // pinMode(_din, OUTPUT);
    F103_GPIO_pinMode_output(_gpio, _din, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);
    // pinMode(_sclk, OUTPUT);
    F103_GPIO_pinMode_output(_gpio, _sclk, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);

        // Set software SPI ports and masks.
    // clkport     = portOutputRegister(digitalPinToPort(_sclk));
    // clkpinmask  = digitalPinToBitMask(_sclk);
    // mosiport    = portOutputRegister(digitalPinToPort(_din));
    // mosipinmask = digitalPinToBitMask(_din);

    // Set common pin outputs.
    // pinMode(_dc, OUTPUT);
    F103_GPIO_pinMode_output(_gpio, _dc, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);
    if (_rst > 0)
        // pinMode(_rst, OUTPUT);
        F103_GPIO_pinMode_output(_gpio, _rst, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);
    if (_cs > 0)
        // pinMode(_cs, OUTPUT);
        F103_GPIO_pinMode_output(_gpio, _cs, GPIO_Mode_Out_PP | GPIO_Speed_2MHz);

    _bias = bias;
    _contrast = contrast;
    _reinit_interval = 0;
    _display_count = 0;
    initDisplay();

    // initial display line
    // set page address
    // set column address
    // write display data

    // Push out pcd8544_buffer to the Display (will show the AFI logo)
    display();
}

/*!
  @brief Write a byte to SPI
  @param d Byte to write
 */
inline void Adafruit_PCD8544::spiWrite(uint8_t d) {
    shiftOut(_gpio, _din, _sclk, MSBFIRST, d ) ;
}

/*!
  @brief  Using hardware or software SPI?
  @return True if hardware SPI, false if not
 */
bool Adafruit_PCD8544::isHardwareSPI() {
  return (_din == -1 && _sclk == -1);
}

/*!
  @brief  Send a command to the LCD
  @param c Command byte
 */
void Adafruit_PCD8544::command(uint8_t c) {
  // digitalWrite(_dc, LOW);
    _gpio->ODR &= ~_BV(_dc);
    if (_cs > 0)
        // digitalWrite(_cs, LOW);
        _gpio->ODR &= ~_BV(_cs);
    spiWrite(c);
    if (_cs > 0)
        // digitalWrite(_cs, HIGH);
        _gpio->ODR |= _BV(_cs);
}

/*!
  @brief  Send data to the LCD
  @param c Data byte
 */
void Adafruit_PCD8544::data(uint8_t c) {
    //digitalWrite(_dc, HIGH);
    _gpio->ODR |= _BV(_dc);
    if (_cs > 0)
        // digitalWrite(_cs, LOW);
        _gpio->ODR &= ~_BV(_cs);
    spiWrite(c);
    if (_cs > 0)
        // digitalWrite(_cs, HIGH);
        _gpio->ODR |= _BV(_cs);
}

/*!
  @brief  Set the contrast level
  @param val Contrast value
 */
void Adafruit_PCD8544::setContrast(uint8_t val) {
  if (val > 0x7f) {
    val = 0x7f;
  }
  _contrast = val;
  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
  command( PCD8544_SETVOP | val);
  command(PCD8544_FUNCTIONSET);

 }

 /*!
   @brief  Set the bias level
   @param val Bias value
  */
void Adafruit_PCD8544::setBias(uint8_t val) {
  if (val > 0x07) {
    val = 0x07;
  }
  _bias = val;
  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
  command(PCD8544_SETBIAS | val);
  command(PCD8544_FUNCTIONSET);
}

/*!
  @brief  Get the bias level
  @return Bias value
 */
uint8_t Adafruit_PCD8544::getBias()
{
  return _bias;
}

/*!
  @brief  Get the contrast level
  @return Contrast value
 */
uint8_t Adafruit_PCD8544::getContrast()
{
  return _contrast;
}

/*!
  @brief  Set the interval for reinitializing the display
  @param val Reinit after this many calls to display()
 */
void Adafruit_PCD8544::setReinitInterval(uint8_t val) {
    _reinit_interval = val;
}

/*!
  @brief  Get the reinit interval
  @return Reinit interval
 */
uint8_t Adafruit_PCD8544::getReinitInterval()
{
  return _reinit_interval;
}

/*!
  @brief Update the display
 */
void Adafruit_PCD8544::display(void) {
  uint8_t col, maxcol, p;

  if(_reinit_interval) {
      _display_count++;
      if(_display_count >= _reinit_interval)
	{
	  _display_count = 0;
	  initDisplay();
#ifdef enablePartialUpdate
	  yUpdateMin = 0;
	  yUpdateMax = LCDHEIGHT-1;
	  xUpdateMin = 0;
	  xUpdateMax = LCDWIDTH-1;
#endif
	}
  }

  for(p = 0; p < 6; p++) {
#ifdef enablePartialUpdate
    // check if this page is part of update
    if ( yUpdateMin >= ((p+1)*8) ) {
      continue;   // nope, skip it!
    }
    if (yUpdateMax < p*8) {
      break;
    }
#endif

    command(PCD8544_SETYADDR | p);


#ifdef enablePartialUpdate
    col = xUpdateMin;
    maxcol = xUpdateMax;
#else
    // start at the beginning of the row
    col = 0;
    maxcol = LCDWIDTH-1;
#endif

    command(PCD8544_SETXADDR | col);

    // digitalWrite(_dc, HIGH);
       _gpio->ODR |= _BV(_dc);
        if (_cs > 0)
            // digitalWrite(_cs, LOW);
            _gpio->ODR &= ~_BV(_cs);
        for(; col <= maxcol; col++) {
            spiWrite(pcd8544_buffer[(LCDWIDTH*p)+col]);
        }
        if (_cs > 0)
            // digitalWrite(_cs, HIGH);
            _gpio->ODR |= _BV(_cs);

  }

  command(PCD8544_SETYADDR );  // no idea why this is necessary but it is to finish the last byte?
#ifdef enablePartialUpdate
  xUpdateMin = LCDWIDTH - 1;
  xUpdateMax = 0;
  yUpdateMin = LCDHEIGHT-1;
  yUpdateMax = 0;
#endif

}

/*!
  @brief Clear the entire display
 */
void Adafruit_PCD8544::clearDisplay(void) {
  memset(pcd8544_buffer, 0, LCDWIDTH*LCDHEIGHT/8);
  cursor_y = cursor_x = 0;
}

/*
// this doesnt touch the buffer, just clears the display RAM - might be handy
void Adafruit_PCD8544::clearDisplay(void) {

  uint8_t p, c;

  for(p = 0; p < 8; p++) {

    st7565_command(CMD_SET_PAGE | p);
    for(c = 0; c < 129; c++) {
      //uart_putw_dec(c);
      //uart_putchar(' ');
      st7565_command(CMD_SET_COLUMN_LOWER | (c & 0xf));
      st7565_command(CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
      st7565_data(0x0);
    }
    }

}

*/
