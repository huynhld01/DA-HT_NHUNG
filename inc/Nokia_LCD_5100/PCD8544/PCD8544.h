#ifndef _PCD8544_H
#define _PCD8544_H
#include <string.h>
#include <stdlib.h>
#include "Core.h"
#include "wiring_shift.h"
// #include "Adafruit_GFX.h"

#define BLACK 1 ///< Black pixel
#define WHITE 0 ///< White pixel

#define LCDWIDTH 84   ///< LCD is 84 pixels wide
#define LCDHEIGHT 48  ///< 48 pixe


#define PCD8544_POWERDOWN 0x04            ///< Function set, Power down mode
#define PCD8544_ENTRYMODE 0x02            ///< Function set, Entry mode
#define PCD8544_EXTENDEDINSTRUCTION 0x01  ///< Function set, Extended instruction set control

#define PCD8544_DISPLAYBLANK 0x0      ///< Display control, blank
#define PCD8544_DISPLAYNORMAL 0x4     ///< Display control, normal mode
#define PCD8544_DISPLAYALLON 0x1      ///< Display control, all segments on
#define PCD8544_DISPLAYINVERTED 0x5   ///< Display control, inverse mode

#define PCD8544_FUNCTIONSET 0x20      ///< Basic instruction set
#define PCD8544_DISPLAYCONTROL 0x08   ///< Basic instruction set - Set display configuration
#define PCD8544_SETYADDR 0x40         ///< Basic instruction set - Set Y address of RAM, 0 <= Y <= 5
#define PCD8544_SETXADDR 0x80         ///< Basic instruction set - Set X address of RAM, 0 <= X <= 83

#define PCD8544_SETTEMP 0x04          ///< Extended instruction set - Set temperature coefficient
#define PCD8544_SETBIAS 0x10          ///< Extended instruction set - Set bias system
#define PCD8544_SETVOP 0x80           ///< Extended instruction set - Write Vop to register

#define PCD8544_SPI_CLOCK_DIV SPI_CLOCK_DIV4  ///< Default to max SPI clock speed for PCD8544 of 4 mhz (16mhz / 4) for normal Arduinos. This can be modified to change the clock speed if necessary (like for supporting other hardware).

#define PCD8544_FONT3X5 0
#define PCD8544_FONT5X7 1
class PCD8544 {
    public:
        PCD8544();
        PCD8544(GPIO_TypeDef* GPIOx, uint16_t SCLK, uint16_t DIN, uint16_t DC, uint16_t CS, uint16_t RST);


        void begin(uint8_t contrast = 60, uint8_t bias = 0x04);

        void command(uint8_t c);
        void data(uint8_t c);

        void setContrast(uint8_t val);
        void setBias(uint8_t val);
        uint8_t getContrast(void);
        uint8_t getBias(void);
        void clearDisplay(void);
        void display();
        void setReinitInterval(uint8_t val);
        uint8_t getReinitInterval(void);

        void drawPixel(int16_t x, int16_t y, uint16_t color);
        uint8_t getPixel(int8_t x, int8_t y);

        void initDisplay();

        void startWrite(void);
        void writePixel(int16_t x, int16_t y, uint16_t color);
        void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                         uint16_t color);
        void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
        void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
        void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                     uint16_t color);
        void endWrite(void);

        // // CONTROL API
        // // These MAY be overridden by the subclass to provide device-specific
        // // optimized code.  Otherwise 'generic' versions are used.
        void setRotation(uint8_t r);
        // virtual void invertDisplay(bool i);

        // // BASIC DRAW API
        // // These MAY be overridden by the subclass to provide device-specific
        // // optimized code.  Otherwise 'generic' versions are used.
        void
        // It's good to implement those, even if using transaction API
        drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
          drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
          fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
          fillScreen(uint16_t color),
          // Optional and probably not necessary to change
          drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color),
          drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

        // These exist only with Adafruit_GFX (no subclass overrides)
        void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
          drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
                           uint16_t color),
          fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
          fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
                           int16_t delta, uint16_t color),
          drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                       int16_t y2, uint16_t color),
          fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                       int16_t y2, uint16_t color),
          drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
                        int16_t radius, uint16_t color),
          fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
                        int16_t radius, uint16_t color),
          drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                     int16_t h, uint16_t color),
          drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                     int16_t h, uint16_t color, uint16_t bg),
          drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h,
                     uint16_t color),
          drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h,
                     uint16_t color, uint16_t bg),
          drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                      int16_t h, uint16_t color);
        /**********************************************************************/
        /*!
        @brief  Set text cursor location
        @param  x    X coordinate in pixels
        @param  y    Y coordinate in pixels
        */
        /**********************************************************************/
        void setCursor(int16_t x, int16_t y) {
            cursor_x = x;
            cursor_y = y;
        }
        void setFont(uint8_t fontSet) {
            font = fontSet;
        }
        void print(char c);
        void print(char*data);
        int print(int number);
        void invertPixel(int16_t x, int16_t y);
        void writeLineInvert(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
        void drawFastVLineInvert(int16_t x, int16_t y, int16_t h);
        void writeFastVLineInvert(int16_t x, int16_t y, int16_t h);
        void fillRectInvert(int16_t x, int16_t y, int16_t w, int16_t h);
    protected:
        // int16_t WIDTH,      ///< This is the 'raw' display width - never changes
            // HEIGHT;         ///< This is the 'raw' display height - never changes
        int16_t _width,     ///< Display width as modified by current rotation
            _height,        ///< Display height as modified by current rotation
            cursor_x,       ///< x location to start print()ing text
            cursor_y;       ///< y location to start print()ing text
        // uint16_t textcolor, ///< 16-bit background color for print()
            // textbgcolor;    ///< 16-bit text color for print()
        // uint8_t textsize_x, ///< Desired magnification in X-axis of text to print()
            // textsize_y;     ///< Desired magnification in Y-axis of text to print()
        uint8_t rotation;       ///< Display rotation (0 thru 3)
        bool wrap;       ///< If set, 'wrap' text at right edge of display


    private:
        GPIO_TypeDef* _gpio;
        uint16_t _din;                  ///< DIN pin
        uint16_t _sclk;                 ///< SCLK pin
        uint16_t _dc;                   ///< DC pin
        uint16_t _rst;                  ///< RST pin
        uint16_t _cs;                   ///< CS pin
        uint8_t _contrast;            ///< Contrast level, Vop
        uint8_t _bias;                ///< Bias value
        uint8_t _reinit_interval;     ///< Reinitialize the display after this many calls to display()
        uint8_t _display_count;       ///< Count for reinit interval
        // volatile PortReg *mosiport;   ///< MOSI port
        // volatile PortReg *clkport;    ///< CLK port
        // PortMask mosipinmask;         ///< MOSI port mask
        // PortMask clkpinmask;          ///< CLK port mask
        uint8_t font;
        void spiWrite(uint8_t c);
        bool isHardwareSPI();

};

class brick {
    public:
        brick(uint8_t x, uint8_t y);
        void draw();
    private:
        uint8_t x,y;
};

#endif
