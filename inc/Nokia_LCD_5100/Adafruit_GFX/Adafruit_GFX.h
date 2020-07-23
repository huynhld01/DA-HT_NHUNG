#ifndef _ADAFRUIT_GFX_H
#define _ADAFRUIT_GFX_H
#include <stdio.h>
#include "stdlib.h"

#include "font.h"

#define BLACK 1 ///< Black pixel
#define WHITE 0 ///< White pixel

class Adafruit_GFX {

public:
  Adafruit_GFX(int16_t w, int16_t h); // Constructor

  // This MUST be defined by the subclass:
  void drawPixel(int16_t x, int16_t y, uint16_t color);

  // TRANSACTION API / CORE DRAW API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  void startWrite(void);
  void writePixel(int16_t x, int16_t y, uint16_t color);
  void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                     uint16_t color);
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                 uint16_t color);
  void endWrite(void);

  // CONTROL API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  void setRotation(uint8_t r);
  void invertDisplay(bool i);

  // BASIC DRAW API
  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
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
                 uint16_t color, uint16_t bg);

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

  /**********************************************************************/
  /*!
  @brief  Set whether text that is too long for the screen width should
          automatically wrap around to the next line (else clip right).
  @param  w  true for wrapping, false for clipping
  */
  /**********************************************************************/
  void setTextWrap(bool w) { wrap = w; }

  /************************************************************************/
  /*!
    @brief      Get width of the display, accounting for current rotation
    @returns    Width in pixels
  */
  /************************************************************************/
  int16_t width(void) const { return _width; };

  /************************************************************************/
  /*!
    @brief      Get height of the display, accounting for current rotation
    @returns    Height in pixels
  */
  /************************************************************************/
  int16_t height(void) const { return _height; }

  /************************************************************************/
  /*!
    @brief      Get rotation setting for display
    @returns    0 thru 3 corresponding to 4 cardinal rotations
  */
  /************************************************************************/
  uint8_t getRotation(void) const { return rotation; }

  // get current cursor position (get rotation safe maximum values,
  // using: width() for x, height() for y)
  /************************************************************************/
  /*!
    @brief  Get text cursor X location
    @returns    X coordinate in pixels
  */
  /************************************************************************/
  int16_t getCursorX(void) const { return cursor_x; }

  /************************************************************************/
  /*!
    @brief      Get text cursor Y location
    @returns    Y coordinate in pixels
  */
  /************************************************************************/
  int16_t getCursorY(void) const { return cursor_y; };

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
  int16_t WIDTH,      ///< This is the 'raw' display width - never changes
      HEIGHT;         ///< This is the 'raw' display height - never changes
  int16_t _width,     ///< Display width as modified by current rotation
      _height,        ///< Display height as modified by current rotation
      cursor_x,       ///< x location to start print()ing text
      cursor_y;       ///< y location to start print()ing text
  uint8_t rotation ;       ///< Display rotation (0 thru 3)
  bool wrap;       ///< If set, 'wrap' text at right edge of display
  uint8_t font;
};
extern uint8_t pcd8544_buffer[504];

#endif // _ADAFRUIT_GFX_H
