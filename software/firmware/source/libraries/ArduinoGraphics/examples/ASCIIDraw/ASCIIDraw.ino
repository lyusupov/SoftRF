/*
  ASCIIDraw

  Use the ArduinoGraphics library to draw ASCII art on the Serial Monitor.

  This is intended primarily to allow testing of the library.
  See the Arduino_MKRRGB library for a more useful demonstration of the ArduinoGraphics library.

  The circuit:
  - Arduino board

  This example code is in the public domain.
*/

#include <ArduinoGraphics.h>

const byte canvasWidth = 61;
const byte canvasHeight = 27;

class ASCIIDrawClass : public ArduinoGraphics {
  public:
    // can be used with an object of any class that inherits from the Print class
    ASCIIDrawClass(Print &printObject = (Print &)Serial) :
      ArduinoGraphics(canvasWidth, canvasHeight),
      _printObject(&printObject) {}

    // this function is called by the ArduinoGraphics library's functions
    virtual void set(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
      // the r parameter is (mis)used to set the character to draw with
      _canvasBuffer[x][y] = r;
      // cast unused parameters to void to fix "unused parameter" warning
      (void)g;
      (void)b;
    }

    // display the drawing
    void endDraw() {
      ArduinoGraphics::endDraw();

      for (byte row = 0; row < canvasHeight; row++) {
        for (byte column = 0; column < canvasWidth; column++) {
          // handle unset parts of buffer
          if (_canvasBuffer[column][row] == 0) {
            _canvasBuffer[column][row] = ' ';
          }
          _printObject->print(_canvasBuffer[column][row]);
        }
        _printObject->println();
      }
    }

  private:
    Print *_printObject;
    char _canvasBuffer[canvasWidth][canvasHeight] = {{0}};
};

ASCIIDrawClass ASCIIDraw;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  ASCIIDraw.beginDraw();

  // configure the character used to fill the background. The second and third parameters are ignored
  ASCIIDraw.background('+', 0, 0);
  ASCIIDraw.clear();

  // add the outer border
  ASCIIDraw.stroke('-', 0, 0);
  ASCIIDraw.fill('*', 0, 0);
  const byte outerBorderThickness = 1;
  ASCIIDraw.rect(outerBorderThickness, outerBorderThickness, canvasWidth - outerBorderThickness * 2, canvasHeight - outerBorderThickness * 2);

  // add the inner border
  ASCIIDraw.stroke('+', 0, 0);
  ASCIIDraw.fill('O', 0, 0);
  const byte borderThickness = outerBorderThickness + 6;
  ASCIIDraw.rect(borderThickness, borderThickness, canvasWidth - borderThickness * 2, canvasHeight - borderThickness * 2);

  // add the text
  ASCIIDraw.background(' ', 0, 0);
  ASCIIDraw.stroke('@', 0, 0);
  const char text[] = "ARDUINO";
  ASCIIDraw.textFont(Font_5x7);
  const byte textWidth = strlen(text) * ASCIIDraw.textFontWidth();
  const byte textHeight = ASCIIDraw.textFontHeight();
  const byte textX = (canvasWidth - textWidth) / 2;
  const byte textY = (canvasHeight - textHeight) / 2;
  ASCIIDraw.text(text, textX, textY);

  // underline the text
  ASCIIDraw.stroke('-', 0, 0);
  ASCIIDraw.line(textX, textY + textHeight - 1, textX + textWidth - 1, textY + textHeight - 1);

  // add some accents to the underline
  ASCIIDraw.stroke('+', 0, 0);
  ASCIIDraw.point(textX + 4, textY + textHeight - 1);
  ASCIIDraw.point(textX + textWidth - 1 - 4, textY + textHeight - 1);

  // print the drawing to the Serial Monitor
  ASCIIDraw.endDraw();
}

void loop() {}
