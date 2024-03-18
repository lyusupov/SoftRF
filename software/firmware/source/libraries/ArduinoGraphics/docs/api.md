# Arduino Graphics Library

## Methods

### `begin()`

#### Description


Initializes the graphics device.

#### Syntax

```
YourScreen.begin()
```


#### Parameters

None

#### Returns

1 for success, 0 on failure.

#### Example

```
if (!YourScreen.begin() {
  Serial.println(“Failed to initialize the display!”);
  while (1);
}
```



### `end()`

#### Description

Stops the graphics device.

#### Syntax

```
YourScreen.end()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
YourScreen.end();
```



### `width()`

#### Description


Returns the pixel width of the graphics device.

#### Syntax

```
YourScreen.width()

```


#### Parameters


None

#### Returns

Returns the pixel width of the graphics device.

#### Example

```
int w = YourScreen.width();
```


### `height()`

#### Description


Returns the pixel height of the graphics device.

#### Syntax

```
YourScreen.height()

```


#### Parameters


None

#### Returns

Returns the pixel height of the graphics device.

#### Example

```
int h = YourScreen.height();
```


### `beginDraw()`

#### Description


Begins a drawing operation.

#### Syntax

```
YourScreen.beginDraw()

```


#### Parameters


None


#### Returns

Nothing

#### Example


YourScreen.beginDraw();
YourScreen.set(0, 0, 255, 0, 0);
YourScreen.endDraw();



### `endDraw()`

#### Description


Ends a drawing operation, any drawing operations after beginDraw() is called will be displayed to the screen.

#### Syntax

```
YourScreen.endDraw()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.set(0, 0, 255, 0, 0);
YourScreen.endDraw();
```


### `background()`

#### Description


Set the background color of drawing operations. Used when calling clear() or drawing text.

#### Syntax

```
YourScreen.background(r, g, b)
YourScreen.background(color)

```


#### Parameters


- r: red color value (0 - 255)
- g: green color value (0 - 255)
- b: blue color value (0 - 255)
- color: 24-bit RGB color, 0xrrggbb

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.background(255, 0, 0);
YourScreen.clear();
YourScreen.endDraw();
```


### `clear()`

#### Description


Set clear the screen contents, uses the background colour set in background().

#### Syntax

```
YourScreen.clear()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.background(255, 0, 0);
YourScreen.clear();
YourScreen.endDraw();
```



### `fill()`

#### Description


Set the fill color of drawing operations.

#### Syntax

```
YourScreen.fill(r, g, b)
YourScreen.fill(color)

```


#### Parameters


- r: red color value (0 - 255)
- g: green color value (0 - 255)
- b: blue color value (0 - 255)
- color: 24-bit RGB color, 0xrrggbb

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.noStroke();
YourScreen.fill(255, 255, 0);
YourScreen.rect(0, 0, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```


### `noFill()`

#### Description


Clears the fill color of drawing operations.

#### Syntax

```
YourScreen.noFill()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(255, 0, 255);
YourScreen.noFill();
YourScreen.rect(0, 0, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```


### `stroke()`

#### Description


Set the stroke color of drawing operations.

#### Syntax

```
YourScreen.stroke(r, g, b)
YourScreen.stroke(color)

```


#### Parameters


- r: red color value (0 - 255)
- g: green color value (0 - 255)
- b: blue color value (0 - 255)
- color: 24-bit RGB color, 0xrrggbb

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(255, 0, 255);
YourScreen.noFill();
YourScreen.rect(0, 0, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```



### `noStroke()`

#### Description


Clears the stroke color of drawing operations.

#### Syntax

```
YourScreen.noStroke()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.noStroke();
YourScreen.fill(255, 255, 0);
YourScreen.rect(0, 0, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```


### `line()`

#### Description


Stroke a line, uses the stroke color set in stroke().

#### Syntax

```
YourScreen.line(x1, y1, x2, y2)

```


#### Parameters


- x1: x position of the starting point of the line
- y1: y position of the starting point of the line
- x2: x position of the end point of the line
- y2: y position of the end point of the line

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(0, 0, 255);
YourScreen.line(0, 0, YourScreen.width() - 1, YourScreen.height() - 1);
YourScreen.endDraw();
```


### `point()`

#### Description


Stroke a point, uses the stroke color set in stroke().

#### Syntax

```
YourScreen.point(x, y)

```


#### Parameters


x: x position of the point
y: y position of the point

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(0, 255, 0);
YourScreen.point(1, 1);
YourScreen.endDraw();
```


### `rect()`

#### Description


Stroke and fill a rectangle, uses the stroke color set in stroke() and the fill color set in fill().

#### Syntax

```
YourScreen.rect(x, y, width, height)

```


#### Parameters


- x: x position of the rectangle
- y: y position of the rectangle
- width: width of the rectangle
- height: height of the rectangle

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.noStroke();
YourScreen.fill(255, 255, 0);
YourScreen.rect(0, 0, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```


### `circle()`

#### Description


Stroke and fill a circle, uses the stroke color set in stroke() and the fill color set in fill().

#### Syntax

```
YourScreen.circle(x, y, diameter)

```


#### Parameters


- x: x center position of the circle
- y: y center position of the circle
- diameter: diameter of the circle

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.noStroke();
YourScreen.fill(255, 255, 0);
YourScreen.circle(YourScreen.width()/2, YourScreen.height()/2, YourScreen.height());
YourScreen.endDraw();
```


### `ellipse()`

#### Description


Stroke and fill an ellipse, uses the stroke color set in stroke() and the fill color set in fill().

#### Syntax

```
YourScreen.ellipse(x, y, width, height)

```


#### Parameters


- x: x center position of the ellipse
- y: y center position of the ellipse
- width: width of the ellipse
- height: height of the ellipse

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.noStroke();
YourScreen.fill(255, 255, 0);
YourScreen.ellipse(YourScreen.width()/2, YourScreen.height()/2, YourScreen.width(), YourScreen.height());
YourScreen.endDraw();
```


### `text()`

#### Description


Draw some text, uses the stroke color set in stroke() and the background color set in background().

#### Syntax

```
YourScreen.text(string)
YourScreen.text(string, x, y)

```


#### Parameters


- string: string to draw
- x: x position for the start of the text
- y: y position for the start of the text

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(255, 255, 255);
YourScreen.text("abc", 0, 1);
YourScreen.endDraw();
```


### `textFont()`

#### Description


Sets the font uses for text. The library current has the Font_4x6 and Font_5x7 built in.

#### Syntax

```
YourScreen.textFont(font)

```


#### Parameters


font: font to set


#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.clear();
YourScreen.stroke(255, 255, 255);
YourScreen.textFont(Font_5x7);
YourScreen.text("abc", 0, 1);
YourScreen.endDraw();
```


### `textFontWidth()`

#### Description


Returns the width, in pixels, of the current font.

#### Syntax

```
YourScreen.textFontWidth()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
int w = YourScreen.textFontWidth();
```


### `textFontHeight()`

#### Description


Returns the height, in pixels, of the current font.

#### Syntax

```
YourScreen.textFontHeight()

```


#### Parameters


None


#### Returns

Nothing

#### Example

```
int h = YourScreen.textFontHeight();
```


### `set()`

#### Description


Set a pixel’s color value.

#### Syntax

```
YourScreen.set(x, y, r, g, b)
YourScreen.set(x, y, color)

```


#### Parameters


x: x position of the pixel
y: y position of the pixel
r: red color value (0 - 255)
g: green color value (0 - 255)
b: blue color value (0 - 255)
color: 24-bit RGB color, 0xrrggbb

#### Returns

Nothing

#### Example

```
YourScreen.beginDraw();
YourScreen.point(1, 1, 0, 255, 0);
YourScreen.endDraw();
```


### `beginText()`

#### Description


Start the process of displaying and optionally scrolling text. The Print interface can be used to set the text.

#### Syntax

```
YourScreen.beginText()
YourScreen.beginText(x, y, r, g, b)
YourScreen.beginText(x, y, color)

```


#### Parameters


x: x position of the text
y: y position of the text
r: red color value (0 - 255)
g: green color value (0 - 255)
b: blue color value (0 - 255)
color: 24-bit RGB color, 0xrrggbb

#### Returns

Nothing

#### Example

```
YourScreen.beginText(0, 0, 127, 0, 0);
YourScreen.print("Hi");
YourScreen.endText();
```


### `endText()`

#### Description


End the process of displaying and optionally scrolling text.

#### Syntax

```
YourScreen.endText()
YourScreen.endText(scrollDirection)

```


#### Parameters


scrollDirection: (optional) the direction to scroll, defaults to NO_SCROLL if not provided. Valid options are NO_SCROLL, SCROLL_LEFT, SCROLL_RIGHT, SCROLL_UP, SCROLL_DOWN

#### Returns

Nothing

#### Example

```
YourScreen.beginText(0, 0, 127, 0, 0);
YourScreen.print("Hi");
YourScreen.endText();
```


### `textScrollSpeed()`

#### Description


Sets the text scrolling speed, the speed controls the delay in milliseconds between scrolling each pixel.

#### Syntax

```
YourScreen.textScrollSpeed(speed)

```


#### Parameters


speed: scroll speed


#### Returns

Nothing

#### Example

```
YourScreen.beginText(0, 0, 127, 0, 0);
YourScreen.textScrollSpeed(500);
YourScreen.print("Hello There!");
YourScreen.endText(true);
```

