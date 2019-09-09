#ifndef GFWLCD_hpp
#define GFWLCD_hpp
#include <stdio.h>
#endif /* GFWLCD_hpp */


#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cstdlib>                      // standard definitions
#include <iostream>                     // C++ I/O
#include <cstdio>                       // C I/O (for sprintf)
#include <cmath>                        // standard definitions

using namespace std;                    // make std accessible

#define ZOOM_SCALE 1
#define THICC_LINES 0
 
class GFWLCD {
public:
    struct RGB {
        float r;
        float g;
        float b;
    };
    typedef struct RGB RGB;
    GFWLCD () {
        WIDTH = 320;
        HEIGHT = 240;
        
        // set default font color to white
        fontColor.r = 1.0;
        fontColor.g = 1.0;
        fontColor.b = 1.0;
        
        // set default background color to black
        backgroundColor.r = 0.0;
        backgroundColor.g = 0.0;
        backgroundColor.b = 0.0;
        
        currentLine = 0;
        currentChar = 0;
        maxLines = 14;
        maxCols = 26;
        
        charWidth = 12;
        charHeight = 17;
        
        touchState = false;
        touchX = 0;
        touchY = 0;
    }
    
    bool touchState;
    float touchX, touchY;
    
    bool Touch (float *x, float *y) {
        *x = touchX;
        *y = touchY;
        return touchState;
    }
    
    void Write (const char *str) {
        int futureCurrentChar = currentChar;
        for (int i = 0; str[i] != '\0'; i++) {
            if (currentChar >= maxCols) {
                currentChar = 0;
                futureCurrentChar = 0;
                currentLine++;
                if (currentLine >= maxLines) {
                    currentLine = 0;
                }
            }
            
            // check if word will get cut off and fix it if necessary
            if (currentChar >= futureCurrentChar) {
                int counter = i;
                futureCurrentChar = currentChar;
                bool stop = false;
                bool nullReached = false;
                
                while (stop == false) {
                    counter++;
                    futureCurrentChar++;
                    if (futureCurrentChar > maxCols) {
                        stop = true;
                    } else if (str[counter] == '\0' ) {
                        nullReached = true;
                        stop = true;
                    } else if (str[counter] == ' ') {
                        stop = true;
                    }
                }
                
                // if the word would get cut off, make it so that it won't be that way
                if (nullReached == false && futureCurrentChar > maxCols && currentChar != 0) {
                    // cout << "boi (" << str[i] << "); ";
                    currentChar = 0;
                    futureCurrentChar = 0;
                    currentLine++;
                    if (currentLine >= maxLines) {
                        currentLine = 0;
                    }
                }
            }
            
            if (currentChar == 0 && (int)(str[i]) == ' ') {
            } else {
                WriteCharAt (str[i], currentChar * charWidth, currentLine * charHeight);
                currentChar++;
            }
            
        }
    }
    void Write ( int i ) {
        char str[32];
        sprintf (str, "%i", i);
        Write (str);
    }
    void Write ( float f ) {
        char str[32];
        sprintf (str, "%.3f", f);
        Write (str);
    }
    void Write ( double d ) {
        char str[32];
        sprintf (str, "%f", d);
        Write (str);
    }
    void Write ( bool b ) {
        char str[32];
        sprintf(str, "%s", b ? "true" : "false");
        Write (str);
    }
    void Write ( char c ) {
        char str[32];
        sprintf (str, "%c", c);
        Write (str);
    }
    
    void WriteLine (const char *str) {
        Write (str);
        currentChar = maxCols + 1;
    }
    void WriteLine ( int i ) {
        char str[32];
        sprintf (str, "%i", i);
        WriteLine (str);
    }
    void WriteLine ( float f ) {
        char str[32];
        sprintf (str, "%.3f", f);
        WriteLine (str);
    }
    void WriteLine ( double d ) {
        char str[32];
        sprintf (str, "%f", d);
        WriteLine (str);
    }
    void WriteLine ( bool b ) {
        char str[32];
        sprintf(str, "%s", b ? "true" : "false");
        WriteLine (str);
    }
    void WriteLine ( char c ) {
        char str[32];
        sprintf (str, "%c", c);
        WriteLine (str);
    }
    
    void WriteRC (const char *str, int x, int y) { // may not function exactly the same as the FEH version ... I'd have to check
        WriteAt (str, x * charWidth, y * charHeight);
    }
    
    void WriteAt (const char *str, int x, int y) {
        for (int i = 0; str[i] != '\0'; i++) {
            WriteCharAt (str[i], x + i * charWidth, y);
        }
    }
    void WriteAt ( int i, int x, int y ) {
        char str[32];
        sprintf (str, "%i", i);
        WriteAt (str, x, y);
    }
    void WriteAt ( float f, int x, int y ) {
        char str[32];
        sprintf (str, "%.3f", f);
        WriteAt (str, x, y);
    }
    void WriteAt ( double d, int x, int y ) {
        char str[32];
        sprintf (str, "%f", d);
        WriteAt (str, x, y);
    }
    void WriteAt ( bool b, int x, int y ) {
        char str[32];
        sprintf(str, "%s", b ? "true" : "false");
        WriteAt (str, x, y);
    }
    void WriteAt ( char c, int x, int y ) {
        char str[32];
        sprintf (str, "%c", c);
        WriteAt (str, x, y);
    }
    
    void Clear (int color) {
        currentLine = 0; // reset the current line that the Write function would write to
        SetBackgroundColor (color);
        glClearColor (backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f );
        glClear (GL_COLOR_BUFFER_BIT);
        glFlush();
    }
    void Clear () {
        currentLine = 0; // reset the current line that the Write function would write to
        glClearColor (backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f );
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glFlush();
    }
    
    void SetFontColor (int color) {
        fontColor = ConvertColor (color);
    }
    void SetBackgroundColor (int color) {
        backgroundColor = ConvertColor (color);
    }
    void SetFontColor (float red, float green, float blue) { // non-proteus method
        fontColor.r = red;
        fontColor.g = green;
        fontColor.b = blue;
    }
    void SetBackgroundColor (float red, float green, float blue) { // non-proteus method
        backgroundColor.r = red;
        backgroundColor.g = green;
        backgroundColor.b = blue;
    }
    
    // note that this is a non-FEH function
    void DrawPolygon (int x[], int y[], int length) {
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        glBegin (GL_LINE_LOOP); // edited
        if (THICC_LINES) {
            glLineWidth (1 * ZOOM_SCALE);
        }
        for (int k = 0; k < length; k++) {
            glVertex2f (   XToLCD (x [k]),  YToLCD (y [k])   );
        }
        glEnd ();
    }
    void FillPolygon (int x[], int y[], int length) {
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        glBegin (GL_TRIANGLE_FAN); // edited
        for (int k = 0; k < length; k++) {
            glVertex2f (   XToLCD (x [k]),  YToLCD (y [k])   );
        }
        glEnd ();
    }
    
    
    void DrawCircle (int x, int y, int r0) {
        int numSegments = 16 + r0;
        float cx = XToLCD (x);
        float cy = YToLCD (y);
        float r = XToLCD (r0);
        
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        if (THICC_LINES) {
            glLineWidth (1 * ZOOM_SCALE);
        }
        // GL_LINE_LOOP
        glLineWidth (1 * ZOOM_SCALE);
        for (int ii = 0; ii < numSegments; ii++)   {
            float theta = 2.0f * 3.1415926f * float(ii) / float(numSegments);//get the current angle
            float X = r * cosf(theta); //calculate the x component
            float Y = r * sinf(theta); //calculate the y component
            glVertex2f(X + cx, Y + cy); //output vertex
        }
        glEnd();
    }
    void FillCircle (int x, int y, int r0) {
        int numSegments = 16 + r0;
        float cx = XToLCD (x);
        float cy = YToLCD (y);
        float r = XToLCD (r0);
        
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        glBegin (GL_TRIANGLE_FAN); // GL_LINE_LOOP
        for (int ii = 0; ii < numSegments; ii++)   {
            float theta = 2.0f * 3.1415926f * float(ii) / float(numSegments);//get the current angle
            float X = r * cosf(theta); //calculate the x component
            float Y = r * sinf(theta); //calculate the y component
            glVertex2f(X + cx, Y + cy); //output vertex
        }
        glEnd();
    }
    
    void DrawLine (int x1, int y1, int x2, int y2) {
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        if (THICC_LINES) {
            glLineWidth (1 * ZOOM_SCALE);
        }
        glBegin (GL_LINES);
        glVertex2f (XToLCD (x1), YToLCD(y1));
        glVertex2f (XToLCD (x2), YToLCD (y2));
        glEnd ();
    }
    void DrawRectangle (int x, int y, int width, int height) {
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        if (THICC_LINES) {
            glLineWidth (1 * ZOOM_SCALE);
        }
        float x1 = XToLCD(x);
        float y1 = YToLCD(y);
        float x2 = XToLCD(x + width);
        float y2 = YToLCD(y + height);
        glBegin (GL_LINE_LOOP); // edited
        glVertex2f (x1, y1);
        glVertex2f (x2, y1);
        glVertex2f (x2, y2);
        glVertex2f (x1, y2);
        glEnd ();
    }
    void FillRectangle (int x, int y, int width, int height) {
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        float x1 = XToLCD(x);
        float y1 = YToLCD(y);
        float x2 = XToLCD(x + width);
        float y2 = YToLCD(y + height);
        glRectf (x1, y1, x2, y2);
    }
    
    RGB ConvertColor (int hexValue) {
        RGB rgbColor;
        rgbColor.r = ((hexValue >> 16) & 0xFF) / 255.0;  // Extract the RR byte
        rgbColor.g = ((hexValue >> 8) & 0xFF) / 255.0;   // Extract the GG byte
        rgbColor.b = ((hexValue) & 0xFF) / 255.0;        // Extract the BB byte
        return rgbColor;
    }
private:
    float WIDTH; // width of the screen
    float HEIGHT; // height of the screen
    
    RGB fontColor;
    RGB backgroundColor;
    
    static unsigned char fontData[];
    
    int currentLine;
    int currentChar;
    int maxLines;
    int maxCols;
    
    int charWidth;
    int charHeight;
    
    float XToLCD (float x) {
        return x;
    }
    float YToLCD (float y) {
        return HEIGHT-y;
    }
    
    int GetStringLength (const char *str) {
        int length = 0;
        for (length = 0; str[length] != '\0'; length++);
        return length+1;
    }
    void WriteCharAt (char ch, int x, int y) {
        // draw background rectangle
        glColor3f (backgroundColor.r, backgroundColor.g, backgroundColor.b);
        float x1 = XToLCD (x);
        float y1 = YToLCD (y);
        float x2 = XToLCD (x + charWidth);
        float y2 = YToLCD (y + 17);
        
        y1 = YToLCD (y + 10);
        glColor3f (fontColor.r, fontColor.g, fontColor.b);
        glRasterPos3f (x1, y1, 0.0);
        glutBitmapCharacter (GLUT_BITMAP_9_BY_15, ch);
    }

};

extern GFWLCD LCD;

/* A world of color for the LCD! */

#define BLACK                    0x000000
#define NAVY                    0x000080
#define DARKBLUE                0x00008B
#define MEDIUMBLUE                0x0000CD
#define BLUE                    0x0000FF
#define DARKGREEN                0x006400
#define GREEN                    0x008000
#define TEAL                    0x008080
#define DARKCYAN                0x008B8B
#define DEEPSKYBLUE                0x00BFFF
#define DARKTURQUOISE            0x00CED1
#define MEDIUMSPRINGGREEN        0x00FA9A
#define LIME                    0x00FF00
#define SPRINGGREEN                0x00FF7F
#define AQUA                    0x00FFFF
#define CYAN                    0x00FFFF
#define MIDNIGHTBLUE            0x191970
#define DODGERBLUE                0x1E90FF
#define LIGHTSEAGREEN            0x20B2AA
#define FORESTGREEN                0x228B22
#define SEAGREEN                0x2E8B57
#define DARKSLATEGRAY            0x2F4F4F
#define LIMEGREEN                0x32CD32
#define MEDIUMSEAGREEN            0x3CB371
#define TURQUOISE                0x40E0D0
#define ROYALBLUE                0x4169E1
#define STEELBLUE                0x4682B4
#define DARKSLATEBLUE            0x483D8B
#define MEDIUMTURQUOISE            0x48D1CC
#define INDIGO                     0x4B0082
#define DARKOLIVEGREEN            0x556B2F
#define CADETBLUE                0x5F9EA0
#define CORNFLOWERBLUE            0x6495ED
#define GRAY                    0x666666
#define MEDIUMAQUAMARINE        0x66CDAA
#define DIMGRAY                    0x696969
#define SLATEBLUE                0x6A5ACD
#define OLIVEDRAB                0x6B8E23
#define SLATEGRAY                0x708090
#define LIGHTSLATEGRAY            0x778899
#define MEDIUMSLATEBLUE            0x7B68EE
#define LAWNGREEN                0x7CFC00
#define CHARTREUSE                0x7FFF00
#define AQUAMARINE                0x7FFFD4
#define MAROON                    0x800000
#define PURPLE                    0x800080
#define OLIVE                    0x808000
#define SKYBLUE                    0x87CEEB
#define LIGHTSKYBLUE            0x87CEFA
#define BLUEVIOLET                0x8A2BE2
#define DARKRED                    0x8B0000
#define DARKMAGENTA                0x8B008B
#define SADDLEBROWN                0x8B4513
#define DARKSEAGREEN            0x8FBC8F
#define LIGHTGREEN                0x90EE90
#define MEDIUMPURPLE            0x9370DB
#define DARKVIOLET                0x9400D3
#define PALEGREEN                0x98FB98
#define DARKORCHID                0x9932CC
#define YELLOWGREEN                0x9ACD32
#define SIENNA                    0xA0522D
#define BROWN                    0xA52A2A
#define DARKGRAY                0xA9A9A9
#define LIGHTBLUE                0xADD8E6
#define GREENYELLOW                0xADFF2F
#define PALETURQUOISE            0xAFEEEE
#define LIGHTSTEELBLUE            0xB0C4DE
#define POWDERBLUE                0xB0E0E6
#define FIREBRICK                0xB22222
#define DARKGOLDENROD            0xB8860B
#define MEDIUMORCHID            0xBA55D3
#define SCARLET                    0xBB0000
#define ROSYBROWN                0xBC8F8F
#define DARKKHAKI                0xBDB76B
#define SILVER                    0xC0C0C0
#define MEDIUMVIOLETRED            0xC71585
#define INDIANRED                 0xCD5C5C
#define PERU                    0xCD853F
#define CHOCOLATE                0xD2691E
#define TAN                        0xD2B48C
#define LIGHTGRAY                0xD3D3D3
#define THISTLE                    0xD8BFD8
#define ORCHID                    0xDA70D6
#define GOLDENROD                0xDAA520
#define PALEVIOLETRED            0xDB7093
#define CRIMSON                    0xDC143C
#define GAINSBORO                0xDCDCDC
#define PLUM                    0xDDA0DD
#define BURLYWOOD                0xDEB887
#define LIGHTCYAN                0xE0FFFF
#define LAVENDER                0xE6E6FA
#define DARKSALMON                0xE9967A
#define VIOLET                    0xEE82EE
#define PALEGOLDENROD            0xEEE8AA
#define LIGHTCORAL                0xF08080
#define KHAKI                    0xF0E68C
#define ALICEBLUE                0xF0F8FF
#define HONEYDEW                0xF0FFF0
#define AZURE                    0xF0FFFF
#define SANDYBROWN                0xF4A460
#define WHEAT                    0xF5DEB3
#define BEIGE                    0xF5F5DC
#define WHITESMOKE                0xF5F5F5
#define MINTCREAM                0xF5FFFA
#define GHOSTWHITE                0xF8F8FF
#define SALMON                    0xFA8072
#define ANTIQUEWHITE            0xFAEBD7
#define LINEN                    0xFAF0E6
#define LIGHTGOLDENRODYELLOW    0xFAFAD2
#define OLDLACE                    0xFDF5E6
#define RED                        0xFF0000
#define FUCHSIA                    0xFF00FF
#define MAGENTA                    0xFF00FF
#define DEEPPINK                0xFF1493
#define ORANGERED                0xFF4500
#define TOMATO                    0xFF6347
#define HOTPINK                    0xFF69B4
#define CORAL                    0xFF7F50
#define DARKORANGE                0xFF8C00
#define LIGHTSALMON                0xFFA07A
#define ORANGE                    0xFFA500
#define LIGHTPINK                0xFFB6C1
#define PINK                    0xFFC0CB
#define GOLD                    0xFFD700
#define PEACHPUFF                0xFFDAB9
#define NAVAJOWHITE                0xFFDEAD
#define MOCCASIN                0xFFE4B5
#define BISQUE                    0xFFE4C4
#define MISTYROSE                0xFFE4E1
#define BLANCHEDALMOND            0xFFEBCD
#define PAPAYAWHIP                0xFFEFD5
#define LAVENDERBLUSH            0xFFF0F5
#define SEASHELL                0xFFF5EE
#define CORNSILK                0xFFF8DC
#define LEMONCHIFFON            0xFFFACD
#define FLORALWHITE                0xFFFAF0
#define SNOW                    0xFFFAFA
#define YELLOW                    0xFFFF00
#define LIGHTYELLOW                0xFFFFE0
#define IVORY                    0xFFFFF0
#define WHITE                    0xFFFFFF

