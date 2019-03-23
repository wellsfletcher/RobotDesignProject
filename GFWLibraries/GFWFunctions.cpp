//
//  GFWFunctions.cpp
//  RobotEnvironment
//
//  Created by Fletcher Wells on 2/6/19.
//  Copyright © 2019 Fletcher Wells. All rights reserved.
//


#ifndef GFWFunctions__hpp
#define GFWFunctions__hpp
#include "GFWFunctions.hpp"
#endif



// I'll put these into a color class and move them over to Classes.hpp in the future
// // otherwise, this library should include functions that simulate FEH functions...

int ConvertColor (float rf, float gf, float bf) {
    int r = rf*255, g = gf*255, b = bf*255;
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}
int ConvertColor (RGB rgbColor) {
    int r = rgbColor.r*255, g = rgbColor.g*255, b = rgbColor.b*255;
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}
RGB ConvertColor (int hexValue) {
    RGB rgbColor;
    rgbColor.r = ((hexValue >> 16) & 0xFF) / 255.0;  // Extract the RR byte
    rgbColor.g = ((hexValue >> 8) & 0xFF) / 255.0;   // Extract the GG byte
    rgbColor.b = ((hexValue) & 0xFF) / 255.0;        // Extract the BB byte
    return rgbColor;
}

// adds a transparent color/shadow to the inputted color and returns the hex value
int ConvertToShadow (int hexInput, int shadowColor, float shadowAlpha) {
    RGB rgbInput = ConvertColor (hexInput);
    RGB rgbShadow = ConvertColor (shadowColor);
    RGB rgbOutput;
    rgbOutput.r = rgbInput.r + (rgbShadow.r-rgbInput.r) * shadowAlpha;
    rgbOutput.g = rgbInput.g + (rgbShadow.g-rgbInput.g) * shadowAlpha;
    rgbOutput.b = rgbInput.b + (rgbShadow.b-rgbInput.b) * shadowAlpha;
    int hexOutput = ConvertColor (rgbOutput);
    return hexOutput;
}
