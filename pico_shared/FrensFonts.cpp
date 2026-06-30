#include "FrensFonts.h"
#include "font_8x8.h"

char getcharslicefrom8x8font(char c, int rowInChar)
{
    return font_8x8[(c - FONT_FIRST_ASCII) + (rowInChar)*FONT_N_CHARS];
}