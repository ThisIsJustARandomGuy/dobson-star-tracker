#pragma once
/*
 * 
 */


// This Macro converts a character to an integer
#define char_to_int(x) (x - '0')
// This Macro converts two characters to an integer. Example: ctoi10('2', '3') => 23
#define multi_char_to_int(x, y) ((x - '0') * 10 + (y - '0'))