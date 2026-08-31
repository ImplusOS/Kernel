#pragma once

#include <stdint.h>
#include "kernel/keycodes.h"
#include "Drivers/Module/DriverBinary.h"

static inline char keycode_to_ascii(uint16_t keycode, uint8_t modifiers) {
    int shift = (modifiers & DRIVER_KBD_MOD_SHIFT) != 0;
    int caps = (modifiers & DRIVER_KBD_MOD_CAPS) != 0;
    int alpha_upper = shift ^ caps;
    
    uint8_t base = (uint8_t)(keycode & 0xFFu);
    int extended = (keycode & 0xFF00u) != 0;

    if (extended) {
        switch (keycode) {
            case KEY_KPENTER: return '\n';
            case KEY_KPSLASH: return '/';
            default: return 0;
        }
    }

    switch (base) {
        case KEY_ESC:       return 0x1B;
        case KEY_ENTER:     return '\n';
        case KEY_BACKSPACE: return '\b';
        case KEY_TAB:       return '\t';
        case KEY_SPACE:     return ' ';
        
        case KEY_A: return alpha_upper ? 'A' : 'a';
        case KEY_B: return alpha_upper ? 'B' : 'b';
        case KEY_C: return alpha_upper ? 'C' : 'c';
        case KEY_D: return alpha_upper ? 'D' : 'd';
        case KEY_E: return alpha_upper ? 'E' : 'e';
        case KEY_F: return alpha_upper ? 'F' : 'f';
        case KEY_G: return alpha_upper ? 'G' : 'g';
        case KEY_H: return alpha_upper ? 'H' : 'h';
        case KEY_I: return alpha_upper ? 'I' : 'i';
        case KEY_J: return alpha_upper ? 'J' : 'j';
        case KEY_K: return alpha_upper ? 'K' : 'k';
        case KEY_L: return alpha_upper ? 'L' : 'l';
        case KEY_M: return alpha_upper ? 'M' : 'm';
        case KEY_N: return alpha_upper ? 'N' : 'n';
        case KEY_O: return alpha_upper ? 'O' : 'o';
        case KEY_P: return alpha_upper ? 'P' : 'p';
        case KEY_Q: return alpha_upper ? 'Q' : 'q';
        case KEY_R: return alpha_upper ? 'R' : 'r';
        case KEY_S: return alpha_upper ? 'S' : 's';
        case KEY_T: return alpha_upper ? 'T' : 't';
        case KEY_U: return alpha_upper ? 'U' : 'u';
        case KEY_V: return alpha_upper ? 'V' : 'v';
        case KEY_W: return alpha_upper ? 'W' : 'w';
        case KEY_X: return alpha_upper ? 'X' : 'x';
        case KEY_Y: return alpha_upper ? 'Y' : 'y';
        case KEY_Z: return alpha_upper ? 'Z' : 'z';

        case KEY_KP0: return '0';
        case KEY_KP1: return '1';
        case KEY_KP2: return '2';
        case KEY_KP3: return '3';
        case KEY_KP4: return '4';
        case KEY_KP5: return '5';
        case KEY_KP6: return '6';
        case KEY_KP7: return '7';
        case KEY_KP8: return '8';
        case KEY_KP9: return '9';
        case KEY_KPDOT:   return '.';
        case KEY_KPASTERISK: return '*';
        case KEY_KPMINUS: return '-';
        case KEY_KPPLUS:  return '+';
    }

    if (!shift) {
        switch (base) {
            case KEY_1: return '1';
            case KEY_2: return '2';
            case KEY_3: return '3';
            case KEY_4: return '4';
            case KEY_5: return '5';
            case KEY_6: return '6';
            case KEY_7: return '7';
            case KEY_8: return '8';
            case KEY_9: return '9';
            case KEY_0: return '0';
            case KEY_MINUS: return '-';
            case KEY_EQUAL: return '=';
            case KEY_LEFTBRACE:  return '[';
            case KEY_RIGHTBRACE: return ']';
            case KEY_SEMICOLON:  return ';';
            case KEY_APOSTROPHE: return '\'';
            case KEY_GRAVE:      return '`';
            case KEY_BACKSLASH:  return '\\';
            case KEY_COMMA:      return ',';
            case KEY_DOT:        return '.';
            case KEY_SLASH:      return '/';
            case KEY_RO:         return '\\';
            case KEY_YEN:        return '\\';
            default: return 0;
        }
    } else {
        switch (base) {
            case KEY_1: return '!';
            case KEY_2: return '@';
            case KEY_3: return '#';
            case KEY_4: return '$';
            case KEY_5: return '%';
            case KEY_6: return '^';
            case KEY_7: return '&';
            case KEY_8: return '*';
            case KEY_9: return '(';
            case KEY_0: return ')';
            case KEY_MINUS: return '_';
            case KEY_EQUAL: return '+';
            case KEY_LEFTBRACE:  return '{';
            case KEY_RIGHTBRACE: return '}';
            case KEY_SEMICOLON:  return ':';
            case KEY_APOSTROPHE: return '"';
            case KEY_GRAVE:      return '~';
            case KEY_BACKSLASH:  return '|';
            case KEY_COMMA:      return '<';
            case KEY_DOT:        return '>';
            case KEY_SLASH:      return '?';
            case KEY_RO:         return '_';
            case KEY_YEN:        return '|';
            default: return 0;
        }
    }
}
