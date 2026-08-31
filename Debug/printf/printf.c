#include "printf.h"
#include "Drivers/Module/DriverManager.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include "Debug/serial/Serial.h"

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#define STB_TRUETYPE_NO_STDIO
#define STBTT_assert(x)

#define STBTT_memcpy  memcpy
#define STBTT_memset  memset
#define STBTT_memmove memmove
#define STBTT_strlen  strlen

#define STBTT_sqrt(x)   sqrt(x)
#define STBTT_pow(x,y)  pow(x,y)
#define STBTT_fmod(x,y) fmod(x,y)
#define STBTT_cos(x)    cos(x)
#define STBTT_acos(x)   acos(x)
#define STBTT_fabs(x)   fabs(x)
#define STBTT_ifloor(x) ((int)floor(x))
#define STBTT_iceil(x)  ((int)ceil(x))

#include "../../../Vendor/Header/stb_truetype.h"

#define DEBUG_FONT_HEIGHT 18

static uint8_t g_char_bitmap[256 * 256];

static struct {
    uint32_t cursor_x;
    uint32_t cursor_y;
    uint32_t char_width;
    uint32_t char_height;
    uint32_t screen_width;
    uint32_t screen_height;
    uint32_t max_cols;
    uint32_t max_rows;
    uint32_t bg_color;
    uint32_t fg_color;
    uint64_t framebuffer_base;
    uint32_t pixels_per_scan_line;
    bool initialized;
    stbtt_fontinfo ttf_font;
    int ttf_valid;
    float ttf_scale;
    int ttf_ascent;
    int ttf_descent;
    int ttf_line_gap;
} g_debugger = {
    .cursor_x = 0,
    .cursor_y = 0,
    .char_width = 8,
    .char_height = 8,
    .screen_width = 0,
    .screen_height = 0,
    .max_cols = 0,
    .max_rows = 0,
    .bg_color = 0x000000,
    .fg_color = 0xFFFFFF,
    .framebuffer_base = 0,
    .pixels_per_scan_line = 0,
    .initialized = false,
    .ttf_valid = 0,
    .ttf_scale = 1.0f,
    .ttf_ascent = 0,
    .ttf_descent = 0,
    .ttf_line_gap = 0,
};

extern bool debugger_display_init(void);

static void debugger_present_if_needed(void)
{
    if (g_debugger.initialized &&
        g_debugger.framebuffer_base == 0 &&
        driver_manager_display_is_ready()) {
        driver_manager_display_present();
    }
}

static uint32_t debugger_draw_char_at(char c, uint32_t x, uint32_t y, uint32_t fg, uint32_t bg)
{
    if (!g_debugger.ttf_valid) {
        return g_debugger.char_width;
    }

    if (c < 0x20) {
        c = '?';
    }

    float scale = g_debugger.ttf_scale;
    int advance, lsb;
    stbtt_GetCodepointHMetrics(&g_debugger.ttf_font, (int)(unsigned char)c, &advance, &lsb);

    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(&g_debugger.ttf_font, (int)(unsigned char)c, scale, scale, &x0, &y0, &x1, &y1);

    int char_w = x1 - x0;
    int char_h = y1 - y0;

    if (char_w > 0 && char_h > 0 && char_w <= 256 && char_h <= 256) {
        memset(g_char_bitmap, 0, (size_t)(char_w * char_h));
        stbtt_MakeCodepointBitmap(&g_debugger.ttf_font, g_char_bitmap, char_w, char_h, char_w, scale, scale, (int)(unsigned char)c);

        uint8_t fg_r = (uint8_t)((fg >> 16) & 0xFF);
        uint8_t fg_g = (uint8_t)((fg >> 8) & 0xFF);
        uint8_t fg_b = (uint8_t)(fg & 0xFF);
        uint8_t bg_r = (uint8_t)((bg >> 16) & 0xFF);
        uint8_t bg_g = (uint8_t)((bg >> 8) & 0xFF);
        uint8_t bg_b = (uint8_t)(bg & 0xFF);

        for (int cy = 0; cy < char_h; cy++) {
            for (int cx = 0; cx < char_w; cx++) {
                uint8_t alpha = g_char_bitmap[cy * char_w + cx];
                int dx = (int)x + x0 + cx;
                int dy = (int)y + g_debugger.ttf_ascent + y0 + cy;

                if (dx >= 0 && dx < (int)g_debugger.screen_width &&
                    dy >= 0 && dy < (int)g_debugger.screen_height) {
                    uint32_t color;
                    if (alpha == 255) {
                        color = fg;
                    } else if (alpha == 0) {
                        color = bg;
                    } else {
                        uint8_t nr = (uint8_t)(((uint32_t)fg_r * alpha + (uint32_t)bg_r * (255U - alpha)) / 255U);
                        uint8_t ng = (uint8_t)(((uint32_t)fg_g * alpha + (uint32_t)bg_g * (255U - alpha)) / 255U);
                        uint8_t nb = (uint8_t)(((uint32_t)fg_b * alpha + (uint32_t)bg_b * (255U - alpha)) / 255U);
                        color = ((uint32_t)0xFF << 24) | ((uint32_t)nr << 16) | ((uint32_t)ng << 8) | (uint32_t)nb;
                    }
                    if (g_debugger.framebuffer_base != 0) {
                        uint32_t* fb = (uint32_t*)g_debugger.framebuffer_base;
                        fb[(uint32_t)dy * g_debugger.pixels_per_scan_line + (uint32_t)dx] = color;
                    } else {
                        driver_manager_display_draw_pixel((uint32_t)dx, (uint32_t)dy, color);
                    }
                }
            }
        }
    }

    uint32_t advance_px = (uint32_t)(advance * scale);
    if (advance_px < 1) advance_px = 1;
    return advance_px;
}

static void debugger_advance_cursor(uint32_t advance)
{
    g_debugger.cursor_x += advance;

    if (g_debugger.cursor_x + g_debugger.char_width > g_debugger.screen_width) {
        g_debugger.cursor_x = 0;
        g_debugger.cursor_y += g_debugger.char_height;

        if (g_debugger.cursor_y + g_debugger.char_height > g_debugger.screen_height) {
            debug_clear_screen();
            g_debugger.cursor_y = 0;
        }
    }
}

void debugger_init(BOOT_INFO *boot_info)
{
    if (boot_info == NULL) {
        return;
    }

    g_debugger.framebuffer_base = boot_info->FrameBufferBase;
    g_debugger.pixels_per_scan_line = boot_info->PixelsPerScanLine;
    g_debugger.screen_width = boot_info->HorizontalResolution;
    g_debugger.screen_height = boot_info->VerticalResolution;

    if (boot_info->FontDataAddress != 0) {
        const unsigned char* data = (const unsigned char*)(uintptr_t)boot_info->FontDataAddress;
        int offset = stbtt_GetFontOffsetForIndex(data, 0);
        g_debugger.ttf_valid = stbtt_InitFont(&g_debugger.ttf_font, data, offset);
        if (g_debugger.ttf_valid) {
            g_debugger.ttf_scale = stbtt_ScaleForPixelHeight(&g_debugger.ttf_font, DEBUG_FONT_HEIGHT);
            int ascent, descent, lineGap;
            stbtt_GetFontVMetrics(&g_debugger.ttf_font, &ascent, &descent, &lineGap);
            g_debugger.ttf_ascent = (int)(ascent * g_debugger.ttf_scale);
            g_debugger.ttf_descent = (int)(descent * g_debugger.ttf_scale);
            g_debugger.ttf_line_gap = (int)(lineGap * g_debugger.ttf_scale);
            g_debugger.char_height = (uint32_t)(g_debugger.ttf_ascent - g_debugger.ttf_descent + g_debugger.ttf_line_gap);
            if (g_debugger.char_height < 1) g_debugger.char_height = 1;
            int advance;
            stbtt_GetCodepointHMetrics(&g_debugger.ttf_font, '0', &advance, NULL);
            g_debugger.char_width = (uint32_t)(advance * g_debugger.ttf_scale);
            if (g_debugger.char_width < 1) g_debugger.char_width = 1;
        }
    }

    g_debugger.max_cols = g_debugger.screen_width / g_debugger.char_width;
    g_debugger.max_rows = g_debugger.screen_height / g_debugger.char_height;

    g_debugger.cursor_x = 0;
    g_debugger.cursor_y = 0;
    g_debugger.initialized = true;
    debugger_display_init();
    serial_set_screen_mirror(debug_putchar);
}

bool debugger_display_init(void)
{
    if (!driver_manager_display_is_ready()) {
        return false;
    }

    g_debugger.screen_width = driver_manager_display_width();
    g_debugger.screen_height = driver_manager_display_height();

    if (g_debugger.screen_width == 0 || g_debugger.screen_height == 0) {
        return false;
    }

    g_debugger.max_cols = g_debugger.screen_width / g_debugger.char_width;
    g_debugger.max_rows = g_debugger.screen_height / g_debugger.char_height;

    g_debugger.cursor_x = 0;
    g_debugger.cursor_y = 0;
    g_debugger.framebuffer_base = 0; 
    g_debugger.initialized = true;
    serial_set_screen_mirror(debug_putchar);

    return true;
}

void debug_putchar(char c)
{
    if (!g_debugger.initialized) {
        return;
    }

    if (c == '\n') {
        g_debugger.cursor_x = 0;
        g_debugger.cursor_y += g_debugger.char_height;

        if (g_debugger.cursor_y + g_debugger.char_height > g_debugger.screen_height) {
            debug_clear_screen();
            g_debugger.cursor_y = 0;
        }
        debugger_present_if_needed();
    } else if (c == '\r') {
        g_debugger.cursor_x = 0;
    } else if (c == '\t') {
        for (int i = 0; i < 4; i++) {
            debugger_advance_cursor(g_debugger.char_width);
        }
    } else if (c == '\b') {
        if (g_debugger.cursor_x >= g_debugger.char_width) {
            g_debugger.cursor_x -= g_debugger.char_width;
            debugger_draw_char_at(' ', g_debugger.cursor_x, g_debugger.cursor_y,
                                  g_debugger.fg_color, g_debugger.bg_color);
        }
    } else {
        uint32_t advance = debugger_draw_char_at(c, g_debugger.cursor_x, g_debugger.cursor_y,
                                                  g_debugger.fg_color, g_debugger.bg_color);
        debugger_advance_cursor(advance);
    }
}

void debug_printf(const char *format, ...)
{
    if (format == NULL) {
        return;
    }

    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    serial_write_string(buffer);
}

void debug_clear_screen(void)
{
    if (!g_debugger.initialized) {
        return;
    }

    if (g_debugger.framebuffer_base != 0) {
        uint32_t* fb = (uint32_t*)g_debugger.framebuffer_base;
        for (uint32_t y = 0; y < g_debugger.screen_height; y++) {
            for (uint32_t x = 0; x < g_debugger.screen_width; x++) {
                fb[y * g_debugger.pixels_per_scan_line + x] = g_debugger.bg_color;
            }
        }
    } else {
        driver_manager_display_fill_rect(0, 0, g_debugger.screen_width, g_debugger.screen_height,
                                         g_debugger.bg_color);
    }

    g_debugger.cursor_x = 0;
    g_debugger.cursor_y = 0;

    debugger_present_if_needed();
}

void debug_reset_cursor(void)
{
    g_debugger.cursor_x = 0;
    g_debugger.cursor_y = 0;
}

void debug_present(void)
{
    debugger_present_if_needed();
}
