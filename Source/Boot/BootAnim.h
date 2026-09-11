#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "kernel/boot_info.h"

/*
 * Boot hand-off animation.
 *
 * Played once, right after the last kernel init phase that draws to the boot
 * screen and before control reaches /Userland/Userland.ELF. The frozen boot
 * screen is captured and then two animations run together on a single
 * ease-in-out curve:
 *
 *   1. scale   100% -> 150%  (about the screen centre)
 *   2. opacity 100% ->   0%  (towards black)
 *
 * so the kernel screen appears to push past the viewer as it dissolves. The
 * matching half of the transition -- 50% -> 100% with the opacity coming back
 * -- is played by the init process on its first screen (Userland/Userland.c),
 * which is why this one has to end on a fully black panel.
 *
 * Everything here is integer/fixed-point: the kernel is built
 * -mgeneral-regs-only (see Kernel/Source/Makefile) so no float or SSE may
 * appear in this translation unit.
 *
 * Returns false when the transition could not be played (no framebuffer, or
 * the snapshot allocation failed); the caller should then just carry on --
 * nothing is left half-drawn.
 */
bool boot_anim_play_handoff(const BOOT_INFO *boot_info);
