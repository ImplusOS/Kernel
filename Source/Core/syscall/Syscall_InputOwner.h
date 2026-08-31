#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Minimal exclusive-ownership gate for the raw HID (keyboard/mouse) event
 * stream. This is the only piece of "window manager" awareness the kernel
 * still needs: once a userland compositor claims ownership, every other
 * process is denied direct SYSCALL_INPUT_READ_KEYBOARD/MOUSE access so it
 * cannot race the compositor for input events. All actual window/cursor
 * policy (position tracking, clamping, keyboard routing) lives entirely in
 * userland now — see Userland/Application/com.ImplusOS.windowmanager/.
 *
 * Roughly analogous to DRM-master / seat ownership in a Wayland-style stack.
 */

/* Attempts to claim exclusive input ownership for `pid`. Succeeds if no
 * owner is currently registered, or the previous owner's process is no
 * longer alive (stale ownership left behind by a crashed/exited
 * compositor). Returns true on success, false if a live owner already
 * holds the claim. */
bool syscall_input_owner_claim(int32_t pid);

/* Releases ownership if `pid` currently holds it. Safe to call
 * unconditionally (e.g. from process teardown) even if `pid` never held
 * ownership. */
void syscall_input_owner_release(int32_t pid);

/* Returns the current owner pid, or -1 if none is registered. */
int32_t syscall_input_owner_get(void);

/* Returns true if `pid` is the current input owner. */
bool syscall_input_owner_is(int32_t pid);
