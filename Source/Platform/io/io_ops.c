#include "interfaces/io_ops.h"

#include "Platform/io/IO_Main.h"

static const io_ops_t g_io_ops = {
    .inb = inb,
    .outb = outb,
    .inw = inw,
    .outw = outw,
    .inl = inl,
    .outl = outl,
};

const io_ops_t *io_ops_get(void)
{
    return &g_io_ops;
}
