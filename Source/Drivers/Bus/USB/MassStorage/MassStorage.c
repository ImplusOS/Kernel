#include "MassStorage.h"
#include "../USB_Main.h"
#include "../EHCI/EHCI.h"
#include "../XHCI/XHCI.h"
#include "Drivers/Module/DriverBinary.h"

extern const driver_binary_t *g_api;

#define CBW_SIGNATURE 0x43425355
#define CSW_SIGNATURE 0x53425355

typedef struct {
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t  bmCBWFlags;
    uint8_t  bCBWLUN;
    uint8_t  bCBWCBLength;
    uint8_t  CBWCB[16];
} __attribute__((packed)) usb_bot_cbw_t;

typedef struct {
    uint32_t dCSWSignature;
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t  bCSWStatus;
} __attribute__((packed)) usb_bot_csw_t;

extern uint8_t  g_mass_storage_addr;
extern uint8_t  g_mass_storage_ep_in;
extern uint8_t  g_mass_storage_ep_out;
extern uint8_t  g_mass_storage_interface;
extern uint16_t g_mass_storage_ep_in_mps;
extern uint16_t g_mass_storage_ep_out_mps;

typedef struct {
    uint8_t  addr;
    uint8_t  interface;
    uint8_t  ep_in;
    uint8_t  ep_out;
    uint16_t ep_in_mps;
    uint16_t ep_out_mps;
    uint32_t block_size;
    uint32_t max_lba;
    bool     write_protected;
} usb_bot_device_t;

#define MAX_BOT_DEVICES 4
static usb_bot_device_t g_bot_devices[MAX_BOT_DEVICES];
static uint32_t g_bot_device_count = 0;
static uint32_t g_bot_current_device = 0;

static uint32_t bot_tag        = 1;
static uint32_t bot_block_size = 512;
static uint32_t bot_max_lba    = 0;
static bool     bot_write_protected = false;

#define BOT_BOUNCE_BUFFER_SIZE      (64u * 1024u)
#define BOT_SCSI_RW10_MAX_BLOCKS    0xFFFFu

static uint8_t  bot_bounce_buffer[BOT_BOUNCE_BUFFER_SIZE];

void bot_reset_devices(void)
{
    if (g_api) {
        g_api->memset(g_bot_devices, 0, sizeof(g_bot_devices));
    } else {
        for (uint32_t i = 0; i < MAX_BOT_DEVICES; ++i) {
            g_bot_devices[i] = (usb_bot_device_t){0};
        }
    }
    g_bot_device_count = 0;
    g_bot_current_device = 0;
    bot_block_size = 512;
    bot_max_lba = 0;
    bot_write_protected = false;
}

void bot_add_device(uint8_t addr, uint8_t interface, uint8_t ep_in, uint8_t ep_out, uint16_t ep_in_mps, uint16_t ep_out_mps)
{
    if (g_bot_device_count < MAX_BOT_DEVICES) {
        g_bot_devices[g_bot_device_count].addr = addr;
        g_bot_devices[g_bot_device_count].interface = interface;
        g_bot_devices[g_bot_device_count].ep_in = ep_in & 0x0F;
        g_bot_devices[g_bot_device_count].ep_out = ep_out & 0x0F;
        g_bot_devices[g_bot_device_count].ep_in_mps = ep_in_mps ? ep_in_mps : 512;
        g_bot_devices[g_bot_device_count].ep_out_mps = ep_out_mps ? ep_out_mps : 512;
        g_bot_device_count++;
    }
}

uint32_t bot_get_device_count(void)
{
    return g_bot_device_count;
}

bool bot_select_device(uint32_t index)
{
    if (index >= g_bot_device_count) return false;
    g_bot_current_device = index;
    g_mass_storage_addr = g_bot_devices[index].addr;
    g_mass_storage_interface = g_bot_devices[index].interface;
    g_mass_storage_ep_in = g_bot_devices[index].ep_in;
    g_mass_storage_ep_out = g_bot_devices[index].ep_out;
    g_mass_storage_ep_in_mps = g_bot_devices[index].ep_in_mps;
    g_mass_storage_ep_out_mps = g_bot_devices[index].ep_out_mps;
    bot_block_size = g_bot_devices[index].block_size;
    bot_max_lba = g_bot_devices[index].max_lba;
    bot_write_protected = g_bot_devices[index].write_protected;
    return true;
}

uint64_t bot_get_total_bytes(void)
{
    if (g_bot_device_count == 0) return 0;
    return (uint64_t)(g_bot_devices[g_bot_current_device].max_lba + 1) * g_bot_devices[g_bot_current_device].block_size;
}

uint32_t bot_get_block_size(void)
{
    return bot_block_size;
}

bool bot_is_read_only(void)
{
    return bot_write_protected;
}

static void bot_clear_endpoint_halts(void);
bool bot_test_unit_ready(void);
static bool bot_execute_command_ex(void *cbw_cb, uint8_t cb_len, uint8_t dir_in,
                                   uint32_t data_len, void *data_buf,
                                   bool request_sense_on_failure);
static bool bot_execute_command(void *cbw_cb, uint8_t cb_len, uint8_t dir_in,
                                uint32_t data_len, void *data_buf)
{
    return bot_execute_command_ex(cbw_cb, cb_len, dir_in, data_len, data_buf,
                                  true);
}

static bool bot_get_sector_geometry(uint32_t *factor_out,
                                    uint32_t *block_bytes_out)
{
    uint32_t factor = 1u;

    if (bot_block_size > 512u) {
        if ((bot_block_size % 512u) != 0u) {
            return false;
        }
        factor = bot_block_size / 512u;
    }

    if (factor == 0u || factor > (UINT32_MAX / 512u)) {
        return false;
    }

    if (factor_out != NULL) {
        *factor_out = factor;
    }
    if (block_bytes_out != NULL) {
        *block_bytes_out = factor * 512u;
    }
    return true;
}

static uint32_t bot_max_bulk_transfer_bytes(void)
{
    usb_hc_type_t hc = usb_get_device_hc_type(g_mass_storage_addr);
    if (hc == USB_HC_NONE) {
        hc = usb_get_hc_type();
    }

    if (hc == USB_HC_XHCI) {
        uint32_t max_bytes = xhci_get_max_bulk_transfer_size();
        if (max_bytes != 0u) {
            return max_bytes;
        }
    }

    return BOT_BOUNCE_BUFFER_SIZE;
}

static uint32_t bot_max_transfer_blocks(uint32_t block_bytes)
{
    if (block_bytes == 0u) {
        return 0u;
    }

    uint32_t blocks = bot_max_bulk_transfer_bytes() / block_bytes;
    if (blocks > BOT_SCSI_RW10_MAX_BLOCKS) {
        blocks = BOT_SCSI_RW10_MAX_BLOCKS;
    }
    return blocks;
}

static void bot_wait_ms(uint32_t ms)
{
    if (usb_get_hc_type() == USB_HC_XHCI) xhci_delay_ms(ms);
    else                                   ehci_delay_ms(ms);
}

static bool bot_wait_ready(uint32_t max_attempts,
                           uint32_t initial_delay_ms,
                           uint32_t max_delay_ms)
{
    uint32_t delay_ms = initial_delay_ms;

    for (uint32_t i = 0; i < max_attempts; i++) {
        if (bot_test_unit_ready()) {
            return true;
        }

        if (delay_ms != 0) {
            bot_wait_ms(delay_ms);
            if (delay_ms < max_delay_ms) {
                delay_ms <<= 1;
                if (delay_ms > max_delay_ms) delay_ms = max_delay_ms;
            }
        }
    }

    return false;
}

static bool bot_retry_simple_command(bool (*fn)(void),
                                     uint32_t max_attempts,
                                     uint32_t retry_delay_ms)
{
    for (uint32_t i = 0; i < max_attempts; i++) {
        if (fn()) {
            return true;
        }

        bot_clear_endpoint_halts();
        if (retry_delay_ms != 0) {
            bot_wait_ms(retry_delay_ms);
        }
    }

    return false;
}

static bool bot_mass_storage_reset(void)
{
    return usb_submit_control(g_mass_storage_addr, 0x21, 0xFF, 0,
                              g_mass_storage_interface, 0, NULL);
}

static void bot_recover_transport(const char *reason)
{
    (void)reason;
    bot_mass_storage_reset();
    bot_wait_ms(100);
    bot_clear_endpoint_halts();
    bot_wait_ms(50);
}

static void bot_clear_endpoint_halts(void)
{
    usb_submit_control(g_mass_storage_addr, 0x02, 0x01, 0x0000,
                       g_mass_storage_ep_in | 0x80, 0, NULL);

    usb_submit_control(g_mass_storage_addr, 0x02, 0x01, 0x0000,
                       g_mass_storage_ep_out, 0, NULL);
}

bool bot_test_unit_ready(void)
{
    uint8_t cb[16] = {0};
    cb[0] = 0x00;
    return bot_execute_command(cb, 6, 0, 0, NULL);
}

bool bot_inquiry(void)
{
    uint8_t cb[16] = {0};
    cb[0] = 0x12;
    cb[4] = 36;
    uint8_t buf[36] = {0};
    bool ok = bot_execute_command(cb, 6, 1, 36, buf);
    return ok;
}

static bool bot_read_capacity_10(void)
{
    uint8_t cb[16] = {0};
    cb[0] = 0x25;
    uint8_t buf[8] = {0};
    if (!bot_execute_command(cb, 10, 1, 8, buf)) return false;

    bot_max_lba    = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                   | ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    bot_block_size = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16)
                   | ((uint32_t)buf[6] <<  8) |  (uint32_t)buf[7];

    if (bot_block_size == 0) bot_block_size = 512;

    if (g_bot_device_count > 0) {
        g_bot_devices[g_bot_current_device].max_lba = bot_max_lba;
        g_bot_devices[g_bot_current_device].block_size = bot_block_size;
    }

    return true;
}

static bool bot_read_capacity_16(void)
{
    uint8_t cb[16] = {0};
    cb[0] = 0x9E;
    cb[1] = 0x10;
    cb[10] = 0;
    cb[11] = 0;
    cb[12] = 0;
    cb[13] = 32;

    uint8_t buf[32] = {0};
    if (!bot_execute_command(cb, 16, 1, sizeof(buf), buf)) return false;

    uint64_t last_lba = ((uint64_t)buf[0] << 56) |
                        ((uint64_t)buf[1] << 48) |
                        ((uint64_t)buf[2] << 40) |
                        ((uint64_t)buf[3] << 32) |
                        ((uint64_t)buf[4] << 24) |
                        ((uint64_t)buf[5] << 16) |
                        ((uint64_t)buf[6] << 8)  |
                        (uint64_t)buf[7];
    uint32_t block_size = ((uint32_t)buf[8] << 24) |
                          ((uint32_t)buf[9] << 16) |
                          ((uint32_t)buf[10] << 8) |
                          (uint32_t)buf[11];

    if (last_lba > 0xFFFFFFFFULL || block_size == 0u) {
        return false;
    }

    bot_max_lba = (uint32_t)last_lba;
    bot_block_size = block_size;

    if (g_bot_device_count > 0) {
        g_bot_devices[g_bot_current_device].max_lba = bot_max_lba;
        g_bot_devices[g_bot_current_device].block_size = bot_block_size;
    }

    return true;
}

bool bot_read_capacity(void)
{
    if (bot_read_capacity_10()) {
        return true;
    }

    bot_clear_endpoint_halts();
    return bot_read_capacity_16();
}

static bool __attribute__((unused)) bot_read_write_protect(bool *write_protected)
{
    uint8_t cb[16] = {0};
    uint8_t buf[4] = {0};

    cb[0] = 0x1A;
    cb[1] = 0x08;
    cb[2] = 0x3F;
    cb[4] = sizeof(buf);
    if (!bot_execute_command(cb, 6, 1, sizeof(buf), buf)) {
        return false;
    }
    if (write_protected != NULL) {
        *write_protected = (buf[2] & 0x80u) != 0u;
    }
    return true;
}

bool bot_init(void)
{
    if (g_bot_device_count == 0) {
        return false;
    }

    uint32_t ready_count = 0;
    for (uint32_t i = 0; i < g_bot_device_count; i++) {
        bot_select_device(i);

        bot_wait_ms(100);
        bot_clear_endpoint_halts();
        bot_wait_ms(200);

        if (!bot_retry_simple_command(bot_inquiry, 8, 150)) {
            continue;
        }

        if (!bot_wait_ready(20, 50, 1000)) {
            bot_mass_storage_reset();
            bot_wait_ms(100);
            bot_clear_endpoint_halts();
            if (!bot_wait_ready(20, 100, 1500)) {
                continue;
            }
        }

        if (!bot_retry_simple_command(bot_read_capacity, 8, 150)) {
            continue;
        }

        bot_write_protected = false;
        g_bot_devices[i].write_protected = false;

        (void)bot_wait_ready(4, 10, 40);

        if (ready_count != i) {
            g_bot_devices[ready_count] = g_bot_devices[i];
        }
        ready_count++;
    }

    g_bot_device_count = ready_count;
    if (g_bot_device_count > 0) {
        bot_select_device(0);
        return true;
    }

    return false;
}

static bool bot_execute_command_ex(void *cbw_cb, uint8_t cb_len, uint8_t dir_in,
                                   uint32_t data_len, void *data_buf,
                                   bool request_sense_on_failure)
{
    (void)request_sense_on_failure;

    usb_bot_cbw_t cbw = {0};
    uint32_t tag = bot_tag++;

    cbw.dCBWSignature          = CBW_SIGNATURE;
    cbw.dCBWTag                = tag;
    cbw.dCBWDataTransferLength = data_len;
    cbw.bmCBWFlags             = dir_in ? 0x80 : 0x00;
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = cb_len;

    if (g_api) g_api->memcpy(cbw.CBWCB, cbw_cb, cb_len);

    if (!usb_submit_bulk(g_mass_storage_addr,
                         g_mass_storage_ep_out,
                         g_mass_storage_ep_out_mps,
                         0,
                         &cbw, sizeof(cbw))) {
        bot_recover_transport("CBW failed");
        return false;
    }

    if (data_len > 0 && data_buf != NULL) {
        uint8_t  pid = dir_in ? 1 : 0;
        uint8_t  ep  = dir_in ? g_mass_storage_ep_in : g_mass_storage_ep_out;
        uint16_t mps = dir_in ? g_mass_storage_ep_in_mps : g_mass_storage_ep_out_mps;

        if (!usb_submit_bulk(g_mass_storage_addr, ep, mps, pid, data_buf, data_len)) {
            bot_recover_transport("DATA failed");
            return false;
        }
    }

    usb_bot_csw_t csw = {0};

    if (!usb_submit_bulk(g_mass_storage_addr,
                         g_mass_storage_ep_in,
                         g_mass_storage_ep_in_mps,
                         1,
                         &csw, sizeof(csw))) {
        bot_recover_transport("CSW failed");
        return false;
    }

    if (csw.dCSWSignature != CSW_SIGNATURE) {
        bot_recover_transport("bad CSW signature");
        return false;
    }
    if (csw.dCSWTag != tag) {
        bot_recover_transport("bad CSW tag");
        return false;
    }

    if (csw.bCSWStatus != 0) {
        return false;
    }

    return true;
}

static bool bot_read_phys_blocks(uint32_t phys_lba,
                                 uint32_t phys_count,
                                 uint8_t *buffer)
{
    uint32_t block_bytes = 0u;

    if (phys_count == 0u) {
        return true;
    }
    if (buffer == NULL ||
        phys_count > BOT_SCSI_RW10_MAX_BLOCKS ||
        !bot_get_sector_geometry(NULL, &block_bytes) ||
        block_bytes == 0u ||
        phys_count > (UINT32_MAX / block_bytes)) {
        return false;
    }

    uint32_t data_len = phys_count * block_bytes;

    for (int retry = 0; retry < 3; retry++) {
        uint8_t cb[16] = {0};
        cb[0] = 0x28;
        cb[2] = (uint8_t)(phys_lba >> 24);
        cb[3] = (uint8_t)(phys_lba >> 16);
        cb[4] = (uint8_t)(phys_lba >>  8);
        cb[5] = (uint8_t)(phys_lba);
        cb[7] = (uint8_t)(phys_count >> 8);
        cb[8] = (uint8_t)(phys_count);

        if (bot_execute_command(cb, 10, 1, data_len, buffer)) {
            return true;
        }

        bot_clear_endpoint_halts();
        bot_wait_ms(50);
        bot_test_unit_ready();
    }

    return false;
}

bool bot_read_sectors(uint32_t lba, uint8_t *buffer, uint32_t sectors)
{
    if (g_mass_storage_addr == 0 || g_mass_storage_ep_in == 0 || g_mass_storage_ep_out == 0) return false;
    if (sectors == 0u) return true;
    if (buffer == NULL || lba > UINT32_MAX - (sectors - 1u)) return false;

    uint32_t factor = 0u;
    uint32_t block_bytes = 0u;
    if (!bot_get_sector_geometry(&factor, &block_bytes)) {
        return false;
    }

    uint32_t max_direct_blocks = bot_max_transfer_blocks(block_bytes);
    if (max_direct_blocks == 0u) {
        return false;
    }

    uint32_t total_sectors_done = 0;

    while (total_sectors_done < sectors) {
        uint32_t current_lba = lba + total_sectors_done;
        uint32_t remaining = sectors - total_sectors_done;
        uint32_t sector_offset = current_lba % factor;

        if (sector_offset == 0u) {
            uint32_t full_blocks = remaining / factor;
            if (full_blocks != 0u) {
                uint32_t phys_lba = current_lba / factor;
                uint32_t phys_count = full_blocks;
                if (phys_count > max_direct_blocks) {
                    phys_count = max_direct_blocks;
                }

                if (!bot_read_phys_blocks(
                        phys_lba,
                        phys_count,
                        buffer + (size_t)total_sectors_done * 512u)) {
                    return false;
                }

                total_sectors_done += phys_count * factor;
                continue;
            }
        }

        if (block_bytes > BOT_BOUNCE_BUFFER_SIZE) {
            return false;
        }

        uint32_t bounce_sectors =
            (sector_offset == 0u) ? factor : (factor - sector_offset);
        if (bounce_sectors > remaining) {
            bounce_sectors = remaining;
        }

        uint32_t phys_lba = current_lba / factor;
        if (!bot_read_phys_blocks(phys_lba, 1u, bot_bounce_buffer)) {
            return false;
        }

        uint32_t offset = sector_offset * 512u;
        if (g_api) {
            g_api->memcpy(buffer + (size_t)total_sectors_done * 512u,
                          bot_bounce_buffer + offset,
                          (size_t)bounce_sectors * 512u);
        }
        total_sectors_done += bounce_sectors;
    }

    return true;
}

bool bot_write_sectors(uint32_t lba, const uint8_t *buffer, uint32_t sectors)
{
    if (bot_write_protected || g_mass_storage_addr == 0 ||
        g_mass_storage_ep_in == 0 || g_mass_storage_ep_out == 0) return false;

    uint32_t total_sectors_done = 0;

    while (total_sectors_done < sectors) {
        uint32_t current_lba = lba + total_sectors_done;
        uint32_t remaining = sectors - total_sectors_done;

        uint32_t factor = (bot_block_size > 512) ? (bot_block_size / 512) : 1;
        uint32_t phys_lba = current_lba / factor;
        uint32_t max_phys_blocks = sizeof(bot_bounce_buffer) / (factor * 512);
        if (max_phys_blocks == 0) return false;

        uint32_t phys_end = (current_lba + remaining - 1) / factor;
        uint32_t phys_count = phys_end - phys_lba + 1;

        if (phys_count > max_phys_blocks) {
            phys_count = max_phys_blocks;
            uint32_t end_sector = (phys_lba + phys_count) * factor - 1;
            uint32_t chunk_sectors = end_sector - current_lba + 1;
            if (chunk_sectors > remaining) chunk_sectors = remaining;
            remaining = chunk_sectors;
        }

        bool chunk_ok = false;
        for (int retry = 0; retry < 3; retry++) {
            uint32_t offset = (current_lba % factor) * 512;
            
            if (factor > 1 && (offset != 0 || remaining < phys_count * factor)) {
                uint8_t cb_rd[16] = {0};
                cb_rd[0] = 0x28;
                cb_rd[2] = (uint8_t)(phys_lba >> 24);
                cb_rd[3] = (uint8_t)(phys_lba >> 16);
                cb_rd[4] = (uint8_t)(phys_lba >>  8);
                cb_rd[5] = (uint8_t)(phys_lba);
                cb_rd[7] = (uint8_t)(phys_count >> 8);
                cb_rd[8] = (uint8_t)(phys_count);
                if (!bot_execute_command(cb_rd, 10, 1, phys_count * (factor * 512), bot_bounce_buffer)) {
                    continue;
                }
            } else {
                if (g_api) g_api->memset(bot_bounce_buffer, 0, phys_count * (factor * 512));
            }

            if (g_api) g_api->memcpy(bot_bounce_buffer + offset,
                                     buffer + total_sectors_done * 512,
                                     remaining * 512);

            uint8_t cb[16] = {0};
            cb[0] = 0x2A;
            cb[2] = (uint8_t)(phys_lba >> 24);
            cb[3] = (uint8_t)(phys_lba >> 16);
            cb[4] = (uint8_t)(phys_lba >>  8);
            cb[5] = (uint8_t)(phys_lba);
            cb[7] = (uint8_t)(phys_count >> 8);
            cb[8] = (uint8_t)(phys_count);

            bool result = bot_execute_command(cb, 10, 0,
                                              phys_count * (factor * 512),
                                              bot_bounce_buffer);

            if (result) {
                chunk_ok = true;
                break;
            }

            bot_clear_endpoint_halts();
            if (usb_get_hc_type() == USB_HC_XHCI) xhci_delay_ms(50);
            else                                   ehci_delay_ms(50);
            bot_test_unit_ready();
        }

        if (!chunk_ok) return false;
        total_sectors_done += remaining;
    }

    return true;
}

bool bot_flush(void)
{
    if (g_mass_storage_addr == 0 || g_mass_storage_ep_in == 0 ||
        g_mass_storage_ep_out == 0) {
        return false;
    }
    uint8_t cb[16] = {0};
    cb[0] = 0x35;
    return bot_execute_command(cb, 10u, 0u, 0u, NULL);
}
