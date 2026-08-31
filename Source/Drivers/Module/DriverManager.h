#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kernel/interfaces/device.h"
#include "DriverBinary.h"
#include "Core/vfs/VFS.h"
#include "Drivers/Module/PCI_Main.h"
#include "SerialManager.h"

typedef device_type_t driver_manager_kind_t;

void driver_manager_init(void);
void driver_manager_schedule_hotplug_poll(void);
bool driver_manager_check_hotplug_poll(void);
void driver_manager_hotplug_poll(void);

bool driver_manager_attach(const char *module_name,
                           driver_manager_kind_t kind,
                           const void *driver_api);
bool driver_manager_detach(const char *module_name);
void driver_manager_detach_all(void);

const void *driver_manager_get_by_module_name(const char *module_name);
const void *driver_manager_get_by_kind(driver_manager_kind_t kind);
const void *driver_manager_get_named(driver_manager_kind_t kind,
                                     const char *module_name);
const device_t *driver_manager_find(driver_manager_kind_t kind,
                                    const char *module_name);
const device_t *device_manager_find(device_type_t type,
                                   const char *module_name);

const pci_driver_t *driver_manager_get_pci_driver(void);
/* First driver registered as DEVICE_TYPE_INPUT (historically the PS/2
 * controller module, hence the name) -- no module filename is hard-coded. */
const driver_input_t *driver_manager_get_ps2_driver(void);
const usb_master_vtable_t *driver_manager_get_usb_driver(void);
const driver_display_t *driver_manager_get_display_driver(const char *module_name);
const driver_nic_t *driver_manager_get_nic_driver(void);
const driver_serial_t *driver_manager_get_serial_driver(void);

bool driver_manager_unload_module(const char *module_name);
bool driver_manager_reload_module(const char *module_name);

bool driver_manager_fs_init(void);
bool driver_manager_fs_find_file(const char *path, vfs_file_t *file);
bool driver_manager_fs_read_file(vfs_file_t *file, uint8_t *buffer);
bool driver_manager_fs_write_file(vfs_file_t *file, const uint8_t *buffer);
bool driver_manager_fs_read_at(vfs_file_t *file, uint32_t offset, uint8_t *buffer, uint32_t size);
bool driver_manager_fs_write_at(vfs_file_t *file, uint32_t offset, const uint8_t *buffer, uint32_t size);
bool driver_manager_fs_truncate(vfs_file_t *file, uint32_t new_size);
uint32_t driver_manager_fs_get_file_size(vfs_file_t *file);
bool driver_manager_fs_close_file(vfs_file_t *file);
void driver_manager_fs_list_root_files(void);
bool driver_manager_fs_creat(const char *path);
bool driver_manager_fs_mkdir(const char *path);
int32_t driver_manager_fs_opendir(const char *path);
int32_t driver_manager_fs_readdir(int32_t dir_handle, vfs_dirent_t *out_entry);
int32_t driver_manager_fs_closedir(int32_t dir_handle);
bool driver_manager_fs_unlink(const char *path);
void driver_manager_fs_set_case_sensitive_lookup(bool enabled);
bool driver_manager_fs_get_case_sensitive_lookup(void);

bool driver_manager_display_init(void);
bool driver_manager_display_is_ready(void);
uint32_t driver_manager_display_width(void);
uint32_t driver_manager_display_height(void);
void driver_manager_display_draw_pixel(uint32_t x, uint32_t y, uint32_t color);
uint32_t driver_manager_display_get_pixel(uint32_t x, uint32_t y);
void driver_manager_display_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);
void driver_manager_display_present(void);
void driver_manager_display_present_rects(const display_rect_t *rects, uint32_t count);
void *driver_manager_display_get_framebuffer(void);
uint32_t driver_manager_display_generation(void);
bool driver_manager_display_poll_config(void);
bool driver_manager_display_get_topology(display_topology_t *out_topology);
bool driver_manager_display_get_monitor_info(uint32_t monitor_index,
                                             display_monitor_info_t *out_info);
bool driver_manager_display_get_monitor_mode_info(uint32_t monitor_index,
                                                  uint32_t mode_index,
                                                  display_mode_info_t *out_info);
bool driver_manager_display_set_monitor_mode(uint32_t monitor_index,
                                             uint32_t mode_index);

bool driver_manager_input_ps2_init(void);
void driver_manager_input_ps2_poll(void);
int32_t driver_manager_input_ps2_read_keyboard(driver_keyboard_event_t *out_event);
int32_t driver_manager_input_ps2_read_mouse(driver_mouse_event_t *out_event);

void driver_manager_input_usb_init(void);
bool driver_manager_input_usb_read_sectors(uint64_t lba, uint8_t *buffer, uint32_t sectors);
bool driver_manager_input_usb_write_sectors(uint64_t lba, const uint8_t *buffer, uint32_t sectors);
int32_t driver_manager_input_usb_read_keyboard(driver_keyboard_event_t *out_event);
int32_t driver_manager_input_usb_read_mouse(driver_mouse_event_t *out_event);
void driver_manager_input_usb_poll(void);
void driver_manager_input_usb_drain_keyboard(driver_keyboard_event_t *tmp,
                                             void (*forward)(driver_keyboard_event_t *));
void driver_manager_input_usb_drain_mouse(driver_mouse_event_t *tmp,
                                          void (*forward)(driver_mouse_event_t *));
void driver_manager_input_usb_schedule_poll(void);
bool driver_manager_input_usb_check_poll(void);

bool driver_manager_nic_init(void);
bool driver_manager_nic_is_ready(void);
uint16_t driver_manager_nic_mtu(void);
void driver_manager_nic_get_mac(uint8_t mac_out[6]);
bool driver_manager_nic_send_frame(const uint8_t *frame, uint16_t frame_len);
void driver_manager_nic_poll(void);
void driver_manager_nic_set_rx_callback(driver_nic_rx_callback_t cb);
void driver_manager_nic_schedule_poll(void);
bool driver_manager_nic_check_poll(void);

bool driver_manager_nic_wifi_scan_start(void);
uint32_t driver_manager_nic_wifi_get_scan_results(driver_wifi_scan_result_t *out, uint32_t max_count);
bool driver_manager_nic_wifi_connect(const char *ssid, const char *psk);
void driver_manager_nic_wifi_disconnect(void);
void driver_manager_nic_wifi_get_status(driver_wifi_status_t *out_status);
