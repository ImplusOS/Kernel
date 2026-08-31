#ifndef KERNEL_MAIN_H
#define KERNEL_MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef uint64_t UINTN;
typedef uint32_t UINT32;

typedef uint64_t EFI_PHYSICAL_ADDRESS;

#define MAX_LOADED_FILES 16
#define LOADED_FILE_NAME_MAX 64

#define BOOT_DRIVE_TYPE_UNKNOWN 0
#define BOOT_DRIVE_TYPE_IDE     1
#define BOOT_DRIVE_TYPE_USB     2
#define BOOT_DRIVE_TYPE_AHCI    3
#define BOOT_DRIVE_TYPE_NVME    4
#define BOOT_DRIVE_TYPE_VIRTIO  5

#define BOOT_STORAGE_IDENTITY_VERSION 1u
#define BOOT_STORAGE_FLAG_PCI_VALID   (1u << 0)
#define BOOT_PROFILE_NAME_MAX 32u

typedef struct {
    char name[BOOT_PROFILE_NAME_MAX];
    uint64_t start_ns;
    uint64_t duration_ns;
} boot_profile_entry_t;

typedef struct {
    UINT32 Type;
    UINT32 Pad;
    uint64_t PhysicalStart;
    uint64_t VirtualStart;
    uint64_t NumberOfPages;
    uint64_t Attribute;
} EFI_MEMORY_DESCRIPTOR;

typedef struct {
    char Name[LOADED_FILE_NAME_MAX];
    EFI_PHYSICAL_ADDRESS PhysAddr;
    uint64_t Size;
} LOADED_FILE;

#pragma pack(push, 1)
typedef struct {
    void               *MemoryMap;
    UINTN                  MemoryMapSize;
    UINTN                  MemoryMapDescriptorSize;
    UINT32                 MemoryMapDescriptorVersion;

    uint64_t FrameBufferBase;
    uint64_t FrameBufferSize;
    UINT32   HorizontalResolution;
    UINT32   VerticalResolution;
    UINT32   PixelsPerScanLine;

    uint64_t  PartitionStartLBA;
    uint64_t AcpiRsdpAddress;
    UINT32   AcpiRsdpSize;
    UINT32   AcpiRsdpRevision;

    UINT32   BootDriveType;

    uint64_t RamDiskBase;
    uint64_t RamDiskSize;
    UINT32   RamDiskSectorSize;
    UINT32   RamDiskReserved;

    UINTN       LoadedFileCount;
    LOADED_FILE LoadedFiles[MAX_LOADED_FILES];

    uint64_t FontDataAddress;
    uint64_t FontDataSize;

    UINT32   BootStorageIdentityVersion;
    UINT32   BootStorageIdentityFlags;
    uint16_t BootStoragePciSegment;
    uint8_t  BootStoragePciBus;
    uint8_t  BootStoragePciDevice;
    uint8_t  BootStoragePciFunction;
    uint8_t  BootStorageTransport;
    uint16_t BootStoragePort;
    UINT32   BootStorageNamespace;
    uint64_t BootStoragePartitionLBA;

    uint8_t  KaslrEnabled;
    uint8_t  KaslrReserved[7];
    uint64_t KernelSlide;
    uint64_t KernelVirtBase;
    uint64_t KernelPhysBase;
} BOOT_INFO;
#pragma pack(pop)

__attribute__((noreturn))
void kernel_main(BOOT_INFO *boot_info);

#endif
