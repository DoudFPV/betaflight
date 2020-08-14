/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_M25P16

#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "flash_m25p16.h"

#define M25P16_INSTRUCTION_RDID             SPIFLASH_INSTRUCTION_RDID
#define M25P16_INSTRUCTION_READ_BYTES       0x03
#define M25P16_INSTRUCTION_READ_STATUS_REG  0x05
#define M25P16_INSTRUCTION_WRITE_STATUS_REG 0x01
#define M25P16_INSTRUCTION_WRITE_ENABLE     0x06
#define M25P16_INSTRUCTION_WRITE_DISABLE    0x04
#define M25P16_INSTRUCTION_PAGE_PROGRAM     0x02
#define M25P16_INSTRUCTION_SECTOR_ERASE     0xD8
#define M25P16_INSTRUCTION_BULK_ERASE       0xC7

#define M25P16_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define M25P16_STATUS_FLAG_WRITE_ENABLED     0x02

#define W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE 0xB7

// Format is manufacturer, memory type, then capacity
// See also flash_m25p16.h for additional JEDEC IDs.
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20ba18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q128_DTR   0xEF7018
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018
#define JEDEC_ID_BERGMICRO_W25Q32      0xE04016

#define M25P16_PAGESIZE 256

STATIC_ASSERT(M25P16_PAGESIZE < FLASH_MAX_PAGE_SIZE, M25P16_PAGESIZE_too_small);

const flashVTable_t m25p16_vTable;


static uint8_t m25p16_readStatus(flashDevice_t *fdevice)
{
    // This routine blocks so no need to use static data
    static uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    static uint8_t readyStatus[2];

    spiReadWriteBuf(fdevice->io.handle.dev, readStatus, readyStatus, sizeof (readStatus));

    return readyStatus[1];
}


static bool m25p16_isReady(flashDevice_t *fdevice)
{
    // If we're waiting on DMA completion, then SPI is busy
    if (fdevice->io.handle.dev->bus->useDMA && spiIsBusy(fdevice->io.handle.dev)) {
        return false;
    }

    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    if (!fdevice->couldBeBusy) {
        return true;
    }

    // Poll the FLASH device to see if it's busy
    fdevice->couldBeBusy = ((m25p16_readStatus(fdevice) & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !fdevice->couldBeBusy;
}

static bool m25p16_waitForReady(flashDevice_t *fdevice)
{
    while (!m25p16_isReady(fdevice));

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */

bool m25p16_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    // Default SPI clock speed
    uint16_t spiSpeed = SPI_CLOCK_ULTRAFAST;
    flashGeometry_t *geometry = &fdevice->geometry;

    geometry->pagesPerSector = 256;

    switch (chipID) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        geometry->sectors = 32;
        spiSpeed = SPI_CLOCK_FAST;
        break;
    case JEDEC_ID_BERGMICRO_W25Q32:
        geometry->sectors = 1024;
        geometry->pagesPerSector = 16;
        spiSpeed = SPI_CLOCK_FAST;
        break;
    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        geometry->sectors = 64;
        break;
    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        geometry->sectors = 128;
        break;
    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_DTR:
    case JEDEC_ID_CYPRESS_S25FL128L:
        geometry->sectors = 256;
        break;
    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        geometry->sectors = 512;
        break;
    default:
        // Unsupported chip or not an SPI NOR flash
        geometry->sectors = 0;
        geometry->pagesPerSector = 0;
        geometry->sectorSize = 0;
        geometry->totalSize = 0;
        return false;
    }

    geometry->flashType = FLASH_TYPE_NOR;
    geometry->pageSize = M25P16_PAGESIZE;
    geometry->sectorSize = geometry->pagesPerSector * geometry->pageSize;
    geometry->totalSize = geometry->sectorSize * geometry->sectors;

    // Adjust the SPI bus clock frequency
#ifndef FLASH_SPI_SHARED
    spiSetClkDivisor(fdevice->io.handle.dev, spiSpeed);
#endif

    if (geometry->totalSize > 16 * 1024 * 1024) {
        fdevice->isLargeFlash = true;

        // This routine blocks so no need to use static data
        uint8_t modeSet[] = { W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE };

        spiReadWriteBuf(fdevice->io.handle.dev, modeSet, NULL, sizeof (modeSet));
    }

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &m25p16_vTable;
    return true;
}

static void m25p16_setCommandAddress(uint8_t *buf, uint32_t address, bool useLongAddress)
{
    if (useLongAddress) {
        *buf++ = (address >> 24) & 0xff;
    }
    *buf++ = (address >> 16) & 0xff;
    *buf++ = (address >> 8) & 0xff;
    *buf = address & 0xff;
}


// Called in ISR context
// A write enable has just been issued
busStatus_e m25p16_callbackWriteEnable(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    // As a write has just occurred, the device could be busy
    fdevice->couldBeBusy = true;

    return BUS_READY;
}

// Called in ISR context
// Write operation has just completed
busStatus_e m25p16_callbackWriteComplete(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    // Call transfer completion callback
    if (fdevice->callback) {
        fdevice->callback(fdevice->callbackArg);
    }

    return BUS_READY;
}

// Called in ISR context
// Check if the status was busy and if so repeat the poll
busStatus_e m25p16_callbackReady(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    uint8_t readyPoll = dev->bus->curSegment->rxData[1];

    if (readyPoll & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) {
        return BUS_BUSY;
    }

    // Bus is now known not to be busy
    fdevice->couldBeBusy = false;

    return BUS_READY;
}


/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void m25p16_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    uint8_t sectorErase[5] = { M25P16_INSTRUCTION_SECTOR_ERASE };
    uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    uint8_t readyStatus[2];
    uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };
    busSegment_t segments[] = {
            {readStatus, readyStatus, sizeof (readStatus), true, m25p16_callbackReady},
            {writeEnable, NULL, sizeof (writeEnable), true, m25p16_callbackWriteEnable},
            {sectorErase, NULL, fdevice->isLargeFlash ? 5 : 4, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    m25p16_setCommandAddress(&sectorErase[1], address, fdevice->isLargeFlash);

    spiSequence(fdevice->io.handle.dev, &segments[0], STACK_DMA_OK);

    // Block pending completion of SPI access, but the erase will be ongoing
    spiWait(fdevice->io.handle.dev);
}

static void m25p16_eraseCompletely(flashDevice_t *fdevice)
{
    uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    uint8_t readyStatus[2];
    uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };
    uint8_t bulkErase[] = { M25P16_INSTRUCTION_BULK_ERASE };
    busSegment_t segments[] = {
            {readStatus, readyStatus, sizeof (readStatus), true, m25p16_callbackReady},
            {writeEnable, NULL, sizeof (writeEnable), true, m25p16_callbackWriteEnable},
            {bulkErase, NULL, sizeof (bulkErase), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    spiSequence(fdevice->io.handle.dev, &segments[0], STACK_DMA_OK);

    // Block pending completion of SPI access, but the erase will be ongoing
    spiWait(fdevice->io.handle.dev);
}


static void m25p16_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    UNUSED(fdevice);

    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}


static void m25p16_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    // The segment list cannot be in automatic storage as this routine is non-blocking
    static uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    static uint8_t readyStatus[2];
    static uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };
    static uint8_t pageProgram[5] = { M25P16_INSTRUCTION_PAGE_PROGRAM };
    static busSegment_t segments[] = {
            {readStatus, readyStatus, sizeof (readStatus), true, m25p16_callbackReady},
            {writeEnable, NULL, sizeof (writeEnable), true, m25p16_callbackWriteEnable},
            {pageProgram, NULL, 0, false, NULL},
            {NULL, NULL, 0, true, NULL},
            {NULL, NULL, 0, true, NULL},
            {NULL, NULL, 0, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    // Patch the pageProgram segment
    segments[2].len = fdevice->isLargeFlash ? 5 : 4;
    m25p16_setCommandAddress(&pageProgram[1], fdevice->currentWriteAddress, fdevice->isLargeFlash);

    // Patch the data segments
    // Handle a maximum of three buffers, silently dropping any more
    // Accumulate the lengths of the segments
    fdevice->callbackArg = 0;
    uint32_t i, n;

    for (i = 0, n = 0; i < 4; i++)
    {
        if ((bufferSizes[i] == 0) && (i < bufferCount - 1)) {
            // There may be three buffers, with data only in the first and third
            continue;
        }
        if ((i < bufferCount) && (bufferSizes[i] > 0)) {
            segments[3 + n].txData = (uint8_t *)buffers[i];
            segments[3 + n].len = bufferSizes[i];
            // Send consecutive blocks with no intervening CS negation
            segments[3 + n].negateCS = false;
            segments[2 + n].callback = NULL;
            fdevice->currentWriteAddress += bufferSizes[i];
            fdevice->callbackArg += bufferSizes[i];
        } else {
            // Mark final data segment to negate CS and callback to indicate transfer end
            segments[2 + n].negateCS = true;
            segments[2 + n].callback = m25p16_callbackWriteComplete;
            // Mark segment following data as being of zero length
            segments[3 + n].len = 0;
            break;
        }
        n++;
    }

    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[0] : &segments[1], true);

    if (fdevice->callback == NULL) {
        // No callback was provided so block
        spiWait(fdevice->io.handle.dev);
    }
}

static void m25p16_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
static void m25p16_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    m25p16_pageProgramBegin(fdevice, address, callback);

    m25p16_pageProgramContinue(fdevice, &data, &length, 1);

    m25p16_pageProgramFinish(fdevice);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int m25p16_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    static uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    static uint8_t readyStatus[2];
    static uint8_t readBytes[5] = { M25P16_INSTRUCTION_READ_BYTES };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    busSegment_t segments[] = {
            {readStatus, readyStatus, sizeof (readStatus), true, m25p16_callbackReady},
            {readBytes, NULL, fdevice->isLargeFlash ? 5 : 4, false, NULL},
            {NULL, buffer, length, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Patch the readBytes command
    m25p16_setCommandAddress(&readBytes[1], address, fdevice->isLargeFlash);

    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[0] : &segments[1], true);

    // Block until code is re-factored to exploit non-blocking
    spiWait(fdevice->io.handle.dev);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* m25p16_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t m25p16_vTable = {
    .isReady = m25p16_isReady,
    .waitForReady = m25p16_waitForReady,
    .eraseSector = m25p16_eraseSector,
    .eraseCompletely = m25p16_eraseCompletely,
    .pageProgramBegin = m25p16_pageProgramBegin,
    .pageProgramContinue = m25p16_pageProgramContinue,
    .pageProgramFinish = m25p16_pageProgramFinish,
    .pageProgram = m25p16_pageProgram,
    .readBytes = m25p16_readBytes,
    .getGeometry = m25p16_getGeometry,
};
#endif
