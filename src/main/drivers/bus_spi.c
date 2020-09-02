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
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma_reqmap.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#include "nvic.h"

static uint8_t spiRegisteredDeviceCount = 0;

spiDevice_t spiDevice[SPIDEV_COUNT];
busDevice_t spiBusDevice[SPIDEV_COUNT];

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1)
        return SPIDEV_1;
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif

    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }

    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device, bool leadingEdge)
{
    switch (device) {
    case SPIINVALID:
        return false;

    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_5:
#if defined(USE_SPI_DEVICE_5)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif

    case SPIDEV_6:
#if defined(USE_SPI_DEVICE_6)
        spiInitDevice(device, leadingEdge);
        return true;
#else
        break;
#endif
    }
    return false;
}

// Return true if DMA engine is busy
bool spiIsBusy(extDevice_t *dev)
{
    return (dev->bus->curSegment != (busSegment_t *)NULL);
}


// Wait for DMA completion
void spiWait(extDevice_t *dev)
{
    // Wait for completion
    while (dev->bus->curSegment != (busSegment_t *)NULL);
}

// Wait for bus to become free, then read/write block of data
void spiReadWriteBuf(extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {txData, rxData, len, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], STACK_DMA_OK);

    spiWait(dev);
}

// Read/Write a block of data, returning false if the bus is busy
bool spiReadWriteBufRB(extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiReadWriteBuf(dev, txData, rxData, length);

    return true;
}

// Wait for bus to become free, then read/write a single byte
uint8_t spiReadWrite(extDevice_t *dev, uint8_t data)
{
    uint8_t retval;

    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&data, &retval, sizeof (data), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], false);

    spiWait(dev);

    return retval;
}

// Wait for bus to become free, then read/write a single byte from a register
uint8_t spiReadWriteReg(extDevice_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t retval;

    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&reg, NULL, sizeof (reg), false, NULL},
            {&data, &retval, sizeof (data), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], false);

    spiWait(dev);

    return retval;
}

// Wait for bus to become free, then write a single byte
void spiWrite(extDevice_t *dev, uint8_t data)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&data, NULL, sizeof (data), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], false);

    spiWait(dev);
}

// Write data to a register
void spiWriteReg(extDevice_t *dev, uint8_t reg, uint8_t data)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&reg, NULL, sizeof (reg), false, NULL},
            {&data, NULL, sizeof (data), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], false);

    spiWait(dev);
}

// Write data to a register, returning false if the bus is busy
bool spiWriteRegRB(extDevice_t *dev, uint8_t reg, uint8_t data)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiWriteReg(dev, reg, data);

    return true;
}

// Read a block of data from a register
void spiReadRegBuf(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&reg, NULL, sizeof (reg), false, NULL},
            {NULL, data, length, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], STACK_DMA_OK);

    spiWait(dev);
}

// Read a block of data from a register, returning false if the bus is busy
bool spiReadRegBufRB(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiReadRegBuf(dev, reg, data, length);

    return true;
}

// Read a block of data where the register is ORed with 0x80, returning false if the bus is busy
bool spiReadRegMskBufRB(extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegBufRB(dev, reg | 0x80, data, length);
}

// Wait for bus to become free, then write a block of data to a register
void spiWriteRegBuf(extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&reg, NULL, sizeof (reg), false, NULL},
            {data, NULL, length, true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], STACK_DMA_OK);

    spiWait(dev);
}

// Wait for bus to become free, then read a byte from a register
uint8_t spiReadReg(extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&reg, NULL, sizeof (reg), false, NULL},
            {NULL, &data, sizeof (data), true, NULL},
            {NULL, NULL, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(dev);

    spiSequence(dev, &segments[0], false);

    spiWait(dev);

    return data;
}

// Wait for bus to become free, then read a block of data where the register is ORed with 0x80
uint8_t spiReadRegMsk(extDevice_t *dev, uint8_t reg)
{
    return spiReadReg(dev, reg | 0x80);
}

// Interrupt handler for SPI receive DMA completion
static void spiRxIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    extDevice_t *dev = (extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        // Negate Chip Select
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiPrivStopDMA(dev);

#ifdef __DCACHE_PRESENT
    if (bus->curSegment->rxData) {
         // Invalidate the D cache covering the area into which data has been read
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)((uint32_t)bus->curSegment->rxData & ~CACHE_LINE_MASK),
            (((uint32_t)bus->curSegment->rxData & CACHE_LINE_MASK) +
              bus->curSegment->len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
    }
#endif // __DCACHE_PRESENT

    if (bus->curSegment->callback) {
        switch(bus->curSegment->callback(dev->callbackArg)) {
        case BUS_BUSY:
            // Repeat the last DMA segment
            bus->curSegment--;
            // Reinitialise the cached init values as segment is not progressing
            spiPrivInitStream(dev, true);
            break;

        case BUS_ABORT:
            bus->curSegment = (busSegment_t *)NULL;
            return;

        case BUS_READY:
        default:
            // Advance to the next DMA segment
            break;
        }
    }

    // Advance through the segment list
    bus->curSegment++;

    if (bus->curSegment->len == 0) {
        // The end of the segment list has been reached, so mark transactions as complete
        bus->curSegment = (busSegment_t *)NULL;
    } else {
        // After the completion of the first segment setup the init structure for the subsequent segment
        if (bus->initSegment) {
            spiPrivInitStream(dev, false);
            bus->initSegment = false;
        }

        // Launch the next transfer
        spiPrivStartDMA(dev);

        // Prepare the init structures ready for the next segment to reduce inter-segment time
        spiPrivInitStream(dev, true);
    }
}


bool spiSetBusInstance(extDevice_t *dev, uint32_t device, resourceOwner_e owner)
{
    if (device > SPIDEV_COUNT) {
        return false;
    }

    dev->bus = &spiBusDevice[SPI_CFG_TO_DEV(device)];

    if (dev->bus->busType == BUSTYPE_SPI) {
        // This bus has already been initialised
        return true;
    }

    busDevice_t *bus = dev->bus;

    bus->busType = BUSTYPE_SPI;
    bus->busType_u.spi.instance = spiInstanceByDevice(SPI_CFG_TO_DEV(device));
    bus->useDMA = false;
    bus->owner = owner;
    bus->initTx = &dev->initTx;
    bus->initRx = &dev->initRx;


    return true;
}

void spiInitBusDMA()
{
    uint32_t device;

    for (device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUSTYPE_SPI) {
            // This bus is not in use
            continue;
        }

        dmaIdentifier_e dmaTxIdentifier = DMA_NONE;
        dmaIdentifier_e dmaRxIdentifier = DMA_NONE;

        for (uint8_t opt = 0; opt < MAX_PERIPHERAL_DMA_OPTIONS; opt++) {
            const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_TX,
                                                                                   device,
                                                                                   opt);

            if (dmaChannelSpec) {
                dmaTxIdentifier = dmaGetIdentifier(dmaChannelSpec->ref);
                bus->dmaTxChannel = dmaChannelSpec->channel;
                break;
            }
        }

        for (uint8_t opt = 0; opt < MAX_PERIPHERAL_DMA_OPTIONS; opt++) {
            const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_RX,
                                                                                   device,
                                                                                   opt);

            if (dmaChannelSpec) {
                dmaRxIdentifier = dmaGetIdentifier(dmaChannelSpec->ref);
                bus->dmaRxChannel = dmaChannelSpec->channel;
                break;
            }
        }

        if (dmaTxIdentifier && dmaRxIdentifier) {
            bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
            bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);

            // Ensure streams are disabled
            spiResetStream(bus->dmaRx);
            spiResetStream(bus->dmaTx);

            // Register the first device to use the bus. It may subsequently be shared
            dmaInit(dmaTxIdentifier, bus->owner, 0);
            dmaInit(dmaRxIdentifier, bus->owner, 0);

            /* Note that this driver may be called both from the normal thread of execution, or from USB interrupt
             * handlers, so the DMA completion interrupt must be at a higher priority
             */
            dmaSetHandler(dmaRxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);

            bus->useDMA = true;
        }
    }
}

void spiSetClkDivisor(extDevice_t *dev, uint16_t divisor)
{
    dev->busType_u.spi.speed = divisor;
}

void spiNegateCS(extDevice_t *dev)
{
    // Negate Chip Select
    IOHi(dev->busType_u.spi.csnPin);
}

bool spiUseDMA(extDevice_t *dev)
{
    return dev->bus->useDMA;
}

void spiBusDeviceRegister(extDevice_t *dev)
{
    UNUSED(dev);

    spiRegisteredDeviceCount++;
}

uint8_t spiGetRegisteredDeviceCount(void)
{
    return spiRegisteredDeviceCount;
}
#endif
