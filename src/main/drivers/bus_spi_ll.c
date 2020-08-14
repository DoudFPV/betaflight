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

#if defined(USE_SPI)

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_NSS_PIN    PA15
#define SPI4_SCK_PIN    PB3
#define SPI4_MISO_PIN   PB4
#define SPI4_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif
#ifndef SPI4_NSS_PIN
#define SPI4_NSS_PIN NONE
#endif

#define SPI_DEFAULT_TIMEOUT 10

#define IS_DTCM(p) (((uint32_t)p & 0xffff0000) == 0x20000000)

static LL_SPI_InitTypeDef defaultInit =
{
    .TransferDirection = SPI_DIRECTION_2LINES,
    .Mode = SPI_MODE_MASTER,
    .DataWidth = SPI_DATASIZE_8BIT,
    .NSS = SPI_NSS_SOFT,
    .BaudRate = SPI_BAUDRATEPRESCALER_8,
    .BitOrder = SPI_FIRSTBIT_MSB,
    .CRCPoly = 7,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
};


static uint16_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
#if !(defined(STM32F1) || defined(STM32F3))
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.

    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#else
    UNUSED(instance);
#endif

    divisor = constrain(divisor, 2, 256);

    return (ffs(divisor) - 2) << SPI_CR1_BR_Pos;
}

static void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    LL_SPI_Disable(instance);
    LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, divisor));
    LL_SPI_Enable(instance);
}

void spiInitDevice(SPIDevice device, bool leadingEdge)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

#ifndef USE_SPI_TRANSACTION
    spi->leadingEdge = leadingEdge;
#else
    UNUSED(leadingEdge);
#endif

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

    if (spi->leadingEdge == true)
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
    else
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);

    LL_SPI_Disable(spi->dev);
    LL_SPI_DeInit(spi->dev);

#ifndef USE_SPI_TRANSACTION
    if (spi->leadingEdge) {
        defaultInit.ClockPolarity = SPI_POLARITY_LOW;
        defaultInit.ClockPhase = SPI_PHASE_1EDGE;
    } else
#endif
    {
        defaultInit.ClockPolarity = SPI_POLARITY_HIGH;
        defaultInit.ClockPhase = SPI_PHASE_2EDGE;
    }

    LL_SPI_SetRxFIFOThreshold(spi->dev, SPI_RXFIFO_THRESHOLD_QF);

    LL_SPI_Init(spi->dev, &defaultInit);
    LL_SPI_Enable(spi->dev);
}


void spiResetStream(dmaChannelDescriptor_t *descriptor)
{
    // Disable the stream
    LL_DMA_DisableStream(descriptor->dma, descriptor->stream);
    while (LL_DMA_IsEnabledStream(descriptor->dma, descriptor->stream));

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}


static bool spiPrivReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    // set 16-bit transfer
    CLEAR_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    while (len > 1) {
        while (!LL_SPI_IsActiveFlag_TXE(instance));
        uint16_t w;
        if (txData) {
            w = *((uint16_t *)txData);
            txData += 2;
        } else {
            w = 0xFFFF;
        }
        LL_SPI_TransmitData16(instance, w);

        while (!LL_SPI_IsActiveFlag_RXNE(instance));
        w = LL_SPI_ReceiveData16(instance);
        if (rxData) {
            *((uint16_t *)rxData) = w;
            rxData += 2;
        }
        len -= 2;
    }
    // set 8-bit transfer
    SET_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    if (len) {
        while (!LL_SPI_IsActiveFlag_TXE(instance));
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        while (!LL_SPI_IsActiveFlag_RXNE(instance));
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }

    return true;
}

void spiPrivInitStream(extDevice_t *dev, bool preInit)
{
    static uint8_t dummyTxByte = 0xff;
    static uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    busSegment_t *segment = bus->curSegment;

    if (preInit) {
        // Prepare the init structure for the next segment to reduce inter-segment interval
        segment++;
        if(segment->len == 0) {
            // There's no following segment
            return;
        }
    }

    uint8_t *txData = segment->txData;
    uint8_t *rxData = segment->rxData;
    int len = segment->len;

    LL_DMA_InitTypeDef *initTx = bus->initTx;
    LL_DMA_InitTypeDef *initRx = bus->initRx;

    LL_DMA_StructInit(initTx);
    LL_DMA_StructInit(initRx);

    initTx->Channel = bus->dmaTxChannel;
    initTx->Mode = LL_DMA_MODE_NORMAL;
    initTx->Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    initTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
    initTx->Priority = LL_DMA_PRIORITY_LOW;
    initTx->PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    initTx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    if (txData) {
#ifdef __DCACHE_PRESENT
        // No need to flush DTCM memory
        if (!IS_DTCM(txData)) {
            // Flush the D cache to ensure the data to be written is in main memory
            SCB_CleanDCache_by_Addr(
                    (uint32_t *)((uint32_t)txData & ~CACHE_LINE_MASK),
                    (((uint32_t)txData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
        }
#endif // __DCACHE_PRESENT
        initTx->MemoryOrM2MDstAddress = (uint32_t)txData;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    } else {
        dummyTxByte = 0xff;
        initTx->MemoryOrM2MDstAddress = (uint32_t)&dummyTxByte;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
    }
    initTx->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    initTx->NbData = len;

    initRx->Channel = bus->dmaRxChannel;
    initRx->Mode = LL_DMA_MODE_NORMAL;
    initRx->Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    initRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
    initRx->Priority = LL_DMA_PRIORITY_LOW;
    initRx->PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    initRx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    if (rxData) {
        /* Flush the D cache for the start and end of the receive buffer as
         * the cache will be invalidated after the transfer and any valid data
         * just before/after must be in memory at that point
         */
#ifdef __DCACHE_PRESENT
        // No need to flush/invalidate DTCM memory
        if (!IS_DTCM(rxData)) {
            if (len > CACHE_LINE_SIZE) {
                // Flush the first line
                SCB_CleanInvalidateDCache_by_Addr(
                        (uint32_t *)((uint32_t)rxData & ~CACHE_LINE_MASK),
                        CACHE_LINE_SIZE);
                // Flush the last line
                SCB_CleanInvalidateDCache_by_Addr(
                        (uint32_t *)(((uint32_t)rxData + len - 1) & ~CACHE_LINE_MASK),
                        CACHE_LINE_SIZE);
            } else {
                // Start and end cache lines are either the same or contiguous
                SCB_CleanInvalidateDCache_by_Addr(
                        (uint32_t *)((uint32_t)rxData & ~CACHE_LINE_MASK),
                        (((uint32_t)rxData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
            }
        }
#endif // __DCACHE_PRESENT
        initRx->MemoryOrM2MDstAddress = (uint32_t)rxData;
        initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    } else {
        initRx->MemoryOrM2MDstAddress = (uint32_t)&dummyRxByte;
        initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
    }
    initRx->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    initRx->NbData = len;
}

void spiPrivStartDMA(extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    // Assert Chip Select
    IOLo(dev->busType_u.spi.csnPin);

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
    DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

    // Use the correct callback argument
    dmaRx->userParam = (uint32_t)dev;

    // Clear transfer flags
    DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    // Disable streams to enable update
    LL_DMA_WriteReg(streamRegsTx, CR, 0U);
    LL_DMA_WriteReg(streamRegsRx, CR, 0U);

    /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
     * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
     */
    LL_EX_DMA_EnableIT_TC(streamRegsRx);

    // Update streams
    LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);
    LL_DMA_Init(dmaRx->dma, dmaRx->stream, bus->initRx);

    /* Note from AN4031
     *
     * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
     * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
     * the first required data to the peripheral (in case of memory-to-peripheral transfer).
     */

    // Enable streams
    LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
    LL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);

    // Enable the SPI DMA Tx & Rx requests
    SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}

void spiPrivStopDMA (extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;

    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;

    // Disable the DMA engine and SPI interface
    LL_DMA_DisableStream(dmaRx->dma, dmaRx->stream);
    LL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);

    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    LL_SPI_DisableDMAReq_TX(instance);
    LL_SPI_DisableDMAReq_RX(instance);
}


// DMA transfer setup and start
void spiSequence(extDevice_t *dev, busSegment_t *segments, bool tryDMA)
{
    busDevice_t *bus = dev->bus;

    bus->initSegment = true;
    bus->curSegment = segments;

    // Switch bus speed
    spiSetDivisor(bus->busType_u.spi.instance, dev->busType_u.spi.speed);

    // TODO Make this decision more sophisticated. Need to determine the rules for when polled is
    // better than DMA driven. Data length, presence of a callback, number of segments may all need
    // to be considered.
    if (bus->useDMA && tryDMA && ((bus->curSegment->len > 7) || (bus->curSegment + 1)->len)) {
        // Intialise the init structures for the first transfer
        spiPrivInitStream(dev, false);

        // Start the transfers
        spiPrivStartDMA(dev);
    } else {
        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            // Assert Chip Select
            IOLo(dev->busType_u.spi.csnPin);

            spiPrivReadWriteBufPolled(
                    bus->busType_u.spi.instance,
                    bus->curSegment->txData,
                    bus->curSegment->rxData,
                    bus->curSegment->len);

            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            if (bus->curSegment->callback) {
                switch(bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    bus->curSegment--;
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
            bus->curSegment++;
        }

        bus->curSegment = (busSegment_t *)NULL;
    }
}
#endif
