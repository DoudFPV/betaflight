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

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

static SPI_InitTypeDef defaultInit = {
    .SPI_Mode = SPI_Mode_Master,
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
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

    return (ffs(divisor) - 2) << 3; // SPI_CR1_BR_Pos
}

static void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
    const uint16_t tempRegister = (instance->CR1 & ~BR_BITS);
    instance->CR1 = tempRegister | spiDivisorToBRbits(instance, divisor);
#undef BR_BITS
}

static void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    SPI_Cmd(instance, DISABLE);
    spiSetDivisorBRreg(instance, divisor);
    SPI_Cmd(instance, ENABLE);
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

#if defined(STM32F3) || defined(STM32F4)
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
#elif defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);
#else
#error Undefined MCU architecture
#endif

    // Init SPI hardware
    SPI_I2S_DeInit(spi->dev);

#ifndef USE_SPI_TRANSACTION
    if (spi->leadingEdge) {
        defaultInit.SPI_CPOL = SPI_CPOL_Low;
        defaultInit.SPI_CPHA = SPI_CPHA_1Edge;
    } else
#endif
    {
        defaultInit.SPI_CPOL = SPI_CPOL_High;
        defaultInit.SPI_CPHA = SPI_CPHA_2Edge;
    }

#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(spi->dev, SPI_RxFIFOThreshold_QF);
#endif

    SPI_I2S_DMACmd(spi->dev, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_Init(spi->dev, &defaultInit);
    SPI_Cmd(spi->dev, ENABLE);
}


void spiResetStream(dmaChannelDescriptor_t *descriptor)
{
    // Disable the stream
    // Disable the DMA engine and SPI interface
    xDMA_Cmd(descriptor->dma, DISABLE);

    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}


static bool spiPrivReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    uint8_t b;
    DISCARD(instance->DR);
    while (len--) {
        b = txData ? *(txData++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET);
#ifdef STM32F303xC
        SPI_SendData8(instance, b);
#else
        SPI_I2S_SendData(instance, b);
#endif

        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET);
#ifdef STM32F303xC
        b = SPI_ReceiveData8(instance);
#else
        b = SPI_I2S_ReceiveData(instance);
#endif
        if (rxData)
            *(rxData++) = b;
    }

    return true;
}

void spiPrivInitStream(extDevice_t *dev, bool preInit)
{
    static uint8_t dummyTxByte = 0xff;
    static uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    volatile busSegment_t *segment = bus->curSegment;

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

    DMA_InitTypeDef *initTx = bus->initTx;
    DMA_InitTypeDef *initRx = bus->initRx;

    DMA_StructInit(initTx);
    DMA_StructInit(initRx);

#ifdef STM32F4
    initTx->DMA_Channel = bus->dmaTxChannel;
    initTx->DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
    initTx->DMA_M2M = DMA_M2M_Disable;
    initTx->DMA_DIR = DMA_DIR_PeripheralDST;
#endif
    initTx->DMA_Mode = DMA_Mode_Normal;
    initTx->DMA_PeripheralBaseAddr = (uint32_t)&bus->busType_u.spi.instance->DR;
    initTx->DMA_Priority = DMA_Priority_Low;
    initTx->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    initTx->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    if (txData) {
#ifdef STM32F4
        initTx->DMA_Memory0BaseAddr = (uint32_t)txData;
#else
        initTx->DMA_MemoryBaseAddr = (uint32_t)txData;
#endif
        initTx->DMA_MemoryInc = DMA_MemoryInc_Enable;
    } else {
        dummyTxByte = 0xff;
#ifdef STM32F4
        initTx->DMA_Memory0BaseAddr = (uint32_t)&dummyTxByte;
#else
        initTx->DMA_MemoryBaseAddr = (uint32_t)&dummyTxByte;
#endif
        initTx->DMA_MemoryInc = DMA_MemoryInc_Disable;
    }
    initTx->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    initTx->DMA_BufferSize = len;

#ifdef STM32F4
    initRx->DMA_Channel = bus->dmaRxChannel;
    initRx->DMA_DIR = DMA_DIR_PeripheralToMemory;
#else
    initRx->DMA_M2M = DMA_M2M_Disable;
    initRx->DMA_DIR = DMA_DIR_PeripheralSRC;
#endif
    initRx->DMA_Mode = DMA_Mode_Normal;
    initRx->DMA_PeripheralBaseAddr = (uint32_t)&bus->busType_u.spi.instance->DR;
    initRx->DMA_Priority = DMA_Priority_Low;
    initRx->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    initRx->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    if (rxData) {
#ifdef STM32F4
        initRx->DMA_Memory0BaseAddr = (uint32_t)rxData;
#else
        initRx->DMA_MemoryBaseAddr = (uint32_t)rxData;
#endif
        initRx->DMA_MemoryInc = DMA_MemoryInc_Enable;
    } else {
#ifdef STM32F4
        initRx->DMA_Memory0BaseAddr = (uint32_t)&dummyRxByte;
#else
        initRx->DMA_MemoryBaseAddr = (uint32_t)&dummyRxByte;
#endif
        initRx->DMA_MemoryInc = DMA_MemoryInc_Disable;
    }
    initRx->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    initRx->DMA_BufferSize = len;
}

void spiPrivStartDMA(extDevice_t *dev)
{
    // Assert Chip Select
    IOLo(dev->busType_u.spi.csnPin);

    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
    DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

    // Use the correct callback argument
    dmaRx->userParam = (uint32_t)dev;

    // Clear transfer flags
    DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    // Disable streams to enable update
    streamRegsTx->CR = 0U;
    streamRegsRx->CR = 0U;

    /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
     * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
     */
    DMA_ITConfig(streamRegsRx, DMA_IT_TC, ENABLE);

    // Update streams
    DMA_Init(streamRegsTx, dev->bus->initTx);
    DMA_Init(streamRegsRx, dev->bus->initRx);

    /* Note from AN4031
     *
     * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
     * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
     * the first required data to the peripheral (in case of memory-to-peripheral transfer).
     */

    // Enable streams
    DMA_Cmd(streamRegsTx, ENABLE);
    DMA_Cmd(streamRegsRx, ENABLE);

    /* Enable the SPI DMA Tx & Rx requests */
    SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}


void spiPrivStopDMA (extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    SPI_TypeDef *instance = dev->bus->busType_u.spi.instance;

    // Disable the DMA engine and SPI interface
    xDMA_Cmd(dmaRx->dma, DISABLE);
    xDMA_Cmd(dmaTx->dma, DISABLE);

    DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
}


// DMA transfer setup and start
void spiSequence(extDevice_t *dev, busSegment_t *segments, bool tryDMA)
{
    dev->bus->initSegment = true;
    dev->bus->curSegment = segments;
    busDevice_t *bus = dev->bus;

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
