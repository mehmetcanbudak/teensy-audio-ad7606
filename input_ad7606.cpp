/* This file is based on input_tdm from
 *   https://github.com/PaulStoffregen/Audio
 *
 * Audio Library for Teensy 3.X
 * Copyright (c) 2017, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Arduino.h>
#include "input_ad7606.h"
#include "memcpy_audio.h"
#include "utility/imxrt_hw.h"
#include <SPI.h>
#include <imxrt.h>
#include <cstdint>
#include <DMAChannel.h>

#define AD7607_BUSY 3
#define AD7607_START_CONVERSION 5
#define AD7607_CHIP_SELECT 36
#define AD7607_RESET 4

int8_t buf[25];
uint16_t txbuf[13];
audio_block_t * AudioInputAD7606::block_incoming[8] = {
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
};
bool AudioInputAD7606::update_responsibility = false;
DMAChannel AudioInputAD7606::dmarx(false);
DMAChannel AudioInputAD7606::dmatx(false);
uint8_t AudioInputAD7606::index = 0;
uint8_t AudioInputAD7606::bytesToRead = 25;         // Only need 16 bytes, but 25 bytes allow us to get more regular samples (whn SPI SCK is 14Mhz)

void AudioInputAD7606::busyFallingEdgeISR() {
    if (index < 128) {
        dmarx.destinationBuffer(buf, bytesToRead);
        dmarx.transferCount(bytesToRead);
        dmarx.transferSize(1);

        dmatx.sourceBuffer(txbuf, bytesToRead);
        dmatx.transferCount(bytesToRead);
        dmatx.transferSize(1);

        digitalWrite(AD7607_CHIP_SELECT, LOW);

        IMXRT_LPSPI1_S.TCR =
                (IMXRT_LPSPI1_S.TCR & ~(LPSPI_TCR_FRAMESZ(31))) | LPSPI_TCR_FRAMESZ(7); // Transmit Control Register: ?
        IMXRT_LPSPI1_S.FCR = 0; // FIFO control register

        IMXRT_LPSPI1_S.DER = LPSPI_DER_RDDE | LPSPI_DER_TDDE;//DMA Enable register: enable DMA on RX
        IMXRT_LPSPI1_S.SR = 0x3f00; // StatusRegister: clear out all of the other status...

        SPI2.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
        dmarx.enable();
        dmatx.enable();
    };
}

void AudioInputAD7606::begin(void)
{
    pinMode(AD7607_BUSY, INPUT_PULLUP);
    pinMode(AD7607_START_CONVERSION, OUTPUT);
    pinMode(AD7607_CHIP_SELECT, OUTPUT);
    pinMode(AD7607_RESET, OUTPUT);
    digitalWrite(AD7607_RESET, LOW);
    digitalWrite(AD7607_CHIP_SELECT, HIGH);

    attachInterrupt(AD7607_BUSY, busyFallingEdgeISR, FALLING);

    SPI2.setSCK(49);        // Using SPI2 SCK on pins on underside of teensy 4.1
    SPI2.setMISO(54);       // Using SPI2 MISO on pins on underside of teensy 4.1
    SPI2.begin();           // Start SPI2

    dmatx.begin(true); // allocate the DMA channel first
    dmatx.destination((volatile uint8_t &)IMXRT_LPSPI1_S.TDR);
    dmatx.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI1_TX);
    dmatx.disableOnCompletion();


    dmarx.begin(true); // allocate the DMA channel first
    dmarx.source((volatile uint8_t &)IMXRT_LPSPI1_S.RDR);
    dmarx.interruptAtCompletion();
    dmarx.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI1_RX);
    dmarx.disableOnCompletion();
    update_responsibility = update_setup();
    dmarx.attachInterrupt(isr);

    digitalWrite(AD7607_START_CONVERSION, LOW);
    digitalWrite(AD7607_START_CONVERSION, HIGH);
}

void AudioInputAD7606::isr(void)
{
    dmarx.clearInterrupt();
    dmarx.disable();
    dmatx.disable();

    digitalWrite(AD7607_CHIP_SELECT, HIGH);

    SPI2.endTransaction();
    if (index < 128) {
        for (int i = 0; i < 8; i++) {
            if (block_incoming[i] != NULL)
                block_incoming[i]->data[index] = (buf[i * 2] << 8) + buf[(i*2)+1];
        }
        index++;

        digitalWrite(AD7607_START_CONVERSION, LOW);
        digitalWrite(AD7607_START_CONVERSION, HIGH);
    };
}

void AudioInputAD7606::update(void)
{
    unsigned int i, j;
    audio_block_t *new_block[8];
    audio_block_t *out_block[8];

    // allocate 8 new blocks.  If any fails, allocate none
    for (i=0; i < 8; i++) {
        new_block[i] = allocate();
        if (new_block[i] == nullptr) {
            for (j=0; j < i; j++) {
                release(new_block[j]);
            }
            memset(new_block, 0, sizeof(new_block));
            break;
        }
    }
    __disable_irq();
    memcpy(out_block, block_incoming, sizeof(out_block));
    memcpy(block_incoming, new_block, sizeof(block_incoming));
    __enable_irq();
    if (out_block[0] != nullptr) {
        // if we got 1 block, all 16 are filled
        for (i=0; i < 8; i++) {
            transmit(out_block[i], i);
            release(out_block[i]);
        }
    }
    index = 0;
    digitalWrite(AD7607_START_CONVERSION, LOW);
    digitalWrite(AD7607_START_CONVERSION, HIGH);
}

void AudioInputAD7606::timingFix(bool enabled) {
    if (enabled) {
        // reading extra bytes, and ignoring the extra bytes,
        // means the audio library buffer update takes about
        // the same amount of time to read as the samples are read,
        // making the read rate much more regular
        bytesToRead = 25;
    }
    else {
        // only read 16 bytes.
        // reading one buffer of 8 bytes * 2 bytes/sample * 128 samples
        // takes much less time than the audio buffer callback occurs...
        // basically, the buffer will fill with samples very quickly
        // and then wait for the audio callback to fill the buffer
        // so the sample rate is very irregular
        bytesToRead = 16;
    }
}
