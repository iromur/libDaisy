#include "daisy_rogue.h"

using namespace daisy;

// Define the port and pin for the Seed's user LED
#define SEED_LED_PORT DSY_GPIOC
#define SEED_LED_PIN 7

// We can use one of the Rogue's trigger pins for debugging
//#define DEBUG_OUT_PORT DSY_GPIOA
//#define DEBUG_OUT_PIN 5

// Initially, the audio buffer size will be hard-coded for now

#define MIX_BUFF_SAMPLES 64
#define AUDIO_DMA_SIZE (MIX_BUFF_SAMPLES * 8)
#define AUDIO_BUFF_SIZE (AUDIO_DMA_SIZE * 2)

// For the time being, these variables will be declared with global scope.
//  They need to be available to the SAI/DMA interrupt service routines.

DMA_HandleTypeDef hdma_sai2_blockB;
bool debugOutState;
bool gAudioIRQ1_ipFlag = false;
bool gAudioIRQ2_ipFlag = false;
int32_t DMA_BUFFER_MEM_SECTION gAudioOutBuff[AUDIO_BUFF_SIZE] __attribute__((aligned(32)));
float gDspBuffer[AUDIO_DMA_SIZE];
//dsy_gpio debugOut;

AudioHandle::TdmAudioCallback callback_;
AudioHandle::TdmInputBuffer   inBuf;
AudioHandle::TdmOutputBuffer  outBuf;

// ****************************************************************************
DaisyRogue::Result DaisyRogue::Init(bool boost)
{

    // Initialize the TDM audio buffer pointers. Because the PCM1682 is a DAC and
    //  has no inputs, we won't be using the input buffer.
    outBuf = (AudioHandle::TdmOutputBuffer)&gDspBuffer[0];
    inBuf  = nullptr;
    for(int i = 0; i < AUDIO_BUFF_SIZE; i++)
        gAudioOutBuff[i] = 0;

    // Initialize the Daisy Seed
    seed.Init(boost);

    // Not sure if there's a better way to control the seed's user LED
    //  but this will certainly work for now.
    seedLed.pin.port = SEED_LED_PORT;
    seedLed.pin.pin  = SEED_LED_PIN;
    seedLed.mode     = DSY_GPIO_MODE_OUTPUT_PP;
    dsy_gpio_init(&seedLed);

    //debugOut.pin.port = DEBUG_OUT_PORT;
    //debugOut.pin.pin  = DEBUG_OUT_PIN;
    //debugOut.mode     = DSY_GPIO_MODE_OUTPUT_PP;
    //dsy_gpio_init(&debugOut);
    //debugOutState = true;
    //dsy_gpio_write(&debugOut, true);

    InitMidi();
    return InitRogueAudioHardware();
 }

// ****************************************************************************
DaisyRogue::Result DaisyRogue::InitRogueAudioHardware(void) {

    // Because the initialization of SAI2 is already handled in libDaisy, and
    //  that code seems to only support I2S, I decided to put all of the code
    //  to set the hardware up for TDM here. At some point, perhaps someone
    //  with more knowledge of the inner workings of libDaisy can move the
    //  functionality to where it belongs.

    // DMA controller clock enable
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA1_Stream4_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    // Initialize SAI2 for TDM transmit only. The Daisy Rogue has a multi-channel
    //  DAC (output only) and the SAI2 input pin is used for something else.
    hsai_BlockB2.Instance = SAI2_Block_B;
    hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
    hsai_BlockB2.Init.DataSize = SAI_DATASIZE_32;
    hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
    hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
    hsai_BlockB2.Init.PdmInit.MicPairsNbr = 1;
    hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
    hsai_BlockB2.FrameInit.FrameLength = 256;
    hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
    hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
    hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
    hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
    hsai_BlockB2.SlotInit.FirstBitOffset = 0;
    hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
    hsai_BlockB2.SlotInit.SlotNumber = 8;
    hsai_BlockB2.SlotInit.SlotActive = 0x0000FFFF;
    if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
        return Result::ERR;

    // SAI2_B_Block_B GPIO Configuration
    // PG9     ------> SAI2_FS_B
    // PA1     ------> SAI2_MCLK_B
    // PA0     ------> SAI2_SD_B
    // PA2     ------> SAI2_SCK_B

    // Initialize the SAI2 GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_SAI2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Peripheral DMA initialization
    hdma_sai2_blockB.Instance = DMA1_Stream4;
    hdma_sai2_blockB.Init.Request = DMA_REQUEST_SAI2_B;
    hdma_sai2_blockB.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_blockB.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai2_blockB.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai2_blockB.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_blockB.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai2_blockB.Init.Mode = DMA_CIRCULAR;
    hdma_sai2_blockB.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_sai2_blockB.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_blockB) != HAL_OK)
        return Result::ERR;

    __HAL_LINKDMA((SAI_HandleTypeDef*)&hsai_BlockB2, hdmarx, hdma_sai2_blockB);
    __HAL_LINKDMA((SAI_HandleTypeDef*)&hsai_BlockB2, hdmatx, hdma_sai2_blockB);

    // Enable SAI to generate clock used by audio driver
    __HAL_SAI_ENABLE(&hsai_BlockB2);

    // Configure the PCM1681 via I2C1
    I2CHandle::Config i2c_cfg;
    i2c_cfg.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_cfg.mode = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_cfg.speed = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_cfg.pin_config.scl = { DSY_GPIOB, 8 };
    i2c_cfg.pin_config.sda = { DSY_GPIOB, 9 };
    I2CHandle i2c1;
    i2c1.Init(i2c_cfg);
    if (codec.Init(i2c1) != daisy::Pcm1681::Result::OK)
        return Result::ERR;

    return Result::OK;
}

// ****************************************************************************
DaisyRogue::Result DaisyRogue::StartRogueAudio(AudioHandle::TdmAudioCallback cb)
 {

     // register the callback
     callback_ = cb;

    // Start the audio engine
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t *)gAudioOutBuff, (AUDIO_BUFF_SIZE)) != HAL_OK)
        return Result::ERR;
    return Result::OK;
  }

// ****************************************************************************
void DaisyRogue::InitMidi() {

      MidiUartHandler::Config midi_config;
      midi_config.transport_config.periph = UartHandler::Config::Peripheral::UART_5;
      midi_config.transport_config.tx = Pin(PORTB, 6);
      midi_config.transport_config.rx = Pin(PORTB, 12);
      midi.Init(midi_config);
  }

// ****************************************************************************
void DaisyRogue::SetSeedLed(bool state)
{
    dsy_gpio_write(&seedLed, state);
}

// ****************************************************************************
//void DaisyRogue::SetDebugOut(bool state)
//{
//    dsy_gpio_write(&debugOut, state);
//}

// ****************************************************************************
extern "C" void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sai2_blockB);
}

// *****************************************************************************
extern "C" void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI2_Block_B)
    {
        //if(gAudioIRQ2_ipFlag)
        //    rogueTrap(2);
        gAudioIRQ1_ipFlag = true;

        int32_t * dstPtr = &gAudioOutBuff[0];
        if(callback_ != nullptr) {
            (callback_)(inBuf, outBuf, 8, MIX_BUFF_SAMPLES);
            for(int s = 0; s < MIX_BUFF_SAMPLES; s++) {
                for (int c = 0; c < 8; c++) {
                    *dstPtr++ = f2s32(outBuf[s + (c * MIX_BUFF_SAMPLES)]);
                }
            }
        }
        SCB_CleanDCache_by_Addr((uint32_t *)&gAudioOutBuff[0], AUDIO_DMA_SIZE * sizeof(uint32_t));
        gAudioIRQ1_ipFlag = false;
    }
}

// *****************************************************************************
extern "C" void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI2_Block_B)
    {
        //if(gAudioIRQ1_ipFlag)
        //    rogueTrap(2);
        gAudioIRQ2_ipFlag = true;

        int32_t* dstPtr = &gAudioOutBuff[AUDIO_DMA_SIZE];
        if (callback_ != nullptr) {
            (callback_)(inBuf, outBuf, 8, MIX_BUFF_SAMPLES);
            for (int s = 0; s < MIX_BUFF_SAMPLES; s++) {
                for (int c = 0; c < 8; c++) {
                    *dstPtr++ = f2s32(outBuf[s + (c * MIX_BUFF_SAMPLES)]);
                }
            }
        }
        SCB_CleanDCache_by_Addr((uint32_t *)&gAudioOutBuff[AUDIO_DMA_SIZE], AUDIO_DMA_SIZE * sizeof(uint32_t));
        gAudioIRQ2_ipFlag = false;
    }
}
