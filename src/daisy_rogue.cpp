#include "daisy_rogue.h"

using namespace daisy;

#define SEED_LED_PORT DSY_GPIOC
#define SEED_LED_PIN 7

#define DEBUG_OUT_PORT DSY_GPIOA
#define DEBUG_OUT_PIN 5

DMA_HandleTypeDef hdma_sai2_blockB;
dsy_gpio debugOut;
bool debugOutState;
bool gAudioIRQ1_ipFlag = false;
bool gAudioIRQ2_ipFlag = false;
int32_t DMA_BUFFER_MEM_SECTION gAudioOutBuff[AUDIO_BUFF_SIZE] __attribute__((aligned(32)));
//int32_t gAudioOutBuff[AUDIO_BUFF_SIZE] __attribute__((section(".RAM_D2"))) __attribute__((aligned(32)));

float gDspBuffer[AUDIO_DMA_SIZE];

AudioHandle::TdmAudioCallback callback_;
AudioHandle::TdmInputBuffer   inBuf;
AudioHandle::TdmOutputBuffer  outBuf;

// ****************************************************************************
int DaisyRogue::Init(bool boost)
{

    outBuf = (AudioHandle::TdmOutputBuffer)&gDspBuffer[0];
    inBuf  = nullptr;

    for(int i = 0; i < AUDIO_BUFF_SIZE; i++)
        gAudioOutBuff[i] = 0;

    // seed init
    seed.Init(boost);

    seedLed.pin.port = SEED_LED_PORT;
    seedLed.pin.pin  = SEED_LED_PIN;
    seedLed.mode     = DSY_GPIO_MODE_OUTPUT_PP;
    dsy_gpio_init(&seedLed);

    debugOut.pin.port = DEBUG_OUT_PORT;
    debugOut.pin.pin  = DEBUG_OUT_PIN;
    debugOut.mode     = DSY_GPIO_MODE_OUTPUT_PP;
    dsy_gpio_init(&debugOut);

    debugOutState = true;
    dsy_gpio_write(&debugOut, true);

    // DMA controller clock enable
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA1_Stream4_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    hsai_BlockB2.Instance                    = SAI2_Block_B;
    hsai_BlockB2.Init.Protocol               = SAI_FREE_PROTOCOL;
    hsai_BlockB2.Init.AudioMode              = SAI_MODEMASTER_TX;
    hsai_BlockB2.Init.DataSize               = SAI_DATASIZE_32;
    hsai_BlockB2.Init.FirstBit               = SAI_FIRSTBIT_MSB;
    hsai_BlockB2.Init.ClockStrobing          = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_BlockB2.Init.Synchro                = SAI_ASYNCHRONOUS;
    hsai_BlockB2.Init.OutputDrive            = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockB2.Init.NoDivider              = SAI_MASTERDIVIDER_ENABLE;
    hsai_BlockB2.Init.FIFOThreshold          = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockB2.Init.AudioFrequency         = SAI_AUDIO_FREQUENCY_44K;
    hsai_BlockB2.Init.SynchroExt             = SAI_SYNCEXT_DISABLE;
    hsai_BlockB2.Init.MonoStereoMode         = SAI_STEREOMODE;
    hsai_BlockB2.Init.CompandingMode         = SAI_NOCOMPANDING;
    hsai_BlockB2.Init.TriState               = SAI_OUTPUT_NOTRELEASED;
    hsai_BlockB2.Init.PdmInit.Activation     = DISABLE;
    hsai_BlockB2.Init.PdmInit.MicPairsNbr    = 1;
    hsai_BlockB2.Init.PdmInit.ClockEnable    = SAI_PDM_CLOCK1_ENABLE;
    hsai_BlockB2.FrameInit.FrameLength       = 256;
    hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
    hsai_BlockB2.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    hsai_BlockB2.FrameInit.FSPolarity        = SAI_FS_ACTIVE_HIGH;
    hsai_BlockB2.FrameInit.FSOffset          = SAI_FS_FIRSTBIT;
    hsai_BlockB2.SlotInit.FirstBitOffset     = 0;
    hsai_BlockB2.SlotInit.SlotSize           = SAI_SLOTSIZE_DATASIZE;
    hsai_BlockB2.SlotInit.SlotNumber         = 8;
    hsai_BlockB2.SlotInit.SlotActive         = 0x0000FFFF;
    if(HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
        return 1;

    // SAI2_B_Block_B GPIO Configuration
    // PG9     ------> SAI2_FS_B
    // PA1     ------> SAI2_MCLK_B
    // PA0     ------> SAI2_SD_B
    // PA2     ------> SAI2_SCK_B

    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_SAI2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Peripheral DMA init
    hdma_sai2_blockB.Instance                 = DMA1_Stream4;
    hdma_sai2_blockB.Init.Request             = DMA_REQUEST_SAI2_B;
    hdma_sai2_blockB.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_blockB.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai2_blockB.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai2_blockB.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_blockB.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sai2_blockB.Init.Mode                = DMA_CIRCULAR;
    hdma_sai2_blockB.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_sai2_blockB.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&hdma_sai2_blockB) != HAL_OK)
        return 2;

    __HAL_LINKDMA((SAI_HandleTypeDef *)&hsai_BlockB2, hdmarx, hdma_sai2_blockB);
    __HAL_LINKDMA((SAI_HandleTypeDef *)&hsai_BlockB2, hdmatx, hdma_sai2_blockB);

     	// Enable SAI to generate clock used by audio driver
    __HAL_SAI_ENABLE(&hsai_BlockB2);

    // Configure the PCM1681 via I2C1
    I2CHandle::Config i2c_cfg;
    i2c_cfg.periph         = I2CHandle::Config::Peripheral::I2C_1;
    i2c_cfg.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_cfg.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_cfg.pin_config.scl = {DSY_GPIOB, 8};
    i2c_cfg.pin_config.sda = {DSY_GPIOB, 9};
    I2CHandle i2c1;
    i2c1.Init(i2c_cfg);
    if(codec.Init(i2c1) != daisy::Pcm1681::Result::OK)
        return 3;

    return 0;
 }

 // ****************************************************************************
 int DaisyRogue::StartRogueAudio(AudioHandle::TdmAudioCallback cb)
 {

     // register the callback
     callback_ = cb;

    // Start the audio engine
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t *)gAudioOutBuff, (AUDIO_BUFF_SIZE)) != HAL_OK)
        return 1;
    return 0;
  }

 //=============================================================================
//  The following functions pertain to the built-in stereo audio input/output
//  of the Seed.
//  ===========================================================================
 
// ****************************************************************************
void DaisyRogue::StartSeedAudio(AudioHandle::InterleavingAudioCallback cb)
{
    seed.StartAudio(cb);
}

// ****************************************************************************
void DaisyRogue::StartSeedAudio(AudioHandle::AudioCallback cb)
{
    seed.StartAudio(cb);
}

// ****************************************************************************
void DaisyRogue::ChangeSeedAudioCallback(AudioHandle::InterleavingAudioCallback cb)
{
    seed.ChangeAudioCallback(cb);
}

// ****************************************************************************
void DaisyRogue::ChangeSeedAudioCallback(AudioHandle::AudioCallback cb)
{
    seed.ChangeAudioCallback(cb);
}

// ****************************************************************************
void DaisyRogue::StopSeedAudio()
{
    seed.StopAudio();
}

// ****************************************************************************
void DaisyRogue::SetSeedAudioBlockSize(size_t size)
{
    seed.SetAudioBlockSize(size);
    //SetHidUpdateRates();
}

// ****************************************************************************
size_t DaisyRogue::SeedAudioBlockSize()
{
    return seed.AudioBlockSize();
}

// ****************************************************************************
void DaisyRogue::SetSeedAudioSampleRate(SaiHandle::Config::SampleRate samplerate)
{
    seed.SetAudioSampleRate(samplerate);
    //SetHidUpdateRates();
}

// ****************************************************************************
float DaisyRogue::SeedAudioSampleRate()
{
    return seed.AudioSampleRate();
}

// ****************************************************************************
float DaisyRogue::SeedAudioCallbackRate()
{
    return seed.AudioCallbackRate();
}

// ****************************************************************************
void DaisyRogue::SetSeedLed(bool state)
{
    dsy_gpio_write(&seedLed, state);
}

// ****************************************************************************
void DaisyRogue::SetDebugOut(bool state)
{
    dsy_gpio_write(&debugOut, state);
}

// ****************************************************************************
void DaisyRogue::ProcessDigitalControls() {
}

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

        if(callback_ != nullptr) {
            (callback_)(inBuf, outBuf, 8, MIX_BUFF_SAMPLES);
            for(int s = 0; s < AUDIO_DMA_SIZE; s++) {
                gAudioOutBuff[s] = f2s32(outBuf[s]);
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

        if(callback_ != nullptr) {
            (callback_)(inBuf, outBuf, 8, MIX_BUFF_SAMPLES);
            for(int s = 0; s < AUDIO_DMA_SIZE; s++) {
                gAudioOutBuff[AUDIO_DMA_SIZE + s] = f2s32(outBuf[s]);
            }
        }

        SCB_CleanDCache_by_Addr((uint32_t *)&gAudioOutBuff[AUDIO_DMA_SIZE], AUDIO_DMA_SIZE * sizeof(uint32_t));
        gAudioIRQ2_ipFlag = false;
    }
}
