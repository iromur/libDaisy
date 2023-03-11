#pragma once
#include "daisy_seed.h"
#include <stm32h7xx_hal.h>

namespace daisy
{
/**
    @brief Class that handles initializing all of the hardware specific to the Daisy Rogue.
    @author Jamie Robertson, Robertsonics
    @date March 2023
    @ingroup boards
*/

class DaisyRogue
{
  public:

    enum class Result {
        OK,
        ERR,
    };

    enum
    {
        SW_1,    /**< trigger input */
        SW_2,    /**< trigger input */
        SW_3,    /**< trigger input */
        SW_4,    /**< trigger input */
        SW_5,    /**< trigger input */
        SW_6,    /**< trigger input */
        SW_7,    /**< trigger input */
        SW_8,    /**< trigger input */
        SW_LAST  /**< & */
    };

    DaisyRogue() {}
    ~DaisyRogue() {}

    Result Init(bool boost = false);
    Result StartRogueAudio(AudioHandle::TdmAudioCallback cb);

    void SetSeedLed(bool state);
    void ProcessDigitalControls();
    Switch* GetSwitch(size_t idx);

    DaisySeed seed;
    System system;
    MidiUartHandler midi;
    Switch sw[SW_LAST];
    SAI_HandleTypeDef hsai_BlockB2;
 
  private:

    dsy_gpio seedLed;
    dsy_gpio trigger[8];
    Pcm1681 codec;

    Result InitRogueAudioHardware();
    void InitMidi();
};

} // namespace daisy