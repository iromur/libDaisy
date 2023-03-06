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
 
    DaisyRogue() {}
    ~DaisyRogue() {}

    Result Init(bool boost = false);
    Result StartRogueAudio(AudioHandle::TdmAudioCallback cb);

    void SetSeedLed(bool state);
    //void SetDebugOut(bool state);

    DaisySeed seed;
    System system;
    MidiUartHandler midi;
    SAI_HandleTypeDef hsai_BlockB2;
 
  private:

    dsy_gpio seedLed;
    Pcm1681 codec;

    Result InitRogueAudioHardware();
    void InitMidi();
};

} // namespace daisy