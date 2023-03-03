#pragma once
#include "daisy_seed.h"
#include <stm32h7xx_hal.h>

#define MIX_BUFF_SAMPLES 128
#define AUDIO_DMA_SIZE (MIX_BUFF_SAMPLES * 8)
#define AUDIO_BUFF_SIZE (AUDIO_DMA_SIZE * 2)

namespace daisy
{
/**
    @brief Class that handles initializing all of the hardware specific to the Rogue WAV hardware.
    @author Jamie Robertson, Robertsonics
    @date February 2023
    @ingroup boards
*/

class DaisyRogue
{
  public:
 
    DaisyRogue() {}
    ~DaisyRogue() {}

    /**Initializes the Rogue, and all of its hardware.*/
    int Init(bool boost = false);

    int StartRogueAudio(AudioHandle::TdmAudioCallback cb);

    /** 
    Wait some ms before going on.
    \param del Delay time in ms.
    */
    void DelayMs(size_t del);

    /** Starts the callback
    \param cb Interleaved callback function
    */
    void StartSeedAudio(AudioHandle::InterleavingAudioCallback cb);

    /** Starts the callback
    \param cb Non-interleaved callback function
    */
    void StartSeedAudio(AudioHandle::AudioCallback cb);

    /**
       Switch callback functions
       \param cb New interleaved callback function.
    */
    void ChangeSeedAudioCallback(AudioHandle::InterleavingAudioCallback cb);

    /**
       Switch callback functions
       \param cb New non-interleaved callback function.
    */
    void ChangeSeedAudioCallback(AudioHandle::AudioCallback cb);

    /** Stops the audio if it is running. */
    void StopSeedAudio();

    /** Sets the number of samples processed per channel by the audio callback.
     */
    void SetSeedAudioBlockSize(size_t size);

    /** Returns the number of samples per channel in a block of audio. */
    size_t SeedAudioBlockSize();

    /** Updates the Audio Sample Rate, and reinitializes.
     ** Audio must be stopped for this to work.
     */
    void SetSeedAudioSampleRate(SaiHandle::Config::SampleRate samplerate);

    /** Returns the audio sample rate in Hz as a floating point number.
     */
    float SeedAudioSampleRate();

    /** Returns the rate in Hz that the Audio callback is called */
    float SeedAudioCallbackRate();

    /** Sets the state of the built in LED */
    void SetSeedLed(bool state);

    /** Sets the state of the debug output */
    void SetDebugOut(bool state);

    /** Process digital controls */
    void ProcessDigitalControls();

    DaisySeed       seed;
    System          system;
    MidiUartHandler midi;

    SAI_HandleTypeDef hsai_BlockB2;
 
  private:

    dsy_gpio seedLed;
    Pcm1681 codec;

    void InitMidi();
};

} // namespace daisy