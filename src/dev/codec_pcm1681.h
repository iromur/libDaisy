#pragma once
#ifndef DSY_CODEC_PCM1681_H
#define DSY_CODEC_PCM1681_H
#include "per/i2c.h"
namespace daisy
{
/**
 * @brief Driver for the TI PCM1681
 */
class Pcm1681
{
  public:
    enum class Result
    {
        OK,
        ERR,
    };

    Pcm1681() {}
    ~Pcm1681() {}

    /** Initializes the PCM1681 in 24-bit TDM mode 
     * \param i2c Initialized I2CHandle configured at 400kHz or less
     */
    Result Init(I2CHandle i2c);

  private:
    /** Reads the data byte corresponding to the register address */
    Result ReadRegister(uint8_t addr, uint8_t *data);

    /** Writes the specified byte to the register at the specified address.*/
    Result WriteRegister(uint8_t addr, uint8_t val);

    I2CHandle i2c_;
    uint8_t   dev_addr_;
};

} // namespace daisy
#endif