#include "dev/codec_pcm1681.h"
#include "sys/system.h"

namespace daisy {

Pcm1681::Result Pcm1681::Init(I2CHandle i2c)
{
    i2c_ = i2c;
    dev_addr_ = 0x98;
    uint8_t val = 0x00;

    // Issue a reset
    if (WriteRegister(10, 0x80) != Result::OK)
        return Result::ERR;
    System::Delay(100);
    if (WriteRegister(10, 0x80) != Result::OK)
        return Result::ERR;
    System::Delay(100);

    // Sharp filter rolloff and left-justified TDM mode
    if (WriteRegister(9, 0x27) != Result::OK)
        return Result::ERR;
    System::Delay(4);
    if (ReadRegister(9, &val) != Result::OK)
        return Result::ERR;
    if(val != 0x27)
        return Result::ERR;
    System::Delay(4);

    // Enable wide mode
    if(WriteRegister(12, 0x80) != Result::OK)
        return Result::ERR;
    System::Delay(4);
    if(ReadRegister(12, &val) != Result::OK)
        return Result::ERR;
    if(val != 0x80)
        return Result::ERR;
    System::Delay(4);

    // Success
    return Result::OK;
}

Pcm1681::Result Pcm1681::ReadRegister(uint8_t addr, uint8_t* data)
{
    if(i2c_.ReadDataAtAddress(dev_addr_, addr, 1, data, 1, 250)
       != I2CHandle::Result::OK)
    {
        return Result::ERR;
    }
    return Result::OK;
}

Pcm1681::Result Pcm1681::WriteRegister(uint8_t addr, uint8_t val)
{
    if(i2c_.WriteDataAtAddress(dev_addr_, addr, 1, &val, 1, 250)
       != I2CHandle::Result::OK)
    {
        return Result::ERR;
    }
    return Result::OK;
}

} // namespace daisy
