#include "common.h"
#include "configblock.h"

uint8_t getMyId(void)
{
    uint64_t address = configblockGetRadioAddress();
    return (uint8_t)((address) & 0x00000000ff);
}