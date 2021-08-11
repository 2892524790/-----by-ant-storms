#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "BSP_Flash.h"
void FlashProgramedata(u16 data)
{
    static u32 flashwptr = 0;//IAPSTART;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_ProgramHalfWord(flashwptr, data);
    flashwptr = flashwptr + 2;

    FLASH_Lock();
}

//flash页擦除
void FlashAllErase(void)
{

//flash解锁
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//	for(n = 8; n < 64; n++)
//	{
    //进行特定扇区擦除
    FLASH_EraseSector(FLASH_Sector_1, VoltageRange_2);
//  FLASH_ErasePage(0x8000000 + (n * PAGESIZE));
//	}
    //flash上锁
    FLASH_Lock();

}
//flash的boot标志位检测程序
void FlashProgramedataBoot(u16 data)
{
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_ProgramHalfWord(0x80e0000, data);

    FLASH_Lock();
}

//flash boot标志页擦除
void FlashAllEraseBoot(void)
{
//flash解锁
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    //进行特定扇区擦除
    FLASH_EraseSector(FLASH_Sector_11, VoltageRange_2);
    FLASH_Lock();
}


//写入校准矩阵数据
int FlashWriteK(u8* data)
{
    u8 i;
    u32 addr;
    addr = 0x80e0000;
    FlashAllEraseBoot();
    FLASH_Unlock();
    for(i = 0; i < 48; i++)
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        FLASH_ProgramByte(addr + i, *(data + i));
    }
    FLASH_Lock();
    return 0;
}

//读出校准数据
int FlashReadK(u8* data)
{
    u8 i;
    u8* addr = (u8*)0x80e0000;
    for(i = 0; i < 48; i++)
    {
        *(data + i) = *(addr + i);
    }
    return 0;
}
