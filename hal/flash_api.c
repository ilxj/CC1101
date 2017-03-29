#include "stm32f0xx.h"
#include "log.h"
#include "flash_api.h"

int32_t fs_Write( uint8_t *pData, uint32_t dataLen )
{
  volatile uint32_t count = 0,i=0;
  volatile uint32_t NbrOfPage = 0x00;
  volatile uint32_t Address = 0x0000;
  volatile int32_t EraseCounter=0;
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();
  /* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
  /* Erase the FLASH pages */
  for(EraseCounter = 0; (EraseCounter < NbrOfPage); EraseCounter++)
  {
    if (FLASH_ErasePage(FLASH_USER_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
    {
        log( ERROR,"Flash erase ERROR!\n");
        /* Error occurred while sector erase. User can add here some code to deal with this error  */
        return (-1);
    }
  }
  count = (dataLen/2) +(dataLen%2);

  Address = FLASH_USER_START_ADDR;
  log(INFO,"fs_WriteAdd:%08X count=%d \n",Address,count );
  for( i=0;i<count;i++ )
  {
    if (FLASH_ProgramHalfWord( Address, *(uint16_t*)pData ) == FLASH_COMPLETE)
    {
      pData=pData+2;
      Address = Address + 2;
    }
    else
    { 
      /* Error occurred while writing data in Flash memory.User can add here some code to deal with this error */
      log( ERROR,"Flash ProgramWord ERROR!\n");
      return (-1);
    }
  }
  log(INFO,"%s \n",__FUNCTION__ );
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  FLASH_Lock(); 
  Delay_ms(1);
  return 0;
}
int32_t fs_Read( uint8_t *pData, uint32_t dataLen )
{
  uint32_t Address = 0x00;
  Address = FLASH_USER_START_ADDR;
  log( INFO,"fs_ReadAdd:%08X .\n",Address );
  {
    memcpy( pData,(void*)Address,dataLen );
  }
  Delay_ms(1);
  return 0;
}
