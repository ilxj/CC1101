
#ifndef FLASH_API_H_
#define FLASH_API_H_

#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800F000)   /* Start @ of user Flash area */
//#define FLASH_USER_START_ADDR   ((uint32_t)0x0800FC00)
#define FLASH_USER_END_ADDR     ((uint32_t)0x08010000)   /* End @ of user Flash area */


void flashOpen( void );
int32_t fs_Write( uint8_t *pData, uint32_t dataLen );
int32_t fs_Read( uint8_t *pData, uint32_t dataLen );

#endif