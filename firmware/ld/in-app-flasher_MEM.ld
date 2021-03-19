MEMORY
{
  /* Define each memory region */
  RamLoc40     (rwx) : ORIGIN = 0x10080000, LENGTH = 0xa000    /* 40K bytes  -- data upload buffer */  
}

  /* Define a symbol for the top of each memory region */
  __base_RamLoc40 = 0x10080000  ; /* RamLoc32 */  
  __base_RAM = 0x10080000 ; /* RAM */  
  __top_RamLoc40 = 0x10080000 + 0xa000 ; /* 32K bytes */  
  __top_RAM = 0x10080000 + 0xa000 ; /* 32K bytes */  
