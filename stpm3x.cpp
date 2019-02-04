#include "stpm3x.h"


//wait_us(int us);

stpm3x::Stpm3x(RawSerial *_serial, stpm3x_pinout_t _pinout);
{
    pinout = _pinout;
    serial = _serial;
};

stpm3x::~stpm3x()
{
    delete txBuf;
    delete rxBuf;
};

/**
  * @brief  This function send 1 pulse on SYN signal to latch metrology registers of STPM external chips
  * @param  METRO_NB_Device_t in_Metro_Device_Id
  * @retval None
  */
void Metro_HAL_STPM_SYN_single_pulse()
{
 
    /* Before to toogle SYN pin , we have to Clear  SS  pin ( Chip select to Ext device ) */
  CS_PIN = pinout->pins[STPM3X_SCS].SET;
  SYN_PIN = pinout->pins[STPM3X_SYN].SET;
  /* reset SYNC pulse */
    wait_us(100); 
  /* set SYNC pulse */
  SYN_PIN = pinout->pins[STPM3X_SYN].RESET;
   wait_us(100);
  CS_PIN = pinout->pins[STPM3X_SCS].RESET;
}

void stpm3x::powerup()
{
    /* set UART mode at STPM3x power up, we have to set SS pin */
    Metro_HAL_CSS_EXT_Device(in_Metro_Device_Id,CS_PIN_ACTIVE);
    CS_PIN = pinout->pins[STPM3X_SCS].SET;

    /* set ENable Pin configured as low in CubeMX*/
    wait_us(1000); 
    EN_PIN = pinout->pins[STPM3X_EN].RESET;
    wait_us(1000); 
    EN_PIN = pinout->pins[STPM3X_EN].SET;
    wait_us(1000); 
}

void stpm3x::powerdown()
{

}

void stpm3x::reset(stmp3x_reset_t reset)
{
    uint32_t address;
    if (reset ==  RESET_SW)
    {
      /* Set the reset bit in the  DSP Control Register 3 of stpm requested(STPM) */
      /* First put the bit inside internal struct */
      REGS.DSPCTRL3.SW_RST = 1;
      address = (uint32_t)((uint8_t*)&REGS.DSPCTRL3 - (uint8_t*)&REGS.DSPCTRL1)/2;
      //dspctrl3_t *t =  ((dspctrl3_t *) &REGS.DSPCTRL3);
      /* Write blocks inside external chip */
      //stpm3x_address_t address, uint8_t length = 1, uint32_t *pdata, uint8_t in_wait_stpm
      write(address,1,(uint32_t *)&REGS.DSPCTRL3,STPM_WAIT);
    }
    /* reset SYN hardware is requested, send 3 pulses to SYN signal pin */
    else if (reset ==  RESET_HW)
    {

      /* Before to toogle SYN pin , we have to Clear  SS  pin ( Chip select to Ext device ) */
      CS_PIN = pinout->pins[STPM3X_SCS].SET;
      SYN_PIN = pinout->pins[STPM3X_SYN].SET;
      /* reset SYNC pulse */
        wait_us(100); 
      /* set SYNC pulse */
      SYN_PIN = pinout->pins[STPM3X_SYN].RESET;
      wait_us(100);
      CS_PIN = pinout->pins[STPM3X_SCS].RESET;

      for(uint8_t i=0;i<=2; i++) 
      {
        /* reset SYNC pulse */
        SYN_PIN = pinout->pins[STPM3X_SYN].RESET;
        wait_us(100);
        SYN_PIN = pinout->pins[STPM3X_SYN].SET;
        wait_us(100);
      }
      CS_PIN = pinout->pins[STPM3X_SCS].RESET;
    }
};



uint8_t stpm3x::byteReverse(uint8_t in_byte)
{
    in_byte = ((in_byte >> 1) & 0x55) | ((in_byte << 1) & 0xaa);
    in_byte = ((in_byte >> 2) & 0x33) | ((in_byte << 2) & 0xcc);
    in_byte = ((in_byte >> 4) & 0x0F) | ((in_byte << 4) & 0xF0);
    return in_byte;
};

uint8_t stpm3x::crcByte (uint8_t in_Data, uint8_t CRC_u8Checksum)
{
    uint8_t loc_u8Idx;
    uint8_t loc_u8Temp;
    loc_u8Idx=0;
    while(loc_u8Idx<8)
    {
        loc_u8Temp=in_Data^CRC_u8Checksum;
        CRC_u8Checksum<<=1;
        if(loc_u8Temp&0x80)
        {
            CRC_u8Checksum^=CRC_8;
        }
        in_Data<<=1;
        loc_u8Idx++;
    }
    return CRC_u8Checksum;
};

uint8_t stpm3x::crcFrame(uint8_t *pBuf)
{
    uint8_t i, CRC_u8Checksum = 0x00;

    for (i=0; i<STPM3x_FRAME_LEN-1; i++)
    {
    	CRC_u8Checksum = crcByte(pBuf[i], CRC_u8Checksum);
    }

    return CRC_u8Checksum;
};

stpm3x_err_t stpm3x::tranceiveHandler(void)
{
    while(!txBuff.empty())
    {
        uint8_t data;
        txBuff.pop(data);
        rxBuff.push(byteTranceive(data));
    }
    return (no_err);
};


uint8_t stpm3x::byteTranceive(uint8_t data)
{
    serial.putc((char)data);
    return (uint8_t)serial.getc();
};

/**
 * 
 * 
*/
/*

*/
uint32_t stpm3x::write(stpm3x_address_t address, uint8_t length = 1, uint32_t *pdata, uint8_t in_wait_stpm)
{
   uint32_t retSize = 0;
   uint8_t CRC_on_reversed_buf;
   uint8_t i=0;
   uint8_t *pSplitedData = (uint8_t*)pdata;
   uint8_t k=0;
   uint8_t frame_with_CRC[STPM3x_FRAME_LEN];
   uint8_t frame_without_CRC[STPM3x_FRAME_LEN -1];


    /* Reset Buffers */
   while(!txBuf.empty())
   {
        txBuf.pop();
        rxBuf.pop();
   }

   length *= 2;

   for (k=0;k<length;k++)
   {

     /* Format the frame with Write base address */
     frame_with_CRC[0] = 0xff; /*  No read requested, put dummy frame  */
     frame_with_CRC[1] = (uint8_t)(address + k); /*  write Address requested */
     frame_with_CRC[2] = *(pSplitedData); /*   DATA for 16-bit register to be written, LSB */
     frame_with_CRC[3] = *(++pSplitedData); /*  DATA for 16-bit register to be written, MSB */

     /* Increment Pointer to next U16 data for the next loop */
     pSplitedData++;


     /* Reverse bytes */
     for (i=0;i<(STPM3x_FRAME_LEN-1);i++)
     {
       frame_without_CRC[i] = byteReverse(frame_with_CRC[i]);
     }

     /* Calculate CRC and put it at the end of the frame */
     CRC_on_reversed_buf = crcFrame(frame_without_CRC);
     frame_with_CRC[4] = byteReverse(CRC_on_reversed_buf);
     
    //frame_with_CRC[4] = byteReverse(crcFrame(frame_without_CRC));

     /* Put the frame inside the TX queue      */
    i=0;
    while(i < STPM3x_FRAME_LEN)
    {
        txBuff.push(frame_with_CRC[STPM3x_FRAME_LEN-i]);
    }

     retSize = retSize + STPM3x_FRAME_LEN;
          
     CS_PIN = pinout->pins[STPM3X_SCS].SET;
     /* Send  Data */
     tranceiveHandler();
     //Metro_HAL_UsartTxStart(in_Metro_Device_Id);

/*
ToDo enable interrupt for rx
*/
} /* end For Nb blocks loop */
   return(retSize);
};

uint32_t stpm3x::read(uint8_t address, uint8_t length, uint32_t *pdata)
{

};

/**
  * @brief  Read Block Registers from device
  *
  *
  * @retval void
  */
uint8_t stpm3x::readblock(uint8_t Offset, uint8_t BlockNum, uint32_t * out_p_Buffer)
{
  uint32_t address = 0x0;
  uint8_t  error=0;

      /* Calculate the base address to read inisde STPM chip  */
     /* the offset should be provided (2 bytes -> 16 bits) format for STPM */
      address = (uint32_t)&METRO_STPM->DSPCTRL1 + Offset;

      /* read blocks from external chip */
      read(in_Metro_Device_Id,(uint8_t*)&address,BlockNum,out_p_Buffer);

   return error;
}
/**
  * @brief  Write Block Registers to device
  *
  *
  * @retval void
  */
uint8_t stpm3x::writeblock(uint8_t Offset, uint8_t BlockNum, uint32_t * in_p_Buffer)
{
  uint32_t address = 0x0;
  uint32_t ret_size;

  /* Calculate the base address to read inisde STPM chip  */
  /* the offset should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
  address = (uint32_t)&METRO_STPM->DSPCTRL1 + (Offset);

   /* write blocks from external chip */
  ret_size = write(in_Metro_Device_Id,(uint8_t*)&address,BlockNum,in_p_Buffer,STPM_WAIT); 
  
  return(ret_size);
}

/**
  * @brief     This function set latch inside Metrology devices
               Latch the device registers according to the latch type selection driving SYN pin
               or writing S/W Latchx bits in the DSP_CR3 register
               or setting auto-latch by S/W Auto Latch bit in the DSP_CR3 register
  * @param[in]   in_Metro_Device_Id (device ID), EXT1 to EXT4
  * @param[in]   Latch type
  * @param[out]  none
  * @retval
  */
void stpm3x::latch(stpm3x_latch_t Latch_Device_Type)
{
  uint32_t address = 0;

  switch (Latch_Device_Type)
    {

      case LATCH_AUTO:
      {

        /* reset latch 1 and 2 bits in the internal DSP Control Register 3 */
        p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 &= ~ BIT_MASK_STPM_LATCH1;
        p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 &= ~ BIT_MASK_STPM_LATCH2;


        /* Set  latch auto in the internal DSP Control Register 3*/
        p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 |= BIT_MASK_STPM_AUTO_LATCH;

        /* Now send data to the external chip */
        /* Calculate the base address to read inisde STPM chip  */
        /* the offset should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
        tmp_addr = (uint32_t)((uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 - (uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL1)/2;

        /* Write register inside external chip */
        Metro_HAL_Stpm_write(in_Metro_Device_Id,(uint8_t*)&tmp_addr,1,&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3,STPM_WAIT);

      }
      break;
      case LATCH_SW:
      {
        /* Set  latch SW 1 et 2 for the Two channels  the internal DSP Control Register 3*/
        p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 |= BIT_MASK_STPM_LATCH1|BIT_MASK_STPM_LATCH2;

        /* Now send data to the external chip */
        /* Calculate the base address to read inisde STPM chip  */
        /* the offset should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
        tmp_addr = (uint32_t)((uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3 - (uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL1)/2;

        /* Write register inside external chip */
        Metro_HAL_Stpm_write(in_Metro_Device_Id,(uint8_t*)&tmp_addr,1,&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL3,STPM_WAIT);

      }
      break;
      case LATCH_HW:
      {
        /* Latch external chip with syn PIN : 1 pulses is needed to latch */
        CS_PIN = pinout->pins[STPM3X_SCS].SET;
        SYN_PIN = pinout->pins[STPM3X_SYN].SET;
        /* reset SYNC pulse */
          wait_us(100); 
        /* set SYNC pulse */
        SYN_PIN = pinout->pins[STPM3X_SYN].RESET;
        wait_us(100);
        CS_PIN = pinout->pins[STPM3X_SCS].RESET;
      }
      break;
    }
     
    
     /* After latch with syn pin or SW reg , we have to retreive metrology data from STPM external chip requested */
    /* from DSPEVENT1 to TOT_REG4 : 49 U32 reg from STPM*/
    /* Calculate the base address of Metrology data to read inisde STPM chip  */
    /* the offset should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
    tmp_addr = (uint32_t)((uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.UARTSPISR - (uint8_t*)&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.DSPCTRL1)/2;

    read(in_Metro_Device_Id,(uint8_t*)&tmp_addr,METRO_STPM_DATA_REG_NB_BLOCKS,&p_Metro_Device_Config[in_Metro_Device_Id].metro_stpm_reg.UARTSPISR);


}


//**************************************************************

uint32_t stpm3x::getActivePower()
{

};

uint32_t stpm3x::getReactivePower()
{

};

uint32_t stpm3x::getApparentPower()
{

};

uint32_t stpm3x::getActiveEnergy()
{

};

uint32_t stpm3x::getReactiveEnergy()
{

};

uint32_t stpm3x::getApparentEnergy()
{

};