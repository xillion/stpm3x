#ifndef _STPM3X_H_
#define _STPM3X_H_

#include "stpm3x_typedef.h"
#include "mbed.h"
#include "platform/CircularBuffer.h"

typedef enum
{
    crc_err,
    no_err
}stpm3x_err_t;

class Stpm3x 
{
public:
    Stpm3x(RawSerial *_serial, stpm3x_pinout_t pinout):EN_PIN(pinout->pins[STPM3X_EN].pin);
    virtual ~Stpm3x();

    void reset(stmp3x_reset_t);
    stpm3x_err_t powerup(void);
    void powerdown(void);
    stpm3x_err_t ping(void);
    
    uint32_t write(stpm3x_address_t address, uint8_t length, uint32_t *pdata, uint8_t in_wait_stpm);
    uint32_t read(stpm3x_address_t address, uint8_t length, uint32_t *pdata);

//Configuration function

//Mettering function
    uint32_t getActivePower();
    uint32_t getReactivePower();
    uint32_t getApparentPower();
    uint32_t getActiveEnergy();
    uint32_t getReactiveEnergy();
    uint32_t getApparentEnergy();
//Calibration function

private:
/**********************************************************************
                Private variable declaration
***********************************************************************/
    stpm_regs_t REGS;
    RawSerial *serial;
    stpm3x_pinout_t *pinout;
    CircularBuffer<unsigned char, 10> txBuf;
    CircularBuffer<unsigned char, 10> rxBuf;
    DigitalOut EN_PIN;
    DigitalOut SYN_PIN(pinout->pins[STPM3X_SYN].pin);
    DigitalOut CS_PIN(pinout->pins[STPM3X_SCS].pin);
    InterruptIn INT1_PIN();
    InterruptIn INT1_PIN();

/**********************************************************************
                Private function declaration
***********************************************************************/
    uint8_t byteReverse(uint8_t in_byte);
    uint8_t crcByte (uint8_t in_Data, uint8_t CRC_u8Checksum);
    uint8_t crcFrame(uint8_t *pBuf);
    stpm3x_err_t tranceiveHandler(void);
    uint8_t byteTranceive(uint8_t data);


protected:


};


#endif
