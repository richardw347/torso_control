#include "i2cMaster.h"

void i2cMaster::cmd(uint8_t cmd)
{
  uint16_t i = 0;
  // send command
  TWCR = cmd;
  // wait for command to complete
  while (!(TWCR & (1 << TWINT)));
  
  // save status bits
  status = TWSR & 0xF8; 
}

bool i2cMaster::read(uint8_t slaveAddr, uint8_t regAddr, uint8_t numOfBytes, uint8_t *data)
{
  uint8_t i, buff[numOfBytes];

  TIMSK1 &= ~(1 << OCIE1A);

  I2C.start(slaveAddr, WRITE);

  I2C.writeByte(regAddr);

  I2C.restart(slaveAddr, READ);

  for(i = 0; i < (numOfBytes - 1); i++)
  {
    I2C.readByte(ACK, &data[i]);
  }

  I2C.readByte(NACK, &data[numOfBytes-1]);

  I2C.stop();

  TIMSK1 |= (1 << OCIE1A);
  
  return 1; 
}

bool i2cMaster::write(uint8_t slaveAddr, uint8_t regAddr, uint8_t numOfBytes, uint8_t *data)
{
  uint8_t i;

  TIMSK1 &= ~(1 << OCIE1A);

  I2C.start(slaveAddr, WRITE);
  I2C.writeByte(regAddr);
  
  for(i = 0; i < numOfBytes; i++)
  {
    I2C.writeByte(*(data + i));
  }
  I2C.stop();

  TIMSK1 |= (1 << OCIE1A);

  return 1;
}

bool i2cMaster::readByte(bool ack, uint8_t *data)
{
  if(ack)
  {
    this->cmd((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
  }
  
  else
  {
    this->cmd((1 << TWINT) | (1 << TWEN));
  }

  *data = TWDR;

  return 1;
}

bool i2cMaster::start(uint8_t addr, bool RW)
{
  // send START condition
  this->cmd((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));

  if (this->getStatus() != START && this->getStatus() != REPSTART) 
  {
    return false;
  }

  // send device address and direction
  TWDR = (addr << 1) | RW;
  this->cmd((1 << TWINT) | (1 << TWEN));
  
  if (RW == READ) 
  {
    return this->getStatus() == RXADDRACK;
  } 

  else 
  {
    return this->getStatus() == TXADDRACK;
  }
}

bool i2cMaster::restart(uint8_t addr, bool RW)
{
  return this->start(addr, RW);
}

bool i2cMaster::writeByte(uint8_t data)
{
  TWDR = data;

  this->cmd((1 << TWINT) | (1 << TWEN));

  return this->getStatus() == TXDATAACK;
}

bool i2cMaster::stop(void)
{
  uint16_t i = 0;
  //  issue stop condition
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  // wait until stop condition is executed and bus released
  while (TWCR & (1 << TWSTO));

  status = I2CFREE;

  return 1;
}

uint8_t i2cMaster::getStatus(void)
{
  return status;
}

void i2cMaster::begin(void)
{
  // set bit rate register to 12 to obtain 400kHz scl frequency (in combination with no prescaling!)
  TWBR = 12;
  // no prescaler
  TWSR &= 0xFC;
}

i2cMaster::i2cMaster(void)
{

}
