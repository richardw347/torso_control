#include <Arduino.h>
#include <util/delay.h>
#include "i2cMaster.h"
#include "uStepperEncoder.h"

i2cMaster I2C;
uStepperEncoder *pointer;

extern "C" {

  void TIMER1_COMPA_vect(void)
  {
    float deltaAngle;
    static float curAngle, oldAngle = 0.0, loops = 1.0;

    sei();
    if (I2C.getStatus() != I2CFREE)
    {
      return;
    }

    curAngle = fmod(pointer->getAngle() - pointer->encoderOffset + 360.0, 360.0);

    deltaAngle = (oldAngle - curAngle);

    if (deltaAngle < -50.0)
    {
      deltaAngle += 360.0;
    }
    else if (deltaAngle > 50.0)
    {
      deltaAngle -= 360;
    }

    if ((deltaAngle >= 10.0) || (deltaAngle <= -10.0))
    {
      pointer->curSpeed = deltaAngle * ENCODERSPEEDCONSTANT;
      pointer->curSpeed /= loops;
      loops = 1.0;
      oldAngle = curAngle;
    }
    else
    {
      loops += 1.0;
      if (loops >= 50000.0)
      {
        pointer->curSpeed = 0.0;
      }
    }

    deltaAngle = pointer->oldAngle - curAngle;

    if (deltaAngle < -50.0)
    {
      pointer->angleMoved += (deltaAngle + 360.0);
    }

    else if (deltaAngle > 50.0)
    {
      pointer->angleMoved -= (360.0 - deltaAngle);
    }
    else
    {
      pointer->angleMoved += deltaAngle;
    }

    pointer->oldAngle = curAngle;
  }

}


uStepperEncoder::uStepperEncoder(void)
{
  I2C.begin();
}

float uStepperEncoder::getAngleMoved(void)
{
  float deltaAngle, angle;

  TIMSK1 &= ~(1 << OCIE1A);

  angle = fmod(this->getAngle() - this->encoderOffset + 360.0, 360.0);
  deltaAngle = this->oldAngle - angle;

  if (deltaAngle < -50.0)
  {
    pointer->angleMoved += (deltaAngle + 360.0);
  }

  else if (deltaAngle > 50.0)
  {
    pointer->angleMoved -= (360.0 - deltaAngle);
  }

  else
  {
    this->angleMoved += deltaAngle;
  }

  this->oldAngle = angle;

  TIMSK1 |= (1 << OCIE1A);

  return this->angleMoved;
}

float uStepperEncoder::getSpeed(void)
{
  return this->curSpeed;
}

void uStepperEncoder::setup(void)
{
  TCCR1A = 0;
  TCNT1 = 0;
  OCR1A = 65535;
  TIFR1 = 0;
  TIMSK1 = (1 << OCIE1A);
  TCCR1B = (1 << WGM12) | (1 << CS10);
  this->encoderOffset = this->getAngle();

  this->oldAngle = 0.0;
  this->angleMoved = 0.0;

  sei();

}

void uStepperEncoder::setHome(void)
{
  this->encoderOffset = this->getAngle();

  this->oldAngle = 0.0;
  this->angleMoved = 0.0;
  this->angleMoved = 0.0;

}

float uStepperEncoder::getAngle()
{
  float angle;
  uint8_t data[2];

  I2C.read(ENCODERADDR, ANGLE, 2, data);

  angle = (float)((((uint16_t)data[0]) << 8 ) | (uint16_t)data[1]) * 0.087890625;

  return angle;
}

uint16_t uStepperEncoder::getStrength()
{
  uint8_t data[2];

  I2C.read(ENCODERADDR, MAGNITUDE, 2, data);

  return (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];
}

uint8_t uStepperEncoder::getAgc()
{
  uint8_t data;

  I2C.read(ENCODERADDR, MAGNITUDE, 1, &data);

  return data;
}

uint8_t uStepperEncoder::detectMagnet()
{
  uint8_t data;

  I2C.read(ENCODERADDR, AGC, 1, &data);

  data &= 0x38;         //For some reason the encoder returns random values on reserved bits. Therefore we make sure reserved bits are cleared before checking the reply !

  if (data == 0x08)
  {
    return 1;         //magnet too strong
  }

  else if (data == 0x10)
  {
    return 2;         //magnet too weak
  }

  else if (data == 0x20)
  {
    return 0;         //magnet detected and within limits
  }

  return 3;           //Something went horribly wrong !
}

