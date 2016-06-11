#include <inttypes.h>
#include <avr/io.h>

#define ENCODERADDR 0x36    /**< I2C address of the encoder chip */

#define ANGLE 0x0E        /**< Address of the register, in the encoder chip, containing the 8 least significant bits of the stepper shaft angle */
#define STATUS 0x0B       /**< Address of the register, in the encoder chip, containing information about whether a magnet has been detected or not */
#define AGC 0x1A        /**< Address of the register, in the encoder chip, containing information about the current gain value used in the encoder chip. This value should preferably be around 127 (Ideal case!) */
#define MAGNITUDE 0x1B      /**< Address of the register, in the encoder chip, containing the 8 least significant bits of magnetic field strength measured by the encoder chip */

#define ENCODERINTFREQ 244.3  /**< Frequency at which the encoder is sampled, for keeping track of angle moved and current speed */
#define ENCODERSPEEDCONSTANT ENCODERINTFREQ/360.0 /**< Constant to convert angle difference between two interrupts to speed in revolutions per second */

extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));

class uStepperEncoder
{
public:
  /**
  * \brief Constructor
  *
  * This is the constructor of the uStepperEncoder class.
  */
  uStepperEncoder(void);
  
  /**
  * \brief Measure the current shaft angle 
  *
  * This function reads the current angle of the motor shaft. The resolution of the angle returned by this function is 0.087890625 degrees (12 bits)
  * The Angle is read by means of the I2C interface, using the I2C interface implemented in this library.
  *
  * \return Floating point representation of the current motor shaft angle
  */
  float getAngle(void);
  
  /**
  * \brief  Measure the current speed of the motor 
  * 
  * This function returns the current speed of the motor. The speed is not calculated in this function, it is merely returning a variable.
  * The speed is calculated in the interrupt routine associated with timer1.
  *
  * \return Current speed in revolutions per second (RPS) 
  *
  */
  float getSpeed(void);
  
  /**
  * \brief  Measure the strength of the magnet
  *
  * This function returns the strength of the magnet
  *
  * \return Strength of magnet
  *
  */
  uint16_t getStrength(void);
  
  /**
  * \brief  Read the current AGC value of the encoder chip 
  *
  * This function returns the current value of the AGC register in the encoder chip (AS5600).
  * This value ranges between 0 and 255, and should preferably be as close to 128 as possible.
  *
  * \return current AGC value
  */
  uint8_t getAgc(void);
  
  /**
  * \brief Detect if magnet is present and within range
  *
  * This function detects whether the magnet is present, too strong or too weak.
  *
  * \return 0 - Magnet detected and within limits
  * \return 1 - Magnet too strong
  * \return 2 - Magnet too weak
  *
  */
  uint8_t detectMagnet(void);
  
  /**
  * \brief Measure the angle moved from reference position
  * 
  * This function measures the angle moved from the shaft reference position.
  * When the uStepper is first powered on, the reference position is reset to the current shaft position, meaning that this function
  * will return the angle rotated with respect to the angle the motor initially had. It should be noted that this function is absolute to
  * an arbitrary number of revolutions !
  * 
  * The reference position can be reset at any point in time, by use of the setHome() function.
  *
  * 
  *
  */
  float getAngleMoved(void);
  
  /**
  * \brief Setup the encoder
  * 
  * This function initializes all the encoder features.
  *
  */
  void setup(void);
  
  /**
  * \brief Define new reference(home) position
  *
  * This function redefines the reference position to the current angle of the shaft
  *
  */
  void setHome(void);

private:

  friend void TIMER1_COMPA_vect(void) __attribute__ ((signal));

  float encoderOffset;        /**< Angle of the shaft at the reference position. */
  volatile float oldAngle;      /**< Used to stored the previous measured angle for the speed measurement, and the calculation of angle moved from reference position */
  volatile float curSpeed;      /**< Variable used to store the last measured rotational speed of the motor shaft */ 
  volatile float angleMoved;      /**< Variable used to store that measured angle moved from the reference position */
};
