#include <inttypes.h>
#include <avr/io.h>

#define I2CFREE 0        /**< I2C bus is not currently in use */
#define READ  1         /**< Value for RW bit in address field, to request a read */
#define WRITE 0         /**< Value for RW bit in address field, to request a write */
#define START  0x08       /**< start condition transmitted */
#define REPSTART 0x10     /**< repeated start condition transmitted */
#define TXADDRACK  0x18     /**< slave address plus write bit transmitted, ACK received */
#define TXDATAACK 0x28      /**< data transmitted, ACK received */
#define RXADDRACK 0x40      /**< slave address plus read bit transmitted, ACK received */
#define ACK 1         /**< value to indicate ACK for i2c transmission */
#define NACK 0          /**< value to indicate NACK for i2c transmission */

class i2cMaster
{
  private:
    uint8_t status;     /**< Contains the status of the I2C bus */


    /**
    * \brief Sends commands over the I2C bus.
    *
    * This function is used to send different commands over the I2C bus.
    *
    * \param cmd - Command to be send over the I2C bus.
    *
    */
    void cmd(uint8_t cmd);

  public:

    /**
    * \brief Constructor
    * 
    * This is the constructor, used to instantiate an I2C object. Under normal circumstances, this should not be needed by the
    * programmer of the arduino sketch, since this library already has a global object instantiation of this class, called "I2C".
    *
    */
    i2cMaster(void);
    
    /**
    * \brief Reads a byte from the I2C bus.
    *
    * This function requests a byte from the device addressed during the I2C transaction setup. The parameter "ack" is used to 
    * determine whether the device should keep sending data or not after the reception of the currently requested data byte.
    *
    * \param data   - Address of the variable to store the requested data byte
    * \param ack    - should be set to "ACK" if more bytes is wanted, and "NACK" if no more bytes should be send (without the quotes)
    *
    */
    bool readByte(bool ack, uint8_t *data);

    /**
    * \brief sets up I2C connection to device, reads a number of data bytes and closes the connection
    *
    * This function is used to perform a read transaction between the arduino and an I2C device. This function will perform everything 
    * from setting up the connection, reading the desired number of bytes and tear down the connection.
    *
    * \return 1     - Currently always returns this value. In the future this value will be used to indicate successful transactions.
    * \param slaveAddr  - 7 bit address of the device to read from
    * \param regAddr    - 8 bit address of the register to read from
    * \param numOfBytes - Number of bytes to read from the device
    * \param data     - Address of the array/string to store the bytes read. Make sure enough space are allocated before calling this function !  
    *
    */    
    bool read(uint8_t slaveAddr, uint8_t regAddr, uint8_t numOfBytes, uint8_t *data);

    /**
    * \brief sets up connection between arduino and I2C device.
    *
    * This function sets up the connection between the arduino and the I2C device desired to communicate with, by sending
    * a start condition on the I2C bus, followed by the device address and a read/write bit.
    *
    * \param addr   - Address of the device it is desired to communicate with
    * \param RW     - Can be set to "READ" to setup a read transaction or "WRITE" for a write transaction (without the quotes)
    * \return 1   - Connection properly set up
    * \return 0   - Connection failed
    *
    */
    bool start(uint8_t addr, bool RW);
    
    /**
    * \brief Restarts connection between arduino and I2C device.
    *
    * This function restarts the connection between the arduino and the I2C device desired to communicate with, by sending
    * a start condition on the I2C bus, followed by the device address and a read/write bit.
    *
    * \param addr   - Address of the device it is desired to communicate with
    * \param RW     - Can be set to "READ" to setup a read transaction or "WRITE" for a write transaction (without the quotes)
    * \return 1   - Connection properly set up
    * \return 0   - Connection failed
    *
    */
    bool restart(uint8_t addr, bool RW);
    
    /**
    * \brief Writes a byte to a device on the I2C bus.
    *
    * This function writes a byte to a device on the I2C bus.
    *
    * \param data - Byte to be written
    * \return 1 - Byte written successfully
    * \return 0 - transaction failed 
    *
    */
    bool writeByte(uint8_t data);
    
    /**
    * \brief sets up I2C connection to device, writes a number of data bytes and closes the connection
    *
    * This function is used to perform a write transaction between the arduino and an I2C device. This function will perform everything 
    * from setting up the connection, writing the desired number of bytes and tear down the connection.
    *
    * \return 1     - Currently always returns this value. In the future this value will be used to indicate successful transactions.
    * \param slaveAddr  - 7 bit address of the device to write to
    * \param regAddr    - 8 bit address of the register to write to
    * \param numOfBytes - Number of bytes to write to the device
    * \param data     - Address of the array/string containing data to write. 
    *
    */    
    bool write(uint8_t slaveAddr, uint8_t regAddr, uint8_t numOfBytes, uint8_t *data);

    /**
    * \brief Closes the I2C connection
    *
    * This function is used to close down the I2C connection, by sending a stop condition on the I2C bus.
    *
    * \return 1 - This is always returned Currently - in a later version, this should indicate that connection is successfully closed
    *
    */
    bool stop(void);
    
    /**
    * \brief Get current I2C status
    *
    * This function returns the status of the I2C bus.
    *
    * \return Status of the I2C bus. Refer to defines for possible status
    *
    */
    uint8_t getStatus(void);
    
    /**
    * \brief Setup TWI (I2C) interface
    *
    * This function sets up the TWI interface, and is automatically called in the instantiation of the uStepper encoder object.
    *
    */
    void begin(void);
};

extern i2cMaster I2C;
