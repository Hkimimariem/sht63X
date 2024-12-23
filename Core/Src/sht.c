#include "sht.h"
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c;
#define SHT30_I2C_ADDR                      0x44

#define SHT3X_CMD_READ_STATUS               0xF32D
#define SHT3X_CMD_CLEAR_STATUS              0x3041
#define SHT3X_COMMAND_HEATER_ENABLE         0x306d
#define SHT3X_COMMAND_HEATER_DISABLE         0x3066
#define STATUS_CMD_MSB 0xF3
#define STATUS_CMD_LSB 0x2D

// Private function prototypes
static void SHT_resetCmd(I2C_HandleTypeDef *hi2c);



uint8_t calculate_crc8(const uint8_t *data, uint8_t len)
{
    const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;

    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
        }
    }

    return crc;
}


void SHT_write(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd[2] = {0x24, 0x00};  // Command to measure temperature and humidity

    HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd, 2, 1000);
}

void SHT_read(I2C_HandleTypeDef *hi2c, float *temperature, float *humidity) {
    uint8_t data[6];
    float temp_raw, humidity_raw;
    uint8_t crc_temp_expected, crc_humidity_expected;
    uint8_t crc_temp_received, crc_humidity_received;

    HAL_I2C_Master_Receive(hi2c, (SHT30_I2C_ADDR << 1) | 0x01, data, 6, 1000);
    // Calculate CRC for temperature data (data[0] and data[1])
        crc_temp_expected = calculate_crc8(data, 2);
        crc_temp_received = data[2];

        // Calculate CRC for humidity data (data[3] and data[4])
        crc_humidity_expected = calculate_crc8(data + 3, 2);
        crc_humidity_received = data[5];

        // Verify CRCs
        if (crc_temp_expected != crc_temp_received || crc_humidity_expected != crc_humidity_received)
        {
            // Handle CRC error, e.g., return an error code or log a message
           printf("error");
            return;
        }
    temp_raw = data[0] * 256 + data[1];
    *temperature = (temp_raw * 175.0 / 65535.0) - 45.0;

    humidity_raw = data[3] * 256 + data[4];
    *humidity = (humidity_raw * 100.0) / 65535.0;
}

void SHT_reset(I2C_HandleTypeDef *hi2c) {
    SHT_resetCmd(hi2c);  // Perform the reset command
}

/*uint16_t SHT_readStatusRegister(I2C_HandleTypeDef *hi2c) {
    uint16_t status_register;
    uint8_t status_command[] = {(uint8_t)(SHT3X_CMD_READ_STATUS >> 8), (uint8_t)(SHT3X_CMD_READ_STATUS & 0xFF)};
    uint8_t status_data[3];

    HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, status_command, sizeof(status_command), HAL_MAX_DELAY);
    HAL_Delay(50);
    HAL_I2C_Master_Receive(hi2c, (SHT30_I2C_ADDR << 1) | 0x01, status_data, sizeof(status_data), HAL_MAX_DELAY);

    status_register = (status_data[0] << 8) | status_data[1];

    return status_register;
}*/

// Private function implementation
static void SHT_resetCmd(I2C_HandleTypeDef *hi2c) {
    uint8_t reset_cmd[2] = {0x30, 0xA2};

    HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, reset_cmd, 2, HAL_MAX_DELAY);
}





 void heater_on(I2C_HandleTypeDef *hi2c)
 {
     uint8_t cmd[2] = {0x30, 0x6D};  // Heater Enable Command

     // Send the command via I2C
     if (HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY) == HAL_OK)
     {
         // Command was successfully transmitted
         printf("Heater enabled.\n");
     }
     else
     {
         // Transmission failed
         printf("Failed to enable heater.\n");
     }
 }


 void heater_off(I2C_HandleTypeDef *hi2c)
 {
     uint8_t cmd[2] = {0x30, 0x66};  // Heater Disable Command

     // Send the command via I2C
     if (HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY) == HAL_OK)
     {
         // Command was successfully transmitted
         printf("Heater disabled.\n");
     }
     else
     {
         // Transmission failed
         printf("Failed to disable heater.\n");
     }
 }

 void heater_on_off_with_delay(uint8_t seconds,I2C_HandleTypeDef *hi2c)
 {
	 uint8_t heatTimeout;
	 heatTimeout = seconds;
 if (heatTimeout > 180)
 {
	   heatTimeout = 180;
 }

 uint8_t cmd1[2] = {0x30, 0x6D};  // Heater enable Command

     // heater_on
      HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd1, 2, HAL_MAX_DELAY);

     HAL_Delay(heatTimeout);  // Wait for the specified delay in milliseconds


     uint8_t cmd2[2] = {0x30, 0x66};  // Heater Disable Command

        // Send the command via I2C
        HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd2, 2, HAL_MAX_DELAY);


 }






 void SHT_writestatus(I2C_HandleTypeDef *hi2c) {
	 uint8_t cmd[2] = {0xF3, 0x2D};  // Command to measure temperature and humidity

     HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd, 2, 1000);
 }

 void read_status_register(uint16_t *status,I2C_HandleTypeDef *hi2c)
 {

     uint8_t data1[3];  // Buffer to hold the status register data and checksum

     uint8_t crc;
     // Receive the status register data
     HAL_I2C_Master_Receive(hi2c, (SHT30_I2C_ADDR << 1) | 0x01 , data1, 3, HAL_MAX_DELAY);

     // Combine the received bytes into a 16-bit status register value
    // *status = (data[0] << 8) | data[1];
     *status = data1[0] * 256 + data1[1];

     // Checksum validation

      crc = calculate_crc8(data1, 2);
         if (crc != data1[2])
         {
             printf("CRC check failed.\n");
             return;
         }


     printf("Status register: 0x%04X\n", *status);
 }



 void Clear_Status_Register(I2C_HandleTypeDef *hi2c)
 {
	 uint8_t cmd[2] = {0x30, 0x41};
	 HAL_I2C_Master_Transmit(hi2c, SHT30_I2C_ADDR << 1, cmd, 2, 1000);
 }








