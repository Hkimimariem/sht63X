#ifndef SHT_H_
#define SHT_H_

#include "main.h"  // Include necessary HAL headers

// Function prototypes
void SHT_init(I2C_HandleTypeDef *hi2c);
void SHT_write(I2C_HandleTypeDef *hi2c);
void SHT_read(I2C_HandleTypeDef *hi2c, float *temperature, float *humidity);
void SHT_reset(I2C_HandleTypeDef *hi2c);
//uint16_t SHT_readStatusRegister(I2C_HandleTypeDef *hi2c);
uint8_t calculate_crc8(const uint8_t *data, uint8_t len);
void heater_on(I2C_HandleTypeDef *hi2c);
void heater_off(I2C_HandleTypeDef *hi2c);
void read_status_register(uint16_t *status,I2C_HandleTypeDef *hi2c);
void SHT_writestatus(I2C_HandleTypeDef *hi2c);
void heater_on_off_with_delay(uint8_t seconds,I2C_HandleTypeDef *hi2c);
void Clear_Status_Register(I2C_HandleTypeDef *hi2c);
#endif /* SHT_H_ */
