#include "robot_frame_config.h"

#if BMI088_Frame
#else

#include "BMI088Middleware.h"

SPI_HandleTypeDef *BMI088_SPI;

void BMI088_ACCEL_NS_L( void )
{
	HAL_GPIO_WritePin( ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET );
}

void BMI088_ACCEL_NS_H( void )
{
	HAL_GPIO_WritePin( ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET );
}

void BMI088_GYRO_NS_L( void )
{
	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET );
}

void BMI088_GYRO_NS_H( void )
{
	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET );
}

uint8_t BMI088_Read_Write_Byte( uint8_t txdata )
{
	uint8_t rx_data;
	HAL_SPI_TransmitReceive( BMI088_SPI, &txdata, &rx_data, 1, 1000 );
	return rx_data;
}

#endif