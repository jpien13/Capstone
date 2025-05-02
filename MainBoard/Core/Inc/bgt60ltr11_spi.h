/*
 * bgt60ltr11_spi.h
 *
 *  Created on: May 2, 2025
 *      Author: root
 */

#ifndef INC_BGT60LTR11_SPI_H_
#define INC_BGT60LTR11_SPI_H_

#include "stdint.h"

typedef enum {
    RADAR_1 = 0,
    RADAR_2 = 1
} RadarId_t;


// Return HAL_OK or HAL_ERROR
uint8_t bgt60ltr11_spi_read(RadarId_t radar_id, uint8_t reg_addr, uint16_t *data);
uint8_t bgt60ltr11_spi_write(RadarId_t radar_id, uint8_t reg_addr, uint16_t data);
uint8_t bgt60ltr11_adc_status(RadarId_t radar_id);
uint8_t bgt60ltr11_soft_reset(RadarId_t radar_id, uint8_t wait);
uint8_t bgt60ltr11_pulsed_mode_init(RadarId_t radar_id);
uint8_t bgt60ltr11_get_RAW_data(RadarId_t radar_id, uint16_t *ifi, uint16_t *ifq);
uint8_t bgt60ltr11_HW_reset(RadarId_t radar_id);
uint8_t bgt60ltr11_pulsed_mode_init_extended_range(RadarId_t radar_id);
uint8_t bgt60ltr11_set_max_range(RadarId_t radar_id);



#endif /* INC_BGT60LTR11_SPI_H_ */
