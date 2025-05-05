/*
 * fft_visualize.c
 *
 *  Created on: May 5, 2025
 *      Author: root
 */

#include "arm_math.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//void visualize_129_frequencies(const float32_t *spectrum){
//
//	const int32_t length = 129;
//
//	if (spectrum == NULL){
//		printf("Error: Spectrum data is null!!!\r\n");
//		return;
//	}
//
//    for (uint32_t i = 0; i < length; i++) {
//    	uint8_t bar_magnitude = (uint8_t)spectrum[i];
//    	printf("[%3lu]: ", i);
//    	for (uint8_t j = 0; j < bar_magnitude; j++) {
//			printf("#");
//		}
//    	printf("\r\n");
//    }
//    printf("\r\n");
//}


//void visualize_129_frequencies(const float32_t *spectrum){
//
//	static const char LUT[10] = {' ', '.', ',', '~', '/', 'l', 'L', 'P', 'R', 'X'};
//
//	const int32_t length = 129;
//
//	if (spectrum == NULL){
//		printf("Error: Spectrum data is null!!!\r\n");
//		return;
//	}
//
//	float32_t max_value = 0.0f;
//	uint32_t max_index = 0;
//
//	for (uint32_t i = 0; i < length; i++) {
//		if (spectrum[i] > max_value) {
//			max_value = spectrum[i];
//			max_index = i;
//		}
//	}
//
//	for (uint32_t i = 0; i < length; i++) {
//		float32_t normalized = spectrum[i] / max_value;
//		uint8_t magnitude = (uint8_t)(normalized * 9.0f);  // Scale to 0-9
//
//		if (magnitude == 0) {
//			printf(" ");
//		} else {
//			printf("%c", LUT[magnitude]);
//		}
//	}
//	printf("\r\n");
//}
void visualize_129_frequencies(const float32_t *spectrum, uint32_t length, uint8_t include_header){
    if (spectrum == NULL || length == 0) {
    	printf("Error: Spectrum data is null or length is 0!!!\r\n");
        return;
    }

    printf("\r\n=== FFT DATA FOR EXCEL ===\r\n");

    if (include_header) {
        printf("Index,Frequency,Magnitude\r\n");
    }

    // (assuming 1000 Hz sampling rate)
    float32_t freq_resolution = 1000.0f / (2 * (length - 1));

    for (uint32_t i = 0; i < length; i++) {
        float32_t frequency = i * freq_resolution;
        printf("%lu,%.2f,%.6f\r\n", i, frequency, spectrum[i]);
    }

    printf("=== END OF FFT DATA ===\r\n\r\n");
}
