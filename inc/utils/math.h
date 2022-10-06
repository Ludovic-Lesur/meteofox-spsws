/*
 * math.h
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#ifndef __MATH_H__
#define __MATH_H__

#include "mode.h"
#include "types.h"

/*** MATH macros ***/

#define MATH_BINARY_MAX_LENGTH			32
#define MATH_DECIMAL_MAX_LENGTH			10
#define MATH_HEXADECIMAL_MAX_LENGTH		4
#define MATH_BYTE_MAX					0xFF

#if (defined CM || defined ATM)
static const int16_t MATH_COS_TABLE[360] = {
	1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988,
	985, 982, 978, 974, 970, 966, 961, 956, 951, 946,
	940, 934, 927, 921, 914, 906, 899, 891, 883, 875,
	866, 857, 848, 839, 829, 819, 809, 799, 788, 777,
	766, 755, 743, 731, 719, 707, 695, 682, 669, 656,
	643, 629, 616, 602, 588, 574, 559, 545, 530, 515,
	500, 485, 469, 454, 438, 423, 407, 391, 375, 358,
	342, 326, 309, 292, 276, 259, 242, 225, 208, 191,
	174, 156, 139, 122, 105, 87, 70, 52, 35, 17,
	0, -17, -35, -52, -70, -87, -105, -122, -139, -156,
	-174, -191, -208, -225, -242, -259, -276, -292, -309, -326,
	-342, -358, -375, -391, -407, -423, -438, -454, -469, -485,
	-500, -515, -530, -545, -559, -574, -588, -602, -616, -629,
	-643, -656, -669, -682, -695, -707, -719, -731, -743, -755,
	-766, -777, -788, -799, -809, -819, -829, -839, -848, -857,
	-866, -875, -883, -891, -899, -906, -914, -921, -927, -934,
	-940, -946, -951, -956, -961, -966, -970, -974, -978, -982,
	-985, -988, -990, -993, -995, -996, -998, -999, -999, -1000,
	-1000, -1000, -999, -999, -998, -996, -995, -993, -990, -988,
	-985, -982, -978, -974, -970, -966, -961, -956, -951, -946,
	-940, -934, -927, -921, -914, -906, -899, -891, -883, -875,
	-866, -857, -848, -839, -829, -819, -809, -799, -788, -777,
	-766, -755, -743, -731, -719, -707, -695, -682, -669, -656,
	-643, -629, -616, -602, -588, -574, -559, -545, -530, -515,
	-500, -485, -469, -454, -438, -423, -407, -391, -375, -358,
	-342, -326, -309, -292, -276, -259, -242, -225, -208, -191,
	-174, -156, -139, -122, -105, -87, -70, -52, -35, -17,
	0, 17, 35, 52, 70, 87, 105, 122, 139, 156,
	174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
	342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
	500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
	643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
	766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
	866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
	940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
	985, 988, 990, 993, 995, 996, 998, 999, 999, 1000
};

static const int16_t MATH_SIN_TABLE[360] = {
	0, 17, 35, 52, 70, 87, 105, 122, 139, 156,
	174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
	342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
	500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
	643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
	766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
	866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
	940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
	985, 988, 990, 993, 995, 996, 998, 999, 999, 1000,
	1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988,
	985, 982, 978, 974, 970, 966, 961, 956, 951, 946,
	940, 934, 927, 921, 914, 906, 899, 891, 883, 875,
	866, 857, 848, 839, 829, 819, 809, 799, 788, 777,
	766, 755, 743, 731, 719, 707, 695, 682, 669, 656,
	643, 629, 616, 602, 588, 574, 559, 545, 530, 515,
	500, 485, 469, 454, 438, 423, 407, 391, 375, 358,
	342, 326, 309, 292, 276, 259, 242, 225, 208, 191,
	174, 156, 139, 122, 105, 87, 70, 52, 35, 17,
	0, -17, -35, -52, -70, -87, -105, -122, -139, -156,
	-174, -191, -208, -225, -242, -259, -276, -292, -309, -326,
	-342, -358, -375, -391, -407, -423, -438, -454, -469, -485,
	-500, -515, -530, -545, -559, -574, -588, -602, -616, -629,
	-643, -656, -669, -682, -695, -707, -719, -731, -743, -755,
	-766, -777, -788, -799, -809, -819, -829, -839, -848, -857,
	-866, -875, -883, -891, -899, -906, -914, -921, -927, -934,
	-940, -946, -951, -956, -961, -966, -970, -974, -978, -982,
	-985, -988, -990, -993, -995, -996, -998, -999, -999, -1000,
	-1000, -1000, -999, -999, -998, -996, -995, -993, -990, -988,
	-985, -982, -978, -974, -970, -966, -961, -956, -951, -946,
	-940, -934, -927, -921, -914, -906, -899, -891, -883, -875,
	-866, -857, -848, -839, -829, -819, -809, -799, -788, -777,
	-766, -755, -743, -731, -719, -707, -695, -682, -669, -656,
	-643, -629, -616, -602, -588, -574, -559, -545, -530, -515,
	-500, -485, -469, -454, -438, -423, -407, -391, -375, -358,
	-342, -326, -309, -292, -276, -259, -242, -225, -208, -191,
	-174, -156, -139, -122, -105, -87, -70, -52, -35, -17
};
#endif

/*** MATH structures ***/

typedef enum {
	MATH_SUCCESS = 0,
	MATH_ERROR_NULL_PARAMETER,
	MATH_ERROR_OVERFLOW,
	MATH_ERROR_UNDEFINED,
	MATH_ERROR_SIGN_BIT,
	MATH_ERROR_BASE_LAST = 0x0100
} MATH_status_t;

/*** MATH functions ***/

MATH_status_t MATH_min_u8(uint8_t* data, uint8_t data_length, uint8_t* result);
MATH_status_t MATH_min_u16(uint16_t* data, uint8_t data_length, uint16_t* result);
MATH_status_t MATH_min_u32(uint32_t* data, uint8_t data_length, uint32_t* result);

MATH_status_t MATH_max_u8(uint8_t* data, uint8_t data_length, uint8_t* result);
MATH_status_t MATH_max_u16(uint16_t* data, uint8_t data_length, uint16_t* result);
MATH_status_t MATH_max_u32(uint32_t* data, uint8_t data_length, uint32_t* result);

MATH_status_t MATH_average_u8(uint8_t* data, uint8_t data_length, uint8_t* result);
MATH_status_t MATH_average_u16(uint16_t* data, uint8_t data_length, uint16_t* result);
MATH_status_t MATH_average_u32(uint32_t* data, uint8_t data_length, uint32_t* result);

MATH_status_t MATH_median_filter_u8(uint8_t* data, uint8_t median_length, uint8_t average_length, uint8_t* result);
MATH_status_t MATH_median_filter_u16(uint16_t* data, uint8_t median_length, uint8_t average_length, uint16_t* result);
MATH_status_t MATH_median_filter_u32(uint32_t* data, uint8_t median_length, uint8_t average_length, uint32_t* result);

MATH_status_t MATH_pow_10(uint8_t power, uint32_t* result);
MATH_status_t MATH_abs(int32_t x, uint32_t* result);
MATH_status_t MATH_atan2(int32_t x, int32_t y, uint32_t* alpha);

MATH_status_t MATH_two_complement(uint32_t value, uint8_t sign_bit_position, int32_t* result);
MATH_status_t MATH_one_complement(int32_t value, uint8_t sign_bit_position, uint32_t* result);

#define MATH_status_check(error_base) { if (math_status != MATH_SUCCESS) { status = error_base + math_status; goto errors; }}
#define MATH_error_check() { ERROR_status_check(math_status, MATH_SUCCESS, ERROR_BASE_MATH); }
#define MATH_error_check_print() { ERROR_status_check_print(math_status, MATH_SUCCESS, ERROR_BASE_MATH); }

#endif /* __MATH_H__ */
