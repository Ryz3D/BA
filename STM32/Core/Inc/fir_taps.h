/*
 * Copyright (c) 2024 Mirco Heitmann
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 * 
 * fir_taps.h
 */

#ifndef INC_FIR_TAPS_H_
#define INC_FIR_TAPS_H_

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1500 Hz
 gain = 1
 desired ripple = 0.5 dB
 actual ripple = 0.0782 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -80 dB
 actual attenuation = -93.5395 dB

 */

#define FIR_TAPS1_LEN 128

const int16_t fir_taps1[FIR_TAPS1_LEN] =
	{ 1, 2, 2, 1, -2, -8, -16, -23, -27, -25, -15, 1, 19, 31, 31, 18, -7, -34, -51, -49, -24, 17, 58, 81, 72, 28, -37, -96, -123, -98, -24, 74, 154, 177, 123, 4, -136, -237, -245, -143, 41, 237, 355, 329, 148, -133, -399, -528, -434, -121, 310, 682, 811, 584, 22, -701, -1294, -1439, -906, 353, 2159, 4139, 5826, 6796, 6796, 5826, 4139, 2159, 353, -906, -1439, -1294, -701, 22, 584, 811, 682, 310, -121, -434, -528, -399, -133, 148, 329, 355, 237, 41, -143, -245, -237, -136, 4, 123, 177, 154, 74, -24, -98, -123, -96, -37, 28, 72, 81, 58, 17, -24, -49, -51, -34, -7, 18, 31, 31, 19, 1, -15, -25, -27, -23, -16, -8, -2, 1, 2, 2, 1 };

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1600 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.5866 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -90 dB
 actual attenuation = -92.1561 dB

 */

#define FIR_TAPS2_LEN 128

const int16_t fir_taps2[FIR_TAPS2_LEN] =
	{ 0, -1, -6, -16, -33, -57, -86, -111, -125, -121, -93, -46, 11, 61, 87, 81, 41, -17, -73, -101, -88, -34, 41, 106, 131, 99, 18, -81, -156, -168, -104, 16, 144, 221, 207, 94, -77, -235, -304, -241, -56, 180, 364, 404, 258, -30, -349, -552, -525, -242, 208, 642, 852, 690, 150, -604, -1269, -1497, -1024, 223, 2071, 4130, 5900, 6922, 6922, 5900, 4130, 2071, 223, -1024, -1497, -1269, -604, 150, 690, 852, 642, 208, -242, -525, -552, -349, -30, 258, 404, 364, 180, -56, -241, -304, -235, -77, 94, 207, 221, 144, 16, -104, -168, -156, -81, 18, 99, 131, 106, 41, -34, -88, -101, -73, -17, 41, 81, 87, 61, 11, -46, -93, -121, -125, -111, -86, -57, -33, -16, -6, -1, 0 };

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1800 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.6606 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -41.1246 dB

 */

#define FIR_TAPS3_LEN 128

const int16_t fir_taps3[FIR_TAPS3_LEN] =
	{ 221, 162, 155, 100, 13, -74, -125, -120, -63, 16, 75, 82, 30, -55, -130, -151, -103, -6, 90, 134, 97, -8, -128, -199, -177, -67, 80, 187, 191, 81, -93, -240, -279, -176, 24, 222, 309, 226, 1, -256, -407, -358, -112, 217, 457, 465, 206, -212, -578, -681, -421, 117, 680, 956, 728, 7, -925, -1595, -1528, -456, 1536, 3990, 6218, 7541, 7541, 6218, 3990, 1536, -456, -1528, -1595, -925, 7, 728, 956, 680, 117, -421, -681, -578, -212, 206, 465, 457, 217, -112, -358, -407, -256, 1, 226, 309, 222, 24, -176, -279, -240, -93, 81, 191, 187, 80, -67, -177, -199, -128, -8, 97, 134, 90, -6, -103, -151, -130, -55, 30, 82, 75, 16, -63, -120, -125, -74, 13, 100, 155, 162, 221 };

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1700 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.8219 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -70 dB
 actual attenuation = -69.2299 dB

 */

#define FIR_TAPS4_LEN 128

const int16_t fir_taps4[FIR_TAPS4_LEN] =
	{ -6, 7, 27, 63, 109, 154, 185, 185, 147, 77, -7, -79, -113, -97, -38, 41, 102, 118, 77, -4, -90, -139, -123, -45, 63, 151, 172, 109, -15, -145, -216, -186, -60, 111, 244, 269, 161, -41, -246, -350, -290, -75, 205, 417, 443, 246, -105, -454, -622, -494, -87, 439, 836, 872, 449, -323, -1134, -1573, -1277, -94, 1835, 4083, 6065, 7223, 7223, 6065, 4083, 1835, -94, -1277, -1573, -1134, -323, 449, 872, 836, 439, -87, -494, -622, -454, -105, 246, 443, 417, 205, -75, -290, -350, -246, -41, 161, 269, 244, 111, -60, -186, -216, -145, -15, 109, 172, 151, 63, -45, -123, -139, -90, -4, 77, 118, 102, 41, -38, -97, -113, -79, -7, 77, 147, 185, 185, 154, 109, 63, 27, 7, -6 };

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1850 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 3.6486 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -45 dB
 actual attenuation = -46.1339 dB

 */

#define FIR_TAPS5_LEN 128

const int16_t fir_taps5[FIR_TAPS5_LEN] =
	{ -21, -247, -401, -607, -755, -788, -673, -427, -114, 170, 337, 339, 195, -21, -207, -279, -210, -39, 146, 252, 224, 79, -110, -242, -248, -120, 77, 240, 279, 168, -40, -238, -317, -227, -5, 235, 363, 300, 66, -224, -416, -395, -151, 202, 480, 520, 272, -161, -559, -697, -456, 86, 675, 985, 782, 67, -883, -1585, -1549, -490, 1511, 3993, 6254, 7599, 7599, 6254, 3993, 1511, -490, -1549, -1585, -883, 67, 782, 985, 675, 86, -456, -697, -559, -161, 272, 520, 480, 202, -151, -395, -416, -224, 66, 300, 363, 235, -5, -227, -317, -238, -40, 168, 279, 240, 77, -120, -248, -242, -110, 79, 224, 252, 146, -39, -210, -279, -207, -21, 195, 339, 337, 170, -114, -427, -673, -788, -755, -607, -401, -247, -21 };

/*

 FIR filter designed with
 http://t-filter.engineerjs.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 2000 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.0981 dB

 * 2500 Hz - 8000 Hz
 gain = 0
 desired attenuation = -80 dB
 actual attenuation = -97.6857 dB

 */

#define FIR_TAPS6_LEN 128

const int16_t fir_taps6[FIR_TAPS6_LEN] =
	{ 0, -2, -6, -14, -23, -30, -29, -17, 2, 20, 27, 16, -8, -30, -36, -16, 19, 47, 47, 12, -39, -71, -57, 1, 69, 98, 60, -29, -112, -127, -51, 76, 167, 150, 19, -147, -231, -157, 46, 246, 296, 134, -154, -373, -350, -61, 323, 531, 377, -90, -579, -725, -347, 384, 994, 987, 193, -1021, -1860, -1501, 418, 3494, 6663, 8668, 8668, 6663, 3494, 418, -1501, -1860, -1021, 193, 987, 994, 384, -347, -725, -579, -90, 377, 531, 323, -61, -350, -373, -154, 134, 296, 246, 46, -157, -231, -147, 19, 150, 167, 76, -51, -127, -112, -29, 60, 98, 69, 1, -57, -71, -39, 12, 47, 47, 19, -16, -36, -30, -8, 16, 27, 20, 2, -17, -29, -30, -23, -14, -6, -2, 0 };

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 400 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 500 Hz - 800 Hz
 gain = 0.25
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 1000 Hz - 1500 Hz
 gain = 0.5
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 1700 Hz - 3000 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 3200 Hz - 8000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -44.9924 dB

 */

#define FIR_TAPS7_LEN 128

const int16_t fir_taps7[FIR_TAPS7_LEN] =
	{ -288, -469, -462, -140, 315, 565, 422, 40, -232, -190, 48, 217, 208, 158, 231, 391, 448, 305, 96, 17, 99, 186, 165, 116, 180, 327, 356, 149, -143, -258, -148, -32, -104, -255, -219, 33, 184, -61, -551, -810, -599, -225, -154, -382, -397, 129, 801, 883, 235, -380, -147, 755, 1281, 833, 162, 551, 2044, 2932, 1517, -1566, -3279, -994, 4505, 9158, 9158, 4505, -994, -3279, -1566, 1517, 2932, 2044, 551, 162, 833, 1281, 755, -147, -380, 235, 883, 801, 129, -397, -382, -154, -225, -599, -810, -551, -61, 184, 33, -219, -255, -104, -32, -148, -258, -143, 149, 356, 327, 180, 116, 165, 186, 99, 17, 96, 305, 448, 391, 231, 158, 208, 217, 48, -190, -232, 40, 422, 565, 315, -140, -462, -469, -288 };

const int16_t *fir_taps_types[] = {
	NULL,
	fir_taps1,
	fir_taps2,
	fir_taps3,
	fir_taps4,
	fir_taps5,
	fir_taps6,
	fir_taps7,
};

uint16_t fir_taps_lens[] = {
	0,
	FIR_TAPS1_LEN,
	FIR_TAPS2_LEN,
	FIR_TAPS3_LEN,
	FIR_TAPS4_LEN,
	FIR_TAPS5_LEN,
	FIR_TAPS6_LEN,
	FIR_TAPS7_LEN,
};

#endif /* INC_FIR_TAPS_H_ */
