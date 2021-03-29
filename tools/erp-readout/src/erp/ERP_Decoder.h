#pragma once

#define ERP_ADC_RANGE (1024)  // 10 bits

// convert raw ADC wiper values [0...ERP_ADC_RANGE[ to a [-180, +180[ interval
// returns 0 on success, else a negative error code:
//  -1 : ADC values are unstable, not within accepted error bounds
int ERP_DecodeWipersToAngle(unsigned const w1, unsigned const w2, int* const pAngle);

// returns multiplier for mapping to 360 degrees
float ERP_AngleMultiplier360(void);

// returns the value which represents 360 degrees
int ERP_Scalefactor360(void);

// returns the distance between two absolute angles [-180, +180[
// can never be larger than 180
int ERP_GetAngleDifference(int const angle, int const previousAngle);
