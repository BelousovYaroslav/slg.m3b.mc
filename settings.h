#ifndef SETTINGS_H
#define SETTINGS_H

#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000

void load_params( void);
void SaveThermoCalibPoint( void);
void save_params( void);

#endif