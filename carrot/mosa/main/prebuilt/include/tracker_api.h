
#ifndef _APP_API_INCLUDE_
#define _APP_API_INCLUDE_

/* APP */
extern void APP_setup(int product_mode, void (*power_chg_cb)(int plug_off));
extern int APP_command_reg(const char *name, const char *alias, int (*func)(int argc, char **argv), const char *help_str);
extern void APP_command_prompt(char *prompt);

extern void APP_init();
extern void APP_start();

/* IMU */
extern void IMU_setup(void (*imu_push_cb)(float f_acc[3], float f_gyro[3]));

/* GPS */
extern void GPS_setup(void (*gps_push_cb)(uint32_t lat, uint32_t lon, int32_t alt, uint32_t speed, uint32_t ac));
extern void GPS_navi_push(uint32_t lat, uint32_t lon, float f_macc[3], float f_mgyro[3]);

#endif

/* end of file */
