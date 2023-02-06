

#ifndef _MOSA_PROTO_INCLUDE_
#define _MOSA_PROTO_INCLUDE_

/* main.c */
extern void mosa_init();
extern int mosa_cmd_init();
extern void mosa_main_task(void *args);
extern void mosa_iope_task(void *args);
extern void mosa_power_off_cb(int plug_off);
extern void mosa_main_timer_cb(void *args);

/* IMU.c */
extern void mosa_imu_init();
extern void mosa_imu_push_cb(float f_acc[3], float f_gyro[3]);
extern int mosa_imu_navi_get(float f_acc[3], float f_gyro[3]);
extern int mosa_iope_navi(void);


/* GPS.c */
extern void mosa_gps_push_cb(uint32_t lat, uint32_t lon, int32_t alt, uint32_t speed, uint32_t acc);

/* time.c */
extern void mosa_time_init(uint32_t *time);
extern uint32_t mosa_time_elapsed(uint32_t *time);
extern uint32_t mosa_time_get();


#endif          // _MOSA_PROTO_INCLUDE_

/* end of file */
