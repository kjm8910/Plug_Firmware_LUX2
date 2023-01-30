
/* public include file */

#ifndef _MOS_PUB_INCLUDE_
#define _MOS_PUB_INCLUDE_


/* define */


/* types */


/* function prototype */

extern int mos_init();
extern void mos_shutdown();
extern int mos_imu_add(float f_acc[3], float f_gyro[3]);
extern int mos_gps_add(uint32_t lat, uint32_t lon, int32_t alt, uint32_t speed, uint32_t acc);


#endif

/* end of file */
