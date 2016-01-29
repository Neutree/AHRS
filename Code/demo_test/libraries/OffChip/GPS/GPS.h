#ifndef _GPS_H
#define _GPS_H
#include "stm32f10x.h"
#include "Vector3.h"
#include "TaskManager.h"

class GPS
{

public:
	 /// GPS status codes
	enum GPS_Status {
		NO_GPS = 0,             ///< No GPS connected/detected
		NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
		GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
		GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
		GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
		GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
	};

    struct GPS_State {
        uint8_t instance; // the instance number of this GPS

        // all the following fields must all be filled by the backend driver
        GPS_Status status;                  ///< driver fix status
        uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
        uint16_t time_week;                 ///< GPS week number
 //       Location location;                  ///< last fix location
        float ground_speed;                 ///< ground speed in m/sec
        int32_t ground_course_cd;           ///< ground course in 100ths of a degree
        uint16_t hdop;                      ///< horizontal dilution of precision in cm
        uint16_t vdop;                      ///< vertical dilution of precision in cm
        uint8_t num_sats;                   ///< Number of visible satelites
        Vector3f velocity;                  ///< 3D velocitiy in m/s, in NED format
        float speed_accuracy;
        float horizontal_accuracy;
        float vertical_accuracy;
        bool have_vertical_velocity:1;      ///< does this GPS give vertical velocity?
        bool have_speed_accuracy:1;
        bool have_horizontal_accuracy:1;
        bool have_vertical_accuracy:1;
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
    };

public:
	GPS();
	~GPS();
	GPS_Status Status();
	uint8_t NumberOfSatellite();
	uint32_t LastFixTimeMs();
	Vector3f Velocity();
};
#endif
