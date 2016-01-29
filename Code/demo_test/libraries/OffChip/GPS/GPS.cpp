#include "GPS.h"

GPS::GPS() {
}

GPS::~GPS() {
}

GPS::GPS_Status GPS::Status() {
	return GPS_OK_FIX_3D;
}

uint8_t GPS::NumberOfSatellite() {

	return 7;
}

uint32_t GPS::LastFixTimeMs() {
	return TaskManager::Time()*1000;
}

Vector3f GPS::Velocity() {
	Vector3f temp;
	return temp;
}
