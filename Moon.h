/*
 * Moon.h
 *
 *  Created on: 03.08.2019
 *      Author: lukas
 */

#ifndef MOON_H_
#define MOON_H_

class Moon {
public:
	Moon();
	long getAzimuth();
	long getElevation();

private:
	long azimuth;
	long elevation;
};

#endif /* MOON_H_ */
