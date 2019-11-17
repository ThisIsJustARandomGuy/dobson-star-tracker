#pragma once

#include "Observer.h"


class FixedObserver : public Observer
{
public:
	FixedObserver(const float altitude, const float latitude, const float longitude, const int year, const int month, const int day, const int hour, const int minute, const int second);

	void initialize();

	void updatePosition();

	bool hasValidPosition();

	void printDebugInfo();
};

