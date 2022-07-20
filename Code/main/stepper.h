#ifndef STEPPER_H
#define STEPPER_H
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

class Stepper {
	public:
		int pinStep, pinDir, pinHome, pinCS, pinMS;
		int intrCount;
		int intrInterval;
		double position, target;
		double error;
		bool complete, homing, homed;
		double anglePrev;
		int stepsPer_mm;

		Stepper(int pinStep, int pinDir, int pinHome, int pinCS, int pinMS);

		void home();
		void update(spi_device_handle_t spi);
		void microstep(bool on);
};

#endif