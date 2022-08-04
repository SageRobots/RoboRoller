#ifndef STEPPER_H
#define STEPPER_H
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define TIMER_INTERVAL0_S    0.0002

class Stepper {
	public:
		gpio_num_t pinStep, pinDir, pinHome, pinCS, pinMS;
		int intrCount;
		int intrInterval;
		float position, target, targetForce;
		float error, forceError;
		bool complete, homing, homed, bPosError, bForceError, bPosMode;
		float anglePrev;
		int stepsPer_mm;
		bool invertPos;
		float speed, transitionSpeed;

		Stepper(gpio_num_t pinStep, gpio_num_t pinDir, gpio_num_t pinHome, gpio_num_t pinCS, gpio_num_t pinMS, bool invert);

		void home();
		void update(spi_device_handle_t spi);
		void microstep(bool on);
};

#endif