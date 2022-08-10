#include "stepper.h"

Stepper::Stepper(gpio_num_t step, gpio_num_t dir, gpio_num_t home, gpio_num_t CS, gpio_num_t MS, bool invert) {
	pinStep = step;
	pinDir = dir;
	pinHome = home;
	pinCS = CS;
	pinMS = MS;
	intrCount = 0;
	intrInterval = 100;
	position = 500;
	target = 0;
	force = 0;
	error = 0;
	forceError = 0;
	bPosError = false;
	bForceError = false;
	bPosMode = true;
	anglePrev = 0;
	stepsPer_mm = 200.0/8.0;
	invertPos = invert;
	transitionSpeed = 17; //deg/s
}

void Stepper::home() {
	speed = 10;
	homing = true;
	homed = false;
	target = 0;
	position = 500;
}

void Stepper::update(spi_device_handle_t spi) {
	if(!gpio_get_level(pinHome) && !homed) {
		homed = true;
		position = 0;
	}

	//get encoder angle
	spi_transaction_t t;
	gpio_set_level(pinCS, 0);
	memset(&t, 0, sizeof(t));
	t.length = 16;
	// 0x3fff = 11111111111111, set 14 to 1 for read, set parity bit to 1
	uint16_t cmd = 0x3fff|1<<14|1<<15;
	t.tx_buffer = &cmd;
	t.flags = SPI_TRANS_USE_RXDATA;
	spi_device_transmit(spi, &t);
	gpio_set_level(pinCS, 1);
	//check parity
	bool even = true;
	for(int j=0; j<2; j++) {
		for(int i=0; i<8; i++) {
    		if(t.rx_data[j] & 1<<i) even = !even;
		}
	}
	if(even) {
		uint16_t trim = 0b0011111111111111;
		uint16_t angle = ((t.rx_data[0]<<8) + t.rx_data[1]) & trim;
		float angleNow = (float)angle/16383.0*360.0;
		float tempPos;

		if(invertPos) {
			tempPos = -position;
		} else {
			tempPos = position;
		}

		//if angle went from near 360 to near 0 it is moving up
		if((angleNow < 45) && (anglePrev > 315)) {
			tempPos += fabs(angleNow - anglePrev + 360.0)/360.0*8.0;
		//if angle went from near 0 to near 360 it is moving down
		} else if((angleNow > 315) && (anglePrev < 45)) {
			tempPos -= fabs(angleNow - anglePrev - 360.0)/360.0*8.0;
		} else { //normal math works
			tempPos += (angleNow - anglePrev)/360.0*8.0;
		}
		anglePrev = angleNow;

		if(invertPos) {
			position = -tempPos;
		} else {
			position = tempPos;
		}
	}

	// update error
	error = target - position;
	forceError = targetForce - force;
	if(fabs(error) > 0.25) {
		bPosError = true;
	} else {
		bPosError = false;
	}
	if(fabs(forceError) > 0.5) {
		bForceError = true;
	} else {
		bForceError = false;
	}

	// update direction
	if (targetForce == 0) {
		bPosMode = true;
		if (error > 0) {
			gpio_set_level(pinDir, 0);
		} else {
			gpio_set_level(pinDir, 1);
		}
	} else {
		bPosMode = false;
		if (forceError > 0) {
			gpio_set_level(pinDir, 0);
		} else {
			gpio_set_level(pinDir, 1);
		}
	}

	if(bPosMode && fabs(error) <= 0.25) complete = true;
	// if(!bPosMode && fabs(forceError) <= 0.5) complete = true;

	//update speed
	if(speed > transitionSpeed) microstep(false);
	else microstep(true);
  intrInterval = 1.0/(speed*stepsPer_mm*TIMER_INTERVAL0_S);
}

void Stepper::microstep(bool on) {
	if(on) {
		gpio_set_level(pinMS, 1);
		stepsPer_mm = 200.0*16.0/8.0;
	} else {
		gpio_set_level(pinMS, 0);
		stepsPer_mm = 200.0/8.0;
	}
}