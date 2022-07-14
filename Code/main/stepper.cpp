#include "stepper.h"

Stepper::Stepper(int pinStep, int pinDir, int pinHome, int pinCS, int pinMS) {
	pinDir = pinDir;
	pinHome = pinHome;
	pinCS = pinCS;
	intrCount = 0;
	intrInterval = 100;
	position = 500;
	target = 0;
	error = 0;
	anglePrev = 0;
}

void Stepper::home() {
	homing = true;
	homed = false;
	target = 0;
	position = 500;
}

void Stepper::update(spi_device_handle_t spi) {
	if(gpio_get_level(pinHome) && !homed) {
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
		position += (angleNow - anglePrev)/360.0*8.0; //8mm per rev
		anglePrev = angleNow;
	}
}

void Stepper::microstep(bool on) {
	if(on) {
		gpio_set_level(pinMS, 1);
	} else {
		gpio_set_level(pinMS, 0);
	}
}