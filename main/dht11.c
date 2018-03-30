// Copyright (c) 2015 Adys Tech
// Author: Adarsha (mvadu)
// Based on code by : Tony DiCola of Adafruit https://github.com/adafruit/Adafruit_Python_DHT
// Tony is using average counter value for 50uS pulse in each bit as a way to identify 0 or 1.
// This code uses half the average of counter value for the 80uS calibration pulses before first data bit.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// This is the only processor specific magic value, the maximum amount of time to
// spin in a loop before bailing out and considering the read a timeout.  This should
// be a high value, but if you're running on a much faster platform than a Raspberry
// Pi then it might need to be increased.
#define DHT_MAXCOUNT 32000

// Define errors and return values.
#define DHT_ERROR_TIMEOUT -1
#define DHT_ERROR_CHECKSUM -2
#define DHT_ERROR_ARGUMENT -3
#define DHT_SUCCESS 0

// Number of bit pulses to expect from the DHT.  40 pulses to represent
// the data after 2 ~80 pulses for calibration purposes.
#define DHT_PULSES 40


int dht_read(int pin, int* humidity, int* temperature) {
	// Validate humidity and temperature arguments and set them to zero.
	if (humidity == NULL || temperature == NULL) {
		return DHT_ERROR_ARGUMENT;
	}
	*temperature = 0;
	*humidity = 0;

	// Store the count that each DHT bit pulse is low and high.
	// Make sure array is initialized to start at zero.
	int bitPulseCounts[DHT_PULSES] = { 0 };

	int timingPulseCounts[3] = { 0 };

	// Set pin to output.
	gpio_pad_select_gpio(pin);
	gpio_set_direction(pin, GPIO_MODE_OUTPUT);

	// Set pin high for ~500 milliseconds, allow the sensor to stabilize.
	gpio_set_level(pin, 1);
	vTaskDelay(600 / portTICK_PERIOD_MS);

	// The next calls are timing critical and care should be taken
	// to ensure no unnecessary work is done below.

	// Set pin low for ~20 milliseconds.
	gpio_set_level(pin, 0);
	vTaskDelay(20 / portTICK_PERIOD_MS);

	// Set pin at input, Signal pin goes back up high due to pull up resistor
	gpio_set_direction(pin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

	// Need a very short delay before reading pins or else value is sometimes still low.
	for (volatile int i = 0; i < 50 && !gpio_get_level(pin); ++i) {

	}

	// Wait for DHT to pull pin low, should happen in 20-40 uS

	while (gpio_get_level(pin)) {
		if (++timingPulseCounts[0] >= DHT_MAXCOUNT) {
			// Timeout waiting for response.
			return DHT_ERROR_TIMEOUT;
		}
	}

	//edit begins
	//once DHT11 pulls down, it will keep it there for 80uS
	while (!gpio_get_level(pin)) {
		if (++timingPulseCounts[1] >= DHT_MAXCOUNT) {
			// Timeout waiting for response.
			return DHT_ERROR_TIMEOUT;
		}
	}
	//then it puls high and keeps it high for 80uS
	while (gpio_get_level(pin)) {
		if (++timingPulseCounts[2] >= DHT_MAXCOUNT) {
			// Timeout waiting for response.
			return DHT_ERROR_TIMEOUT;
		}
	}

	//during data transfer, 0 is ~26uS, and 1 is ~70uS of HIGH which follows 50uS of low
	//Every bit is 50uS low + 26-70uS High.

	// Record pulse widths for the expected result bits.
	for (int i = 0; i < DHT_PULSES; i++) {
		volatile int timingCounter=0;
		// Count how long pin is low and store in bitPulseCounts[i] should be ~50uS
		while (!gpio_get_level(pin)) {
			if (++timingCounter >= DHT_MAXCOUNT) {
				// Timeout waiting for response.
				return DHT_ERROR_TIMEOUT;
			}
		}

		// Count how long pin is high and store in bitPulseCounts[i+1] can be 26-70uS depending on 0 or 1 bit
		while (gpio_get_level(pin)) {
			if (++bitPulseCounts[i] >= DHT_MAXCOUNT) {
				// Timeout waiting for response.
				return DHT_ERROR_TIMEOUT;
			}
		}
	}

	// Done with timing critical code, now interpret the results.

	//compute the average time taken for 40uS pulse, which is average half time for first two pulses
	uint32_t threshold = 0;
	threshold = (timingPulseCounts[1] + timingPulseCounts[2]) / 4;

	/* Print counters for each bits
	for (int i = 0; i < DHT_PULSES; i++) {
		printf("%d: %d %d	%d\n", i, bitPulseCounts[i], timingPulseCounts[i + 3], bitPulseCounts[i] > threshold ? 1 : 0);
	}
	*/

	// Interpret each high pulse as a 0 or 1 by comparing it to the 40us reference.
	// If the count is less than 40us it must be a ~28us 0 pulse, and if it's higher
	// then it must be a ~70us 1 pulse.
	int data[5] = { 0 };
	int index = 0, bit = 7;
	for (int i = 0; i < DHT_PULSES; i++)
	{
		if (bitPulseCounts[i] >= threshold)
			data[index] |= (1 << bit);
		if (bit == 0)   // next byte?
		{
			bit = 7;    // restart at MSB
			index++;      // next byte!
		}
		else bit--;
	}

	// Useful debug info:
	//printf("Data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3], data[4]);

	// Verify checksum of received data.
	if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
		// Get humidity and temp for DHT11 sensor.
		//*humidity = (float)data[0];
		//*temperature = (float)data[2];
		*humidity = data[0];
		*temperature = data[2];

		return DHT_SUCCESS;
	}
	else {
		return DHT_ERROR_CHECKSUM;
	}
}

//can have 0.6544C variance
// reference: http://en.wikipedia.org/wiki/Dew_point
//ref http://playground.arduino.cc/Main/DHT11Lib
double dewPointFast(double celsius, double humidity)
{
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
	double dP = (b * temp) / (a - temp);
	return dP;
}
//http://forum.arduino.cc/index.php/topic,107569.msg807598.html#msg807598
double humidex(double tempC, double DewPoint)
{
  double e = 19.833625 - 5417.753 /(273.16 + DewPoint);
  double h = tempC + 3.3941 * exp(e) - 5.555;
  return h;
}

float ParseFloat(uint8_t intData, uint8_t fractionData)
{
	float val=fractionData * 1.0f;
	while(val>1)
		val/=10.0;
	return val + (float)intData;
}
