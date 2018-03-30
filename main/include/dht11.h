#ifndef DHT_READ_H
#define DHT_READ_H

// Read DHT sensor connected to GPIO pin (using BCM numbering).  Humidity and temperature will be
// returned in the provided parameters. If a successfull reading could be made a value of 0
// (DHT_SUCCESS) will be returned.  If there was an error reading the sensor a negative value will
// be returned.  Some errors can be ignored and retried, specifically DHT_ERROR_TIMEOUT or DHT_ERROR_CHECKSUM.
int dht_read(int pin, int* humidity, int* temperature);
double dewPointFast(double celsius, double humidity);
double humidex(double tempC, double DewPoint);
float ParseFloat(uint8_t intData, uint8_t fractionData);
#endif
