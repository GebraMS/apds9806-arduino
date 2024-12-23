#include "GebraBit_APDS9306.h"

GebraBit_APDS9306 APDS9306;

void setup() {
    Wire.begin();           // Initialize the I2C bus
    Serial.begin(9600);     // Initialize serial communication for debugging

    GB_APDS9306_initialize(&APDS9306); // Initialize the APDS9306 sensor
    GB_APDS9306_Configuration(&APDS9306); // Configure the APDS9306 sensor
}

void loop() {
    GB_APDS9306_Get_Data(&APDS9306); // Read data from the sensor
    
    Serial.print("luminosity: ");
    Serial.print(APDS9306.LUMINOSITY);
    Serial.println(" lx");
    
    delay(2000); // Delay between readings
}
