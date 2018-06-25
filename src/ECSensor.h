#ifndef __EC_ATMEGA__
#define __EC_ATMEGA__

#include <Arduino.h>


// Pin definitions for various AVR processors.
#ifdef __AVR_ATmega328P__
#define CAPPOS 2                              
#define CAPNEG 4                              
#define ECPIN 7         

#elif defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX41__)
#define CAPPOS PA1                            // Digital pin for CapPos.
#define CAPNEG PA0                            // Digital pin for CapNeg.
#define ECPIN PA2                             // Digital pin for EC.
#define INTERRUPT PCINT1                      // The pin change interrupt linked to PA1.

#elif defined(__AVR_ATtinyX5__)
#define CAPPOS PB1                            // digital pin for capPos.
#define INTERRUPT PCINT1                      // The pin change interrupt linked to PB1.
#define CAPNEG PB4                            // digital pin for capNeg.
#define ECPIN PB3                             // digital pin for EC.

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CAPPOS PE4                            // digital pin for capPos.
#define CAPNEG PA2                            // digital pin for capNeg.
#define ECPIN PA0                             // digital pin for EC.

#endif



// Time constants.                     
#define CHARGEDELAY 40                        // Time in microseconds it takes for the cap to charge; at least 5x RC.
                                              // 22 nF & 330R resistor RC = 7.25 us, times 5 = 36.3 us.
#define DISCHARGEDELAY 15                     // Discharge the cap before flipping polarity. Better for the pins. 2xRC.
                                              
#define EC_TIMEOUT 2000                       // Timeout for the EC measurement in microseconds.
                                              // 2 ms half cycle --> 250 Hz.

// Temperature compensation.
/**
 * As ion activity changes drastically with the temperature of the liquid, we have to correct for that. The 
 * temperature correction is a simple "linear correction", typical value for this ALPHA factor is 2%/degC.
 * 
 * Source and more information:
 * https://www.analyticexpert.com/2011/03/temperature-compensation-algorithms-for-conductivity/
 */
#define ALPHA 0.02

// Sensor status.
#define EC_FAIL 0
#define EC_SUCCESS 1

class ECSensor {

  public:
    ECSensor(void);
#if defined(__AVR__)
    void begin(void);
#elif defined(ESP8266)
    void begin(uint8_t, uint8_t, uint8_t);
#endif
    
    uint32_t readSensor(void);
    uint32_t readSensor(float temperature);
    void setOversampling(uint8_t);
    uint8_t getStatus(void);

  private:
    uint8_t oversamplingRate;
    uint8_t sensorStatus;
    uint8_t clockspeed;
#ifdef ESP8266
    uint8_t cpPin;
    uint8_t cnPin;
    uint8_t ecPin;
    static void capDischarged(void);
#endif

};

#endif

