/**
 * capacitor based TDS measurement
 * pin CapPos ----- 330 ohm resistor ----+------------+
 *                                       |            |
 *                                10-47 nF cap     EC probe or
 *                                       |         resistor (for simulation)
 * pin CapNeg ---------------------------+            |
 *                                                    |
 * pin ECpin ------ 330 ohm resistor -----------------+
 * 
 * So, what's going on here?
 * EC - electic conductivity - is the reciprocal of the resistance of the liquid.
 * So we have to measure the resistance, but this can not be done directly as running
 * a DC current through an ionic liquid just doesn't work, as we get electrolysis and
 * the migration of ions to the respective electrodes.
 * 
 * So this routing is using the pins of the microprocessor and a capacitor to produce a
 * high frequency AC current (at least 1 kHz, best 3 kHz - based on the pulse length, but the
 * pulses come at intervals). Alternating the direction of the current in these 
 * short pulses prevents the problems mentioned above. Maximum frequency should be about
 * 500 kHz (i.e. pulse length about 1 microsecond).
 *
 * To get the needed timing resolution, especially for higher EC values, we measure
 * clock pulses rather than using micros().
 * 
 * Then to get the resistance it is not possible to measure the voltage over the
 * EC probe (the normal way of measuring electrical resistance) as this drops with
 * the capacitor discharging. Instead we measure the time it takes for the cap to
 * discharge enough for the voltage on the pin to drop so much, that the input
 * flips from High to Low state. This time taken is a direct measure of the
 * resistance encountered (the cap and the EC probe form an RC circuit) in the
 * system, and that's what we need to know.
 * 
 * Now the working of this technique.
 * Stage 1: charge the cap full through pin CapPos.
 * Stage 2: let the cap drain through the EC probe, measure the time it takes from
 * flipping the pins until CapPos drops LOW.
 * Stage 3: charge the cap full with opposite charge.
 * Stage 4: let the cap drain through the EC probe, for the same period of time as
 * was measured in Stage 2 (as compensation).
 * Cap is a small capacitor, in this system we use 10-47 nF but with other probes a
 * larger or smaller value can be required (the original research this is based
 * upon used a 3.3 nF cap). Resistor R1 is there to protect pin CapPos and 
 * CapNeg from being overloaded when the cap is charged up, resistor R2
 * protects ECpin from too high currents caused by very high EC or shorting the
 * probe.
 * 
 * Pins set to input are assumed to have infinite impedance, leaking is not taken into
 * account. The specs of NodeMCU give some 66 MOhm for impedance, several orders of
 * magnitude above the typical 1-100 kOhm resistance encountered by the EC probe.
 * 
 * Original research this is based upon:
 * https://hal.inria.fr/file/index/docid/635652/filename/TDS_Logger_RJP2011.pdf
 * 
 */

#include "ECSensor.h"

volatile uint32_t dischargeCycles;

ECSensor::ECSensor() {
  oversamplingRate = 6;
  #ifdef __AVR__
  clockspeed = F_CPU / 1000000;
  #else
  clockspeed = 1;
  #endif
}

uint32_t ECSensor::readSensor(float temperature) {
  uint32_t reading = readSensor();
  reading = (float)reading / (1 + ALPHA * (temperature - 25));
  return reading;
}

void ECSensor::setOversampling(uint8_t rate) {
  oversamplingRate = rate;
}

uint8_t ECSensor::getStatus() {
  return sensorStatus;
}

// The actual reading of the sensor is highly platform dependent, as is the initialisation of timers and so.
// So the begin() and readSensor() functions are defined for the various platforms here.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Arduino Uno / Nano / Micro / Mini (ATmega328P processor)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef __AVR_ATmega328P__
void ECSensor::begin() {
  TCCR1A = 0;                                 // Clear timer/counter control register 1A
  TCCR1B = 0;                                 // Clear timer control register
  TCCR1B |= (1 << CS10);                      // Set timer to no prescaler
  EIMSK &= ~(1 << INT0);                      // External Interrupt Mask Register - EIMSK - is for enabling INT[6;3:0] interrupts,
  //                                          // INT0 is disabled to avoid false interrupts when manipulating EICRA
  EICRA |= (1 << ISC01);                      // External Interrupt Control Register A - EICRA - defines the interrupt edge profile,
  //                                          // here configured to trigger on falling edge
}

uint32_t ECSensor::readSensor() {

  uint32_t totalCycles = 0;
  bool timeout = false;
  EIMSK |= (1 << INT0);                       // Enable INT0.
  for (uint16_t i = 0; i < (1 << oversamplingRate); i++) {

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    DDRD |= (1 << CAPPOS);                    // CAPPOS output.
    DDRD |= (1 << CAPNEG);                    // CAPNEG output.
    DDRD &= ~(1 << ECPIN);                    // ECPIN input.
    PORTD |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    PORTD &= ~(1 << CAPNEG);                  // CAPNEG LOW.
    PORTD &= ~(1 << ECPIN);                   // ECPIN pull up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    // CAPPOS: input.
    // CAPNEG: output, low (unchanged).
    // EC: output, low.
    dischargeCycles = 0;
    DDRD &= ~(1 << CAPPOS);                   // CAPPOS input.
    DDRD |= (1 << ECPIN);                     // ECPIN output.
    PORTD &= ~(1 << CAPPOS);                  // CAPPOS pull up resistor off.
    PORTD &= ~(1 << ECPIN);                   // ECPIN LOW.
    TCNT1 = 0;                                // Reset the timer.
    timeout = true;
    while (TCNT1 < (EC_TIMEOUT * clockspeed)) {
      if (dischargeCycles) {
        timeout = false;
        break;
      }
    }
    totalCycles += dischargeCycles;
    if (timeout) {
      break;
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    DDRD |= (1 << CAPPOS);                    // CAPPOS output.
    DDRD &= ~(1 << ECPIN);                    // ECPIN input.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    PORTD |= (1 << CAPNEG);                   // CAPNEG HIGH: Charge the cap
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    DDRD &= ~(1 << CAPPOS);                   // CAPPOS input
    DDRD |= (1 << ECPIN);                     // ECPIN output
    PORTD |= (1 << ECPIN);                    // ECPIN HIGH

    // Delay based on dischargeCycles, making it equal in duration to the positive discharge cycle.
    TCNT1 = 0;
    while (TCNT1 < dischargeCycles) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    DDRD |= (1 << CAPPOS);                    // CAPPOS output.
    PORTD |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    DDRD &= ~(1 << ECPIN);                    // ECPIN input.
    PORTD &= ~(1 << ECPIN);                   // ECPIN pull-up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}
  }

  uint32_t averageCycles;
  if (timeout) {
    averageCycles = 0;
  }
  else {
    averageCycles = (totalCycles >> oversamplingRate);
  }
  EIMSK &= ~(1 << INT0);                      // Disable INT0.

  // Disconnect the sensor.
  DDRD &= ~(1 << CAPPOS);                     // CAPPOS input.
  DDRD &= ~(1 << CAPNEG);                     // CAPNEG input.
  DDRD &= ~(1 << ECPIN);                      // ECPIN input.
  return averageCycles;
}

// ISR routine
ISR(INT0_vect) {                              
  dischargeCycles = TCNT1;                    // Record the value of the timer - this is the time it took for the discharge in clock cycles.
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ESP8266
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif defined(ESP8266)
static volatile uint32_t endCycle;            // Used in the interrupt handler: stores the cycle number when the sampling is complete.

void ECSensor::begin(uint8_t cp, uint8_t cn, uint8_t ec) {
  cpPin = cp;
  cnPin = cn;
  ecPin = ec;
}

uint32_t ECSensor::readSensor() {

  uint32_t dischargeCycles = 0;               // The number of clock cycles it took for the capacitor to discharge.
  uint32_t totalCycles = 0;                   // The cumulative number of clock cycles over all measurements.
  uint32_t startCycle;                        // The clock cycle count at which the measurement starts.
  uint32_t startTime;                         // The micros() count at which the measurement starts (for timeout).
  bool timeout = false;
  for (uint16_t i=0; i < (1 << oversamplingRate); i++) { // take 2^ECSAMPLES measurements of the EC.

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    digitalWrite(cpPin, HIGH);
    digitalWrite(cnPin, LOW);
    pinMode (ecPin, INPUT);
    pinMode (cpPin, OUTPUT);
    pinMode (cnPin, OUTPUT);
    delayMicroseconds(CHARGEDELAY);           // allow the cap to charge fully.

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    // CAPPOS: input.
    // CAPNEG: output, low (unchanged).
    // EC: output, low.
    endCycle = 0;
    startTime = micros();
    pinMode (cpPin,INPUT);
    startCycle = ESP.getCycleCount();
    attachInterrupt(digitalPinToInterrupt(cpPin), reinterpret_cast<void (*)()>(&capDischarged), FALLING);
    digitalWrite(ecPin, LOW);
    pinMode(ecPin, OUTPUT);
    while (endCycle == 0) {                   // endCyle gets set in the ISR, when an interrupt is received.
      if (micros() - startTime > EC_TIMEOUT) { // Time out - in case sensor not connected or not in water.
        timeout = true;
        break;
      }
    }
    detachInterrupt(digitalPinToInterrupt(cpPin));
    if (timeout) break;
    dischargeCycles = endCycle - startCycle;
    totalCycles += dischargeCycles;
    
    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    digitalWrite(cpPin, LOW);
    digitalWrite(cnPin, LOW);
    pinMode (ecPin, INPUT);
    pinMode (cpPin, OUTPUT);
    pinMode (cnPin, OUTPUT);
    delayMicroseconds(DISCHARGEDELAY);

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    digitalWrite (cnPin, HIGH);
    delayMicroseconds (CHARGEDELAY);

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    digitalWrite (ecPin, HIGH);
    pinMode (cpPin,INPUT);
    pinMode (ecPin, OUTPUT); 
    delayMicroseconds (dischargeCycles / 80);

    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    digitalWrite(cpPin, HIGH);
    pinMode(cpPin, OUTPUT);
    pinMode(ecPin, INPUT);
    delayMicroseconds(DISCHARGEDELAY);
  }
  if (timeout) {
    dischargeCycles = 0;
  }
  else {
    dischargeCycles = (totalCycles >> oversamplingRate);
  }   
  yield();                                    // For the ESP8266: allow for background processes to run.
  
  // Disconnect the sensor.
  pinMode(cpPin, INPUT);
  pinMode(cnPin, INPUT);
  pinMode(ecPin, INPUT);
  return dischargeCycles;
}

/*
 * The ISR which registers when the cap has discharged to the point the pin flips.
 */
void ICACHE_RAM_ATTR ECSensor::capDischarged() {
  endCycle = ESP.getCycleCount();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ATtiny25/45/85
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#elif defined(__AVR_ATtinyX5__)
void ECSensor::begin() {

  // Set up the timers.
  // These two timers are combined to effectively have a single 16-bit timer.
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0B |= (1 << CS02);                      // prescaler = 256.

  TCCR1 = 0;                                  //  clear timer control register
  TCCR1 |= (1 << CS10);                       //  Set to no prescaler

  GIMSK |= (1 << PCIE);                       // Enable pin change interrupts.
  SREG |= (1 << 7);
}

uint32_t ECSensor::readSensor() {
  uint32_t totalCycles = 0;
  bool timeout = false;
  for (uint8_t i = 0; i < (1 << oversamplingRate); i++) {

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    DDRB |= (1 << CAPPOS);                    // CAPPOS output.
    DDRB |= (1 << CAPNEG);                    // CAPNEG output.
    DDRB &= ~(1 << ECPIN);                    // ECPIN input.
    PORTB |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    PORTB &= ~(1 << CAPNEG);                  // CAPNEG LOW.
    PORTB &= ~(1 << ECPIN);                   // ECPIN pull up resistor off.
    TCNT0 = 0;
    while (TCNT0 < ((CHARGEDELAY * clockspeed) >> 8)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    // CAPPOS: input.
    // CAPNEG: output, low (unchanged).
    // EC: output, low.
    dischargeCycles = 0;
    TCNT1 = 0;                                // Reset the timers.
    TCNT0 = 0;
    DDRB &= ~(1 << CAPPOS);                   // CAPPOS input.
    PORTB &= ~(1 << CAPPOS);                  // CAPPOS pull up resistor off.
    DDRB |= (1 << ECPIN);                     // ECPIN output.
    PCMSK |= (1 << INTERRUPT);                // Set up the pin change interrupt on CAPPOS.
    timeout = true;
    while (TCNT0 < ((EC_TIMEOUT * clockspeed) >> 8)) {
      if (dischargeCycles) {
        timeout = false;
        break;
      }
    }
    totalCycles += dischargeCycles;
    PCMSK &= ~(1 << INTERRUPT);               // Clear the pin change interrupt on CAPPOS.
    if (timeout) {                            // A timeout here means the sensor is not measuring anything.
      break;
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    DDRB |= (1 << CAPPOS);                    // CAPPOS output.
    DDRB &= ~(1 << ECPIN);                    // ECPIN input.
    TCNT0 = 0;
    while (TCNT0 < (DISCHARGEDELAY * clockspeed) >> 8) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    PORTB |= (1 << CAPNEG);                   // CAPNEG HIGH: Charge the cap
    TCNT0 = 0;
    while (TCNT0 < (CHARGEDELAY * clockspeed) >> 8) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    DDRB &= ~(1 << CAPPOS);                   // CAPPOS input
    DDRB |= (1 << ECPIN);                     // ECPIN output
    PORTB |= (1 << ECPIN);                    // ECPIN HIGH
    TCNT0 = 0;
    while (TCNT0 < (dischargeCycles >> 8)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    DDRB |= (1 << CAPPOS);                    // CAPPOS output.
    PORTB |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    DDRB &= ~(1 << ECPIN);                    // ECPIN input.
    PORTB &= ~(1 << ECPIN);                   // ECPIN pull-up resistor off.
    TCNT0 = 0;
    while (TCNT0 < (DISCHARGEDELAY * clockspeed) >> 8) {}
  }
  uint16_t averageCycles = (totalCycles >> oversamplingRate);

  // Disconnect the sensor.
  DDRB &= ~(1 << CAPPOS);                     // CAPPOS input.
  DDRB &= ~(1 << CAPNEG);                     // CAPPOS input.
  DDRB &= ~(1 << ECPIN);                      // ECPIN input.
  return averageCycles;
}

ISR(PCINT0_vect) {
  dischargeCycles = TCNT0 << 8 + TCNT1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ATtiny24/44/84/441/841/441a/841a
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#elif defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX41__)
void ECSensor::begin() {

  // Set up the timer.
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10);                      // Set to no prescaler to count clock ticks.
  GIMSK |= (1 << PCIE0);                      // Enable pin change interrupts on the PB pins.
}

uint32_t ECSensor::readSensor() {
  uint32_t totalCycles = 0;
  bool timeout = false;                       // Record whether we finished due to timeout (which would result in an invalid reading).
  for (uint16_t i = 0; i < (1 << oversamplingRate); i++) {

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    DDRA |= (1 << CAPPOS);                    // CAPPOS output.
    PORTA |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    DDRA |= (1 << CAPNEG);                    // CAPNEG output.
    PORTA &= ~(1 << CAPNEG);                  // CAPNEG LOW.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    PORTA &= ~(1 << ECPIN);                   // ECPIN pull up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    // CAPPOS: input.
    // CAPNEG: output, low (unchanged).
    // EC: output, low.
    dischargeCycles = 0;
    TCNT1 = 0;                                // Reset the timer.
    DDRA &= ~(1 << CAPPOS);                   // CAPPOS input.
    PORTA &= ~(1 << CAPPOS);                  // CAPPOS pull up resistor off.
    DDRA |= (1 << ECPIN);                     // ECPIN output.
    PCMSK0 |= (1 << INTERRUPT);               // Set up the pin change interrupt on CAPPOS.
    timeout = true;
    while (TCNT1 < (EC_TIMEOUT * clockspeed)) {
      if (dischargeCycles) {
        timeout = false;
        break;
      }
    }
    totalCycles += dischargeCycles;
    PCMSK0 &= ~(1 << INTERRUPT);              // Clear the pin change interrupt on CAPPOS.
    if (timeout) {                            // A timeout here means the sensor is not measuring anything.
      break;
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    DDRA |= (1 << CAPPOS);                    // CAPPOS output.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    PORTA |= (1 << CAPNEG);                   // CAPNEG HIGH: Charge the cap
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    DDRA &= ~(1 << CAPPOS);                   // CAPPOS input
    DDRA |= (1 << ECPIN);                     // ECPIN output
    PORTA |= (1 << ECPIN);                    // ECPIN HIGH

    // Delay based on dischargeCycles making it equal in duration to the positive discharge cycle.
    TCNT1 = 0;
    while (TCNT1 < dischargeCycles) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    DDRA |= (1 << CAPPOS);                    // CAPPOS output.
    PORTA |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    PORTA &= ~(1 << ECPIN);                   // ECPIN pull-up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}
  }

  uint32_t dischargeCycles;
  sensorStatus = EC_SUCCESS;
  if (timeout) {
    sensorStatus = EC_FAIL;                   // A timeout has been recorded - probably no probe present.
    dischargeCycles = 0;
  }
  else {
    dischargeCycles = (totalCycles >> oversamplingRate);
  }

  // Disconnect the sensor.
  DDRA &= ~(1 << CAPPOS);                     // CAPPOS input.
  DDRA &= ~(1 << CAPNEG);                     // CAPNEG input.
  DDRA &= ~(1 << ECPIN);                      // ECPIN input.
  return dischargeCycles;
}

ISR(PCINT0_vect) {
  dischargeCycles = TCNT1;
  PCMSK0 &= ~(1 << INTERRUPT);                // Clear the pin change interrupt on CAPPOS.
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Arduino Mega or Arduino Mega2580 (ATmega1280 or ATmega2560)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#elif defined(__AVR_ATmega640__) || (__AVR_ATmega1280__) || (__AVR_ATmega1281__) || (__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
void ECSensor::begin() {
  TCCR1A = 0;                                 // Clear timer/counter control register 1A
  TCCR1B = 0;                                 // Clear timer control register
  TCCR1B |= (1 << CS10);                      // Set timer to no prescaler
  EIMSK &= ~(1 << INT4);                      // External Interrupt Mask Register - EIMSK - is for enabling INT[6;3:0] interrupts,
  //                                             INT0 is disabled to avoid false interrupts when manipulating EICRA
  EICRB |= (1 << ISC41);                      // External Interrupt Control Registers - EICRA & EICRB - define the interrupt edge profile,
  //                                             here INT4 is configured to trigger on falling edge.
}

uint32_t ECSensor::readSensor() {

  uint32_t totalCycles = 0;
  bool timeout = false;
  EIMSK |= (1 << INT4);                       // Enable INT4

  for (uint16_t i = 0; i < (1 << oversamplingRate); i++) {

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    DDRE |= (1 << CAPPOS);                    // CAPPOS output.
    DDRA |= (1 << CAPNEG);                    // CAPNEG output.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    PORTE |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    PORTA &= ~(1 << CAPNEG);                  // CAPNEG LOW.
    PORTA &= ~(1 << ECPIN);                   // ECPIN pull up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    dischargeCycles = 0;
    DDRE &= ~(1 << CAPPOS);                   // CAPPOS input.
    DDRA |= (1 << ECPIN);                     // ECPIN output.
    PORTE &= ~(1 << CAPPOS);                  // CAPPOS pull up resistor off.
    PORTA &= ~(1 << ECPIN);                   // ECPIN LOW.
    TCNT1 = 0;                                // Reset the timer.
    timeout = true;
    while (TCNT1 < (EC_TIMEOUT * clockspeed)) {
      if (dischargeCycles) {
        timeout = false;
        break;
      }
    }
    totalCycles += dischargeCycles;
    if (timeout) {
      break;
    }

    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    DDRE |= (1 << CAPPOS);                    // CAPPOS output.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    PORTA |= (1 << CAPNEG);                   // CAPNEG HIGH: Charge the cap
    TCNT1 = 0;
    while (TCNT1 < (CHARGEDELAY * clockspeed)) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    DDRE &= ~(1 << CAPPOS);                   // CAPPOS input
    DDRA |= (1 << ECPIN);                     // ECPIN output
    PORTA |= (1 << ECPIN);                    // ECPIN HIGH

    // Delay based on dischargeCycles, making it equal in duration to the positive discharge cycle.
    TCNT1 = 0;
    while (TCNT1 < dischargeCycles) {}

    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    DDRE |= (1 << CAPPOS);                    // CAPPOS output.
    PORTA |= (1 << CAPPOS);                   // CAPPOS HIGH: Charge the cap.
    DDRA &= ~(1 << ECPIN);                    // ECPIN input.
    PORTA &= ~(1 << ECPIN);                   // ECPIN pull-up resistor off.
    TCNT1 = 0;
    while (TCNT1 < (DISCHARGEDELAY * clockspeed)) {}
  }

  EIMSK &= ~(1 << INT4);                      // Disable INT4
  uint32_t averageCycles;
  if (timeout) {
    averageCycles = 0;
  }
  else {
    averageCycles = (totalCycles >> oversamplingRate);
  }
  return averageCycles;
}

// ISR routine
ISR(INT4_vect) {                              // INTO_vect indicates that ISR() handles an INT0 external interrupt
  dischargeCycles = TCNT1;                    // Record the value of the timer - this is the time it took for the discharge in clock cycles.
}

#else
#error Unsupported platform.
#endif


