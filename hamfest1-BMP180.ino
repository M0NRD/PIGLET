
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <string.h>

//include library code
#include <TinyGPS.h>
//#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 bmp180;

TinyGPS gps;

#define ASCII 7            // 7 bit ascii
#define STOPBITS 2         // 2 stop bits
#define TXDELAY 1          // Delay between sentence TX's
#define RTTY_BAUD 50       // Baud rate

#define RADIO_TXD  6      //NTX-2 TXD (PWM)
#define RADIO_EN   4      //NTX-2 ENABLE

#define BATTERY  0        //Analogue pin 0

volatile int txstatus = 1;
volatile int txstringlength = 0;
volatile char txc;
volatile int txi;
volatile int txj;

volatile unsigned long count; //message counter
unsigned char flags;
// GPS data
float flat = 0;
float flon = 0;	//latitude, longitude
float new_flat, new_flon;
long new_alt;
long alt = 0; //altitude
unsigned long fix_age, date, time; //fix age
unsigned short sats;
unsigned long stats_chars;
unsigned short stats_sentences, stats_failed;

char datastring[100]; //where the telementry string is stored
char txstring[100]; //copy of telementry string for transmission

char timestring[10] = "00:00:00"; //to process real time into string with colons
char latstr[12] = "53.098225";	  //Lat converted to string
char lonstr[12] = "-0.770781";	  //Lng converted to string
char altstr[8] = "0";
char satstr[3] = "0";
char flagstr[3] = "00";

float batVolt = 0; //battery voltage as float
float busVolt = 0; //bus voltage as float
char voltageBat[6] = "0.00"; //battery voltage as string
char temp[8] = "-00.00";
char pzPressure[10] = "0.00";

int value = 0;
float vout = 0;

float denominator;

double temperature = 0.0;
double pressure = 0.0;
float atm = 0.0;
float altitude = 0.0;
//SoftwareSerial db(10, 9);
byte gps_set_sucess = 0 ;

#define FLAG_NO_SERIAL	0x01
#define FLAG_NO_DECODE	0x02
#define FLAG_NO_FIX		0x04
#define FLAG_NO_GPSTIME	0x08

#define SET true
#define RESET false

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  //db.begin(9600);

  pinMode(RADIO_EN, OUTPUT);
  pinMode(RADIO_TXD, OUTPUT);
  setPwmFrequency(RADIO_TXD, 1);

  digitalWrite(RADIO_EN, LOW);    //Radio off.
  delay(1000);
  digitalWrite(RADIO_EN, HIGH);   //Radio on..

  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT
  //db.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  while (!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
  }
  gps_set_sucess = 0;

  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  // UNCOMMENT AS NEEDED

  //db.println("Switching off NMEA GLL: ");
  uint8_t setGLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B
  };
  while (!gps_set_sucess)
  {
    sendUBX(setGLL, sizeof(setGLL) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGLL);
  }
  gps_set_sucess = 0;
  //db.println("Switching off NMEA GSA: ");
  uint8_t setGSA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32
  };
  while (!gps_set_sucess)
  {
    sendUBX(setGSA, sizeof(setGSA) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGSA);
  }
  gps_set_sucess = 0;
  //db.println("Switching off NMEA GSV: ");
  uint8_t setGSV[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39
  };
  while (!gps_set_sucess)
  {
    sendUBX(setGSV, sizeof(setGSV) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGSV);
  }
  gps_set_sucess = 0;
  //db.println("Switching off NMEA RMC: ");
  uint8_t setRMC[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40
  };
  while (!gps_set_sucess)
  {
    sendUBX(setRMC, sizeof(setRMC) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setRMC);
  }

  count = 0;
  sats = 0;
  ChangeStatusFlag((FLAG_NO_SERIAL | FLAG_NO_DECODE | FLAG_NO_FIX | FLAG_NO_GPSTIME), RESET);
  flags = 0;

  bmp180.begin();
  initialise_interrupt();
}

//	loop
//
//	The main loop
void loop()
{
  static unsigned long serial_timeout = millis() + 5000;
  static unsigned long sentence_timeout = millis() + 5000;
  if ( Serial.available() ) {
    char c = Serial.read();
    //Serial.write(c);
    ChangeStatusFlag(FLAG_NO_SERIAL, RESET);

    serial_timeout = millis() + 2000;
    if (gps.encode(c)) { // Has a new valid sentence come in?
      ChangeStatusFlag(FLAG_NO_DECODE, RESET);
      sentence_timeout = millis() + 5000;
      ProcessGPS();
    }
    else if ( millis() > sentence_timeout )	{
      ChangeStatusFlag(FLAG_NO_DECODE, SET);
      sentence_timeout = millis() + 5000;
    }
  }
  else if ( millis() > serial_timeout )
  {
    ChangeStatusFlag(FLAG_NO_SERIAL, SET);
    serial_timeout = millis() + 5000;
  }
  ProcessSensors();
}

//	ProcessGPS
//
//	Called when we have received a new sentence
void ProcessGPS()
{
  gps.get_datetime(&date, &time, &fix_age);	//get date/time
  gps.f_get_position(&new_flat, &new_flon, &fix_age);	//latitude/longitude in degrees (+/-)
  new_alt = gps.altitude();					//altitude in centimeters (+/-)
  new_alt = new_alt / 100;
  sats = gps.satellites();				//number of satellites
  gps.stats(&stats_chars, &stats_sentences, &stats_failed); //update statistics
  //Check for certain conditions
  if ( sats == TinyGPS::GPS_INVALID_SATELLITES || fix_age == TinyGPS::GPS_INVALID_AGE ) {
    sats = 0;
    ChangeStatusFlag(FLAG_NO_FIX, SET);
  }
  else
  {
    ChangeStatusFlag(FLAG_NO_FIX, RESET);
    flat = new_flat;
    flon = new_flon;
    alt = new_alt;
  }

  if ( date == TinyGPS::GPS_INVALID_DATE ) {
    date = 0;
  }

  if ( time == TinyGPS::GPS_INVALID_TIME ) {
    time = 0;
  }

  if ( time == 0 || date == 0) {
    ChangeStatusFlag(FLAG_NO_GPSTIME, SET);
  }
  else {
    ChangeStatusFlag(FLAG_NO_GPSTIME, RESET);
  }

  dtostrf(flat, 8, 6, (char*)latstr); // convert lat from float to string
  dtostrf(flon, 8, 6, (char*)lonstr); // convert lon from float to string
  if ( time != 0 )
  {
    time = time / 100; //drop cc (millis) from time to leave us with HHMMSS
    sprintf((char*)timestring, "%06lu", time); //convert to characters
    sprintf((char*)timestring, "%c%c:%c%c:%c%c", timestring[0], timestring[1], timestring[2], timestring[3], timestring[4], timestring[5]); //Insert Colons
  }

  sprintf((char*)altstr, "%ld", alt);
  sprintf((char*)satstr, "%d", sats);
}

void ChangeStatusFlag(unsigned short flagbits, bool set)
{
  if ( set ) {
    flags = (flags | flagbits);
  }
  else {
    flags = (flags & ~flagbits);
  }
  sprintf((char*)flagstr, "%02X", flags);
}

//	ProcessSensors
//
//	Read sensors
void ProcessSensors()
{
  ReadTempPressure();

  static unsigned long timer = millis() + 1000;

  if ( millis() > timer )
  {
    //ReadTempPressure();
    //busVolt = ((float)readVcc() / (float)1000);
    //dtostrf(busVolt, 3, 2, (char*)voltageBus); // convert from float to string
    batVolt = analogRead(BATTERY);
    batVolt = batVolt * (5.0 / 1023.0);
    //batVolt += 0.6;
    dtostrf(batVolt, 3, 2, (char*)voltageBat);
//dtostrf(temperature, 2, 1, (char*)temp); // convert from float to string
    timer = millis() + 1000; //reset timer
  }
}

//Timer that fires to send each RTTY bit. From Upu
ISR(TIMER1_COMPA_vect)
{
  switch (txstatus) {
    case 0: // This is the optional delay between transmissions.
      txj++;
      if (txj > (TXDELAY * RTTY_BAUD)) {
        txj = 0;
        txstatus = 1;
      }
      break;

    case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
      //Example sentence  $$$$NERDTEST,1000,144854,51.452808,00.176337,100,5,00,8.95,5.01*9652
      //Callsign
      //Sentence Id (%i)
      //Time (%s)
      //Latitude (%s)
      //Longitude (%s)
      //Altitude (%i)
      //Satellites (%i)
      //Flags (%i)
      //Battery Voltage (%s)
      //Bus Voltage (%s)
      //Temperature (%s)
      sprintf((char*)datastring, "$$$$PIGLET,%lu,%s,%s,%s,%s,%s,%s,%s,%s", count, timestring, latstr, lonstr, altstr, satstr, voltageBat, temp, pzPressure); //put together all var into one string //now runs at end of loop()
      crccat((char*)datastring + 4); //add checksum (lunars code)

      count++;
      strcpy(txstring, (char*)datastring);
      txstringlength = strlen(txstring);
      txstatus = 2;
      txj = 0;
      break;

    case 2: // Grab a char and lets go transmit it.
      if ( txj < txstringlength)
      {
        txc = txstring[txj];
        txj++;
        txstatus = 3;
        rtty_txbit (0); // Start Bit;
        txi = 0;
      }
      else
      {
        txstatus = 0; // Should be finished
        txj = 0;
      }
      break;

    case 3:
      if (txi < ASCII)
      {
        txi++;
        if (txc & 1) rtty_txbit(1);
        else rtty_txbit(0);
        txc = txc >> 1;
        break;

      }
      else
      {
        rtty_txbit (1); // Stop Bit
        txstatus = 4;
        txi = 0;
        break;

      }
    case 4:
      if (STOPBITS == 2)
      {
        rtty_txbit (1); // Stop Bit
        txstatus = 2;
        break;

      }
      else
      {
        txstatus = 2;
        break;
      }

  }
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 64:
        mode = 0x03;
        break;
      case 256:
        mode = 0x04;
        break;
      case 1024:
        mode = 0x05;
        break;
      default:
        return;
    }

    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 32:
        mode = 0x03;
        break;
      case 64:
        mode = 0x04;
        break;
      case 128:
        mode = 0x05;
        break;
      case 256:
        mode = 0x06;
        break;
      case 1024:
        mode = 0x7;
        break;
      default:
        return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

//Switch the radios frequency Low/High depending on the current bit.
void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    //digitalWrite(RADIO_TXD, HIGH);
    analogWrite(RADIO_TXD, 110);
  }
  else
  {
    // low
    //digitalWrite(RADIO_TXD, LOW);
    analogWrite(RADIO_TXD, 100);

  }
}

//Start the timer interrupt to send RTTY bits.
void initialise_interrupt()
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

// CRC16 Checksum from Lunar_Lander
uint16_t crccat(char *msg)
{
  uint16_t x;
  for (x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);
  snprintf(msg, 8, "*%04X\n", x);
  return (x);
}

// Secret internal rail voltage meter
// http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
//
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void ReadTempPressure()
{
  static char phase = 0;
  static unsigned long timer = millis();
  char status;

  if  (millis() > timer )
  {
    switch (phase)
    {
      case 0:
        status = bmp180.startTemperature();
        if (status != 0)
        {
          timer = millis() + status;
          phase = 1;
        }
        else
        {
          timer = millis() + 1000;
        }
        break;

      case 1:
        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getTemperature(temperature);
        if (status != 0)
        {
          // Start a pressure measurement:
          // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
          // If request is successful, the number of ms to wait is returned.
          // If request is unsuccessful, 0 is returned.

          status = bmp180.startPressure(0);
          if (status != 0)
          {
            timer = millis() + status;
            phase = 2;
          }
          else
          {
            phase = 0;
            timer = millis() + 1000;
          }
          break;

        case 2:
          // Retrieve the completed pressure measurement:
          // Note that the measurement is stored in the variable P.
          // Note also that the function requires the previous temperature measurement (T).
          // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
          // Function returns 1 if successful, 0 if failure.

          status = bmp180.getPressure(pressure, temperature);
          if (status != 0)
          {
            dtostrf(temperature, 1, 0, temp); // convert from float to string
            dtostrf(pressure, 1, 0, pzPressure); // convert from float to string
          }
          phase = 0;
          timer = millis() + 1000;
          break;
        }
    }
  }
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial.write(MSG[i]);
    //Serial.print(MSG[i], HEX);
  }
  Serial.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      //Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      //Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        //Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
