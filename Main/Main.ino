// Included
#include <Wire.h>
// http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <OneWire.h>
// https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DallasTemperature.h>
// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>
// https://github.com/adafruit/Adafruit-Motor-Shield-library
#include <AFMotor.h>
// https://github.com/sparkfun/SparkFun_APDS-9960_Sensor_Arduino_Library
#include <SparkFun_APDS9960.h>
// https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
#include <SparkFunCCS811.h>
// https://github.com/pvizeli/CmdParser
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#define Firmware "0.0.3"

static const uint8_t fillPin = 2;
// static const uint8_t pwmFanReadPin   = 2;
static const uint8_t pwmFanPin = 3; // digital PWM pin 9
//static const uint8_t x         = 4; // must be INPUT_PULLUP compatible // USED FOR MOTORSHIELD!
static const uint8_t dhtPin = 5;       // DHT11 or DHT22
static const uint8_t waterTempPin = 6; // DS18B20
// static const uint8_t x          = 7; // USED FOR MOTORSHIELD!
// static const uint8_t x          = 8; // USED FOR MOTORSHIELD!
static const uint8_t relay0 = 9;
static const uint8_t relay1 = 10;
//static const uint8_t motor0           = 11; // USED FOR MOTORSHIELD!
//static const uint8_t x                = 12; // USED FOR MOTORSHIELD!

static const uint8_t soilMoisture0 = A0;
static const uint8_t soilMoisture1 = A1;
// static const uint8_t ???  = A2;
// static const uint8_t ???  = A3;
// static const uint8_t I2C_SDA        = A4; // USED FOR I2C
// static const uint8_t I2C_SCL        = A5; //USED FOR I2C

#define DHTTYPE DHT22

DHT dht(dhtPin, DHTTYPE);

float humidity;
float temperature; // In °C

OneWire oneWire(waterTempPin);
DallasTemperature sensors(&oneWire);

AF_DCMotor motor(1);

SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;

#define CCS811_ADDR 0x5A //Default I2C Address
CCS811 myCCS811(CCS811_ADDR);

uint16_t tvoc;
uint16_t eco2;

CmdCallback<13> cmdCallback;
CmdBuffer<32> myBuffer;
CmdParser myParser;

// unsigned long time;
// unsigned int rpm;
// String stringRPM;

String setFandSpeedString;
uint8_t setFandSpeed;
String ledStatus;
String relayNo;
String relayStatus;
String motorDir;
String soilMoistureSensorNo;

// Setup functions:

inline void setupPwmFan()
{
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
    TCCR2B = (1 << CS21);                            // start timer (ck/8 prescalar)
    OCR2B = 0;                                       //Timer A OCR2A is used by the motor lib, so we use OCR2B
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 3 is now PE5 (OC3C)
    TCCR3A |= _BV(COM1C1) | _BV(WGM10); // fast PWM, turn on oc3c
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    OCR3C = 0;
#else
#error "This chip is currently not supported!"
#endif
    pinMode(3, OUTPUT);
}

inline void setupRelay()
{
    pinMode(relay0, OUTPUT);
    pinMode(relay1, OUTPUT);

    digitalWrite(relay0, HIGH);
    digitalWrite(relay1, HIGH);
}

// Callback functions:

void getFirmware(CmdParser *myParser)
{
    Serial.println(Firmware);
}

void setFanSpeed(CmdParser *myParser)
{
    setFandSpeedString = myParser->getCmdParam(1);
    setFandSpeed = setFandSpeedString.toInt();
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2B = setFandSpeed;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    OCR3C = setFandSpeed;
#elif defined(__PIC32MX__)
    // Set the OC1 (pin3) PMW duty cycle from 0 to 255
    OC1RS = setFandSpeed;
#else
#error "This chip is not supported!"
#endif
    Serial.println("OK");
}

void getFanSpeed(CmdParser *myParser)
{
    // time = pulseIn(pwmFanReadPin, HIGH);
    // rpm = (1000000 * 60) / (time * 4);
    // time = pulseIn(pwmFanReadPin, HIGH);     //  measure HIGH part of the pulse
    // time += pulseIn(pwmFanReadPin, LOW);    //  add in the LOW part of another pulse
    // rpm = (1000000 * 60) / (time * 2);    // calculate RPM based upon the fan generating two pulses/rev
    // stringRPM = String(rpm);
    // if (stringRPM.length() < 5) {
    //     Serial.println(rpm, DEC);
    //     return;
    // }
    Serial.println("ERROR");
}

void getFillStatus(CmdParser *myParser)
{
    if (digitalRead(fillPin) == HIGH)
    {
        Serial.println("1");
    }
    else
    {
        Serial.println("0");
    }
}

void getDhtStatus(CmdParser *myParser)
{
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("ERROR");
        return;
    }

    Serial.print(humidity);
    Serial.print(" ");
    Serial.println(temperature);
}

void getWaterTemp(CmdParser *myParser)
{
    sensors.requestTemperatures();
    Serial.println(sensors.getTempCByIndex(0));
}

void setRelay(CmdParser *myParser)
{
    relayNo = myParser->getCmdParam(1);
    relayStatus = myParser->getCmdParam(2);
    if (relayNo.toInt() == 1)
    {
        if (relayStatus.toInt() == 1)
        {
            digitalWrite(relay1, LOW);
            Serial.println("OK");
            return;
        }
        digitalWrite(relay1, HIGH);
        Serial.println("OK");
        return;
    }

    if (relayStatus.toInt() == 1)
    {
        digitalWrite(relay0, LOW);
        Serial.println("OK");
        return;
    }
    digitalWrite(relay0, HIGH);
    Serial.println("OK");
    return;
}

void setMotor(CmdParser *myParser)
{
    motorDir = myParser->getCmdParam(1);
    if (motorDir.toInt() == 1)
    {
        motor.run(FORWARD);
        Serial.println("OK");
        return;
    }

    if (motorDir.toInt() == -1)
    {
        motor.run(BACKWARD);
        Serial.println("OK");
        return;
    }

    if (motorDir.toInt() == 0)
    {
        motor.run(RELEASE);
        Serial.println("OK");
        return;
    }

    Serial.println("ERROR");
}

void getSoilMoisture(CmdParser *myParser)
{
    soilMoistureSensorNo = myParser->getCmdParam(1);
    if (soilMoistureSensorNo.toInt() == 0)
    {
        Serial.println(analogRead(soilMoisture0));
        return;
    }

    if (soilMoistureSensorNo.toInt() == 1)
    {
        Serial.println(analogRead(soilMoisture1));
        return;
    }

    Serial.println("ERROR");
}

void getAmbientLight()
{
    if (apds.readAmbientLight(ambient_light))
    {
        Serial.println(ambient_light);
        return;
    }

    Serial.println("ERROR");
}

void getCSS811()
{
    if (myCCS811.dataAvailable())
    {
        humidity = dht.readHumidity();
        temperature = dht.readTemperature();

        //This sends the temperature data to the CCS811
        myCCS811.setEnvironmentalData(humidity, temperature);

        //Calling this function updates the global tVOC and eCO2 variables
        myCCS811.readAlgorithmResults();

        eco2 = myCCS811.getCO2();
        tvoc = myCCS811.getTVOC();

        Serial.print(eco2);
        Serial.print(" ");
        Serial.println(tvoc);
    }
    else if (myCCS811.checkForStatusError())
    {
        // uint8_t error = myCCS811.getErrorRegister();

        // if (error == 0xFF) //comm error
        // {
        //     Serial.println("Failed to get ERROR_ID register.");
        // }
        // else
        // {
        //     Serial.print("Error: ");
        //     if (error & 1 << 5)
        //         Serial.print("HeaterSupply");
        //     if (error & 1 << 4)
        //         Serial.print("HeaterFault");
        //     if (error & 1 << 3)
        //         Serial.print("MaxResistance");
        //     if (error & 1 << 2)
        //         Serial.print("MeasModeInvalid");
        //     if (error & 1 << 1)
        //         Serial.print("ReadRegInvalid");
        //     if (error & 1 << 0)
        //         Serial.print("MsgInvalid");
        //     Serial.println();
        // }

        Serial.println("ERROR");
    }
}

void setLed(CmdParser *myParser)
{
    ledStatus = myParser->getCmdParam(1);
    if (ledStatus.toInt() == 1)
    {
        digitalWrite(LED_BUILTIN, 1);
        Serial.println("OK");
        return;
    }
    digitalWrite(LED_BUILTIN, 0);
    Serial.println("OK");
}

void scanI2C()
{
    uint8_t nDevices = 0;
    for (byte address = 1; address < 127; ++address)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            if (nDevices != 0)
            {
                Serial.print(" ");
            }
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);

            ++nDevices;
        }
        else if (error == 4)
        {
            //   Serial.print("Unknown error at address 0x");
            //   if (address < 16) {
            //     Serial.print("0");
            //   }
            //   Serial.println(address, HEX);
        }
    }

    Serial.println("");
}

void setup()
{
    Serial.begin(9600);

    motor.setSpeed(255);
    motor.run(RELEASE);

    setupPwmFan();
    setupRelay();
    pinMode(fillPin, INPUT_PULLUP);
    dht.begin();
    sensors.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    if (!apds.init())
    {
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    }
    if (!apds.enableLightSensor(false))
    {
        Serial.println(F("Something went wrong during light sensor init!"));
    }

    Wire.begin();
    CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
    if(returnCode != CCS811Core::CCS811_Stat_SUCCESS)
    {
        Serial.println(myCCS811.statusString(returnCode));
    }

    cmdCallback.addCmd("setfan", &setFanSpeed);
    cmdCallback.addCmd("getfan", &getFanSpeed);
    cmdCallback.addCmd("getfill", &getFillStatus);
    cmdCallback.addCmd("getdht", &getDhtStatus);
    cmdCallback.addCmd("getwatertmp", &getWaterTemp);
    cmdCallback.addCmd("setrelay", &setRelay);
    cmdCallback.addCmd("setmotor", &setMotor);
    cmdCallback.addCmd("getsoilmoisture", &getSoilMoisture);
    cmdCallback.addCmd("getambientlight", &getAmbientLight);
    cmdCallback.addCmd("getcss811", &getCSS811);
    cmdCallback.addCmd("led", &setLed);
    cmdCallback.addCmd("scani2c", &scanI2C);
    cmdCallback.addCmd("getfw", &getFirmware);
}

void loop()
{
    // Check for new char on serial and call function if command was entered
    //cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial);#
    if (myBuffer.readSerialChar(&Serial))
    {
        // parse command line
        // return ERROR when parsing failed
        if (myParser.parseCmd(&myBuffer) != CMDPARSER_ERROR)
        {
            // search command in store and call function
            // return ERROR if command was not found
            if (cmdCallback.processCmd(&myParser) == false)
            {
                Serial.println("ERROR");
            }
            myBuffer.clear();
        }
        else
        {
            Serial.println("ERROR");
        }
    }
}