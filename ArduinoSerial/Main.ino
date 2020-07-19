#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

// Pwm set and rpm calculation from: https://forum.arduino.cc/index.php?topic=155089.msg1175321#msg1175321

#define Firmware "0.0.2"

// uint8_t pwmFanPin       = 3; // digital PWM pin 9
// uint8_t pwmFanReadPin   = 2;
uint8_t fillPin         = 4; // must be INPUT_PULLUP compatible
uint8_t dhtPin          = 5; // DHT11 or DHT22
uint8_t waterTempPin    = 6; // DS18B20
uint8_t relay0          = 2;
uint8_t relay1          = 3;

#define DHTTYPE DHT22

DHT dht(dhtPin, DHTTYPE);

float humidity;
float temperature;

OneWire oneWire(waterTempPin);
DallasTemperature sensors(&oneWire);

CmdCallback<8> cmdCallback;
CmdBuffer<32> myBuffer;
CmdParser myParser;

unsigned long time;
unsigned int rpm;
String stringRPM;
int pwmVal = 1; // The PWM Value
String setFandSpeedString;
int setFandSpeed;

String ledStatus;
String relayNo;
String relayStatus;

// Setup functions:

void setupPwmFan() {
    // generate 25kHz PWM pulse rate on Pin 3
    // pinMode(pwmFanPin, OUTPUT);   // OCR2B sets duty cycle
    // // Set up Fast PWM on Pin 3
    // TCCR2A = 0x23; // _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);?      // COM2B1, WGM21, WGM20 
    // // Set prescaler  
    // TCCR2B = 0x0A; // _BV(WGM21);?   // WGM21, Prescaler = /8
    // // Set TOP and initialize duty cycle to zero(0)
    // OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
    // OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0 :
    // digitalWrite(pwmFanReadPin, HIGH);   // Starts reading
}

void setupRelay() {
    pinMode(relay0, OUTPUT);
    pinMode(relay1, OUTPUT);
}

// Callback functions:

void getFirmware(CmdParser *myParser) {
    Serial.println(Firmware);
}

void setFanSpeed(CmdParser *myParser) {
    // setFandSpeedString = myParser->getCmdParam(1);
    // setFandSpeed = setFandSpeedString.toInt();
    // if (setFandSpeed > 0 && setFandSpeed < 80) {
    //     OCR2B = setFandSpeed;
    //     Serial.println("OK");
    //     return;
    // }

    Serial.println("ERROR");
}

void getFanSpeed(CmdParser *myParser) {
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

void getFillStatus(CmdParser *myParser) {
    if(digitalRead(fillPin) == HIGH) {
        Serial.println("1");
    }
    else {
        Serial.println("0");
    }
}

void getDhtStatus(CmdParser *myParser) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {       
        Serial.println("ERROR");
        return;
    }

    Serial.print(humidity);
    Serial.print(" ");
    Serial.println(temperature);
}

void getWaterTemp(CmdParser *myParser) {
    sensors.requestTemperatures();
    Serial.println(sensors.getTempCByIndex(0));
}

void setRelay(CmdParser *myParser) {
    relayNo = myParser->getCmdParam(1);
    relayStatus = myParser->getCmdParam(2);
    if(relayNo.toInt() == 1) {
        if(relayStatus.toInt() == 1) {
            digitalWrite(relay1, 1);
            Serial.println("OK");
            return;
        }
        digitalWrite(relay1, 0);
        Serial.println("OK");
        return;
    }

    if(relayStatus.toInt() == 1) {
        digitalWrite(relay0, 1);
        Serial.println("OK");
        return;
    }
    digitalWrite(relay0, 0);
    Serial.println("OK");
    return;
}

void setLed(CmdParser *myParser) {
    ledStatus = myParser->getCmdParam(1);
    if(ledStatus.toInt() == 1) { 
        digitalWrite(LED_BUILTIN, 1);
        Serial.println("OK");
        return;
    }
    digitalWrite(LED_BUILTIN, 0);
    Serial.println("OK");
}

void setup()
{
    setupPwmFan();
    pinMode(fillPin, INPUT_PULLUP);
    dht.begin();
    sensors.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(9600);

    cmdCallback.addCmd("setfan", &setFanSpeed);
    cmdCallback.addCmd("getfan", &getFanSpeed);
    cmdCallback.addCmd("getfill", &getFillStatus);
    cmdCallback.addCmd("getdht", &getDhtStatus);
    cmdCallback.addCmd("getwatertmp", &getWaterTemp);
    cmdCallback.addCmd("setrelay", &setRelay);
    cmdCallback.addCmd("led", &setLed);
    cmdCallback.addCmd("getfw", &getFirmware);
    
}

void loop()
{
   // Check for new char on serial and call function if command was entered
    //cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial);#
    if (myBuffer.readSerialChar(&Serial)) {
        // parse command line
        // return ERROR when parsing failed
        if (myParser.parseCmd(&myBuffer) != CMDPARSER_ERROR) {
            // search command in store and call function
            // return ERROR if command was not found
            if(cmdCallback.processCmd(&myParser) == false) {
                Serial.println("ERROR");
            }
            myBuffer.clear();
        }
        else {
            Serial.println("ERROR");
        }
    }
}