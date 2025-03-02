/*
 Name:		GPS_LoRaWAN.ino
 Created:	3/6/2020 4:37:36 PM
 Author:	Mark Mortimer
*/

#include <Adafruit_GPS.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LoRa-otaa.h"
#define GPSSERIAL Serial1
#define VBATPIN A7


Adafruit_GPS GPS(&GPSSERIAL);

#define GPSECHO false


uint32_t timer = millis();
int timeBetweenReads = 10000;

uint8_t mydata[19];

bool dataSent = false;

union {
    float a;
    unsigned char bytes[4];
} coord;

union {
    int a;
    unsigned char bytes[2];
} Speed ;

union {
    float a;
    unsigned char bytes[4];
}Voltage;


void loraInit(){
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);
    Serial.println("lora setup");
}

void setup() {
    Serial.begin(115200);

    while (!Serial);

    Serial.println("starting ...");

    loraInit();

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
}

void loop() {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
            return;
    }

    if (millis() - timer > timeBetweenReads) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        Serial.print("Battery: "); Serial.println(batteryVoltage());

        if (GPS.fix) {

            Serial.print("Location: ");
            Serial.print(GPS.latitude, 6); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

            coord.a = GPS.latitude * 1000;
            if (GPS.lat == 'S') { coord.a = coord.a * -1; }
            mydata[0] = coord.bytes[0];
            mydata[1] = coord.bytes[1];
            mydata[2] = coord.bytes[2];
            mydata[3] = coord.bytes[3];

            Serial.print("Lat ");
            Serial.println(coord.a);

            //Serial.print(" B0 ");
            //Serial.print(coord.bytes[0]);
            //Serial.print(" B1 ");
            //Serial.print(coord.bytes[1]);
            //Serial.print(" B2 ");
            //Serial.print(coord.bytes[2]);
            //Serial.print(" B3 ");
            //Serial.println(coord.bytes[3]);

            coord.a = GPS.longitude * 1000;
            if (GPS.lon == 'W') { coord.a = coord.a * -1; }

            mydata[4] = coord.bytes[0];
            mydata[5] = coord.bytes[1];
            mydata[6] = coord.bytes[2];
            mydata[7] = coord.bytes[3];

            Serial.print("Lon ");
            Serial.println(coord.a);

            coord.a = GPS.altitude;

            mydata[8] = coord.bytes[0];
            mydata[9] = coord.bytes[1];
            mydata[10] = coord.bytes[2];
            mydata[11] = coord.bytes[3];

            Serial.print("Alt ");
            Serial.println(coord.a);

            Speed.a = GPS.speed;

            mydata[12] = Speed.bytes[0];
            mydata[13] = Speed.bytes[1];

            mydata[14] = GPS.satellites;

            Voltage.a = batteryVoltage();
            mydata[15] = Voltage.bytes[0];
            mydata[16] = Voltage.bytes[1];
            mydata[17] = Voltage.bytes[2];
            mydata[18] = Voltage.bytes[3];

            Serial.print("Bat Volt ");
            Serial.println(Voltage.a);

            sendData();

        }
        timer = millis();
    }
    
}

boolean sendData() {
    dataSent = false;
    Serial.println("sending ...");

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);

    int Cnt = 0;
    while (!dataSent) {
        os_runloop_once();
        delay(1);

        if (Cnt > 1000) {
            Serial.print("Freq ");
            Serial.print(LMIC.freq);
            Serial.print(", Mills ");
            Serial.println(millis());
            Cnt = 0;
        }
        Cnt = Cnt + 1;
    }

    Serial.println("PayLoad Sent");
}

float batteryVoltage() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2; //double 100K resistor on Bat pin, So multiply back.
    measuredvbat *= 3.3; //3.3 is referance voltage.
    measuredvbat /= 1024; //Convert to voltage.
    return measuredvbat;
}


void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println("EV_SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        Serial.println("EV_BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        Serial.println("EV_BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        Serial.println("EV_BEACON_TRACKED");
        break;
    case EV_JOINING:
        Serial.println("EV_JOINING");
        break;
    case EV_JOINED:
        Serial.println("EV_JOINED");
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println("EV_RFU1");
        break;
    case EV_JOIN_FAILED:
        Serial.println("EV_JOIN_FAILED");
        break;
    case EV_REJOIN_FAILED:
        Serial.println("EV_REJOIN_FAILED");
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ack");
        if (LMIC.dataLen) {
            Serial.println("Received ");
            Serial.println(LMIC.dataLen);
            Serial.println(" bytes of payload");
            for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                    Serial.print("0");
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
        }
        // Schedule next transmission
        //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        dataSent = true;
        break;
    case EV_LOST_TSYNC:
        Serial.println("EV_LOST_TSYNC");
        break;
    case EV_RESET:
        Serial.println("EV_RESET");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println("EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        Serial.println("EV_LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        Serial.println("EV_LINK_ALIVE");
        break;
    default:
        Serial.println("Unknown event");
        break;
    }
}
