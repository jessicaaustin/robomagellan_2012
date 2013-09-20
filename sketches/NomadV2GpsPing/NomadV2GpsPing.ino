#include <SoftwareSerial.h>
#include <string.h>

#define MAX_SENTENCE_LENGTH 255
#define SENTENCE_END '\n'

uint8_t receivePin = 3,
        transmitPin = 4,
        sentenceIndex = 0;
SoftwareSerial gpsDevice(receivePin, transmitPin);
char sentence[MAX_SENTENCE_LENGTH + 1];
char inputBuffer[256];
int bytesRead;
char eostr = ((char) 0);

char*
calcChecksum(
    char* message,
    int messageLength
    ) {

    static unsigned char checksum;
    static char checksumString[2];

    checksum = (unsigned char) 0;
    for (int index = 0; index < messageLength; index++) {
        checksum ^= message[index];
    };

    sprintf(checksumString, "%02X", checksum);

    return checksumString;
}

void
findDataRate() {
	long dataRates[] = { 57600, 115200, 9600, 4800, 14400, 19200, 38400 };

    gpsDevice.setTimeout((unsigned long) 1000);

    for (int index = 0; index < 7; index++) {
        Serial.print("Trying ");
        Serial.println(dataRates[index]);

        gpsDevice.begin(dataRates[index]);
        gpsDevice.flush();
        gpsDevice.println("$PMTK000*32");
        bytesRead = gpsDevice.readBytes(inputBuffer, 255);
        inputBuffer[bytesRead] = 0;
        if (strstr(inputBuffer, "$PMTK001") != NULL) {
            Serial.print("Found data rate ");
            Serial.println(dataRates[index]);
            break;
        };
        gpsDevice.end();
        delay(100);
    };

    return;
};

void
sendSentence(char *message) {
    gpsDevice.print("$");
    gpsDevice.print(message);
    gpsDevice.print("*");
    gpsDevice.println(calcChecksum(message, strlen(message)));

    return;
};

void
setup() {
    Serial.begin(115200);
    Serial.println("GPS sketch 1");
    
    findDataRate();
    delay(1000);

    /*
     * - select the NMEA sentences
     */
    Serial.println("GPMRC, GPGGA sentences");
    sendSentence("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

    /*
     * - set the update frequency - once per second
     */
    Serial.println("Set update frequency to 1 Hz");
    sendSentence("PMTK220,1000");

    /*
     * - enable WAAS
     */
    Serial.println("Enable WAAS");
    sendSentence("PMTK301,2");
    
    sentence[sentenceIndex] = eostr;
    return;
}

void
readGpsSentence() {
    bytesRead = gpsDevice.readBytes(inputBuffer, 255);
    if (bytesRead == 0) {
        Serial.println("No bytes read");
    } else {
        inputBuffer[bytesRead] = (char) 0;
        Serial.print(inputBuffer);
    };
        
};

void
loop() {
  readGpsSentence();
}
