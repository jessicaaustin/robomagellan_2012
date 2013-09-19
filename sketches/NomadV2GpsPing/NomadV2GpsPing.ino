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
    while (messageLength > 0) {
        checksum ^= *message;
        messageLength--;
    };

    sprintf(checksumString, "%02X", checksum);
    /*
    Serial.print("C>");
    Serial.println(checksumString);
     */

    return checksumString;
}

void
setup() {
    Serial.begin(115200);
    Serial.println("GPS sketch 1");
    Serial.println("Setting data rate to 9600");
    gpsDevice.begin(115200);
    gpsDevice.setTimeout((unsigned long) 1000);
    
    /*
     * - set the baud rate
     */
    gpsDevice.println("$PMTK251,9600*17");
    gpsDevice.end();
    gpsDevice.begin(9600);
    gpsDevice.flush();

    /*
     * - select the NMEA sentences - GPGGA only
     */
    Serial.println("Only the GPGGA sentence");
    gpsDevice.println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");

    /*
     * - set the update frequency - once per second
     */
    Serial.println("Set update frequency to 1 Hz");
    gpsDevice.println("$PMTK220,1000*1F");

    /*
     * - enable WAAS
     */
    Serial.println("Enable WAAS");
    gpsDevice.println("$PMTK301,2*2E");
    
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
