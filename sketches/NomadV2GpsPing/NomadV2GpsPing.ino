#include <SoftwareSerial.h>
#include <string.h>

#define MAX_SENTENCE_LENGTH 255
#define SENTENCE_END '\n'

uint8_t receivePin = 3,
        transmitPin = 4,
        sentenceIndex = 0;
SoftwareSerial gpsDevice(receivePin, transmitPin);
char sentence[MAX_SENTENCE_LENGTH + 1];
char byteRead;
char emptyBuffer = ((char) -1);
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
    Serial.print("C>");
    Serial.println(checksumString);

    return checksumString;
}

void
setup() {
    Serial.begin(115200);
    gpsDevice.begin(9600);
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
    gpsDevice.println("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");

    /*
     * - set the update frequency - once per second
     */
    gpsDevice.println("$PMTK220,1000*1F");

    /*
     * - enable WAAS
     */
    gpsDevice.println("$PMTK301,2*2E");
    
    sentence[sentenceIndex] = eostr;
    return;
}

void
readGpsSentence() {
    for (;;) {
        byteRead = gpsDevice.read();
        if (byteRead == emptyBuffer) {
            break;
        };
    
        switch(byteRead) {
    
        case SENTENCE_END :
            sentence[sentenceIndex] = byteRead;
            sentenceIndex++;
            sentence[sentenceIndex] = eostr;
            Serial.println(sentence);
            sentenceIndex = 0;
            return;
    
        default   : 
            if (byteRead < 32) {
                break;
            }
            if (byteRead > 126) {
                break;
            }
    
            sentence[sentenceIndex] = byteRead;
            sentenceIndex++;
            break;
        }
    
    };
};

void
loop() {
  readGpsSentence();
}
