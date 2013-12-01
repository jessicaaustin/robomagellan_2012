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
int bytesRead = 0,
    bufferIndex = 0;
short sentenceIncomplete = 1;
char eostr = ((char) 0);

char*
calcCheckSum(
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
};

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
    gpsDevice.println(calcCheckSum(message, strlen(message)));

    return;
};

void
setup() {
    Serial.begin(115200);
    Serial.println("GPS sketch 4");
    
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
    int sentenceIncomplete = 1,
        needCheckSum = 0,
        foundStart = 0;
    char checkSum[2],
         *calculatedCheckSum;
    int checkSumIndex = 0;

    Serial.print("bytesRead ");
    Serial.print(bytesRead);
    Serial.print(" bufferIndex ");
    Serial.println(bufferIndex);
    while (sentenceIncomplete) {
        if (bufferIndex == 0) {
            Serial.println("readBytes()");
	        bytesRead = gpsDevice.readBytes(inputBuffer, 255);
        };

	    if (bytesRead == 0) {
            /*
             * No part of a sentence was available.
             */
            Serial.println("Nothing read");
	        return;
	    } else {
            while (bufferIndex < bytesRead) {
                if (needCheckSum) {
                    checkSum[checkSumIndex] = inputBuffer[bufferIndex];
                    checkSumIndex++;

                    if (checkSumIndex == 2) {
                        /*
                         * A complete sentence with checksum was received.
                         * Calculate the checksum of the sentence and
                         * compare it with the checksum included with the
                         * sentence. If they match, a valid sentence was
                         * received.
                         */
                        calculatedCheckSum = calcCheckSum(
                            sentence,
                            sentenceIndex
                            );
                        if (calculatedCheckSum[0] == checkSum[0] &&
                            calculatedCheckSum[1] == checkSum[1]) {
                            sentenceIncomplete = 0;
                            Serial.println("Checksum valid");
                            break;
                        } else {
                            Serial.println("Checksum comparison failed");
                            return;
                        };
                    };
                } else if (inputBuffer[bufferIndex] == '$') {
                    /*
                     * The $ starts every sentence.
                     */
                    foundStart = 1;
                    needCheckSum = 0;
                    sentenceIndex = 0;
                    Serial.println("Start");
                } else if (foundStart) {
                    if (inputBuffer[bufferIndex] == '*') {
                        /*
                         * The * ends every sentence.
                         */
                        needCheckSum = 1;
                        checkSumIndex = 0;
                        foundStart = 0;
                        Serial.println("End");
                    } else {
                        sentence[sentenceIndex] = inputBuffer[bufferIndex];
                        sentenceIndex++;
                    };
                };

                bufferIndex++;
            };
            bufferIndex = 0;
        };
    };

	sentence[sentenceIndex] = (char) 0;
	Serial.println(sentence);
        
};

void
loop() {
  readGpsSentence();
}
