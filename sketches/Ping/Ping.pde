/* Ultrasound Sensor
 *------------------
 *
 * Reads values (00014-01199) from an ultrasound sensor (3m sensor)
 * and writes the values to the serialport.
 *
 * http://www.xlab.se | http://www.0j0.org
 * copyleft 2005 Mackie for XLAB | DojoDave for DojoCorp
 *
 */

int pingOne = 10;
int pingTwo = 11;
int pingThree = 12;
int pingFour = 13;
int timeoutValue = 512;

void ping(int pingUnit, int pingPin) {
    int pinValue = 0;
    int timecount = 0;
    int timeoutCounter = 0;
    unsigned long startTime,
                  oneWayTime;

	pinMode(pingPin, OUTPUT);
	
	/* Send low-high-low pulse to activate the trigger pulse of the sensor
	 * -------------------------------------------------------------------
	 */
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pingPin, LOW);
	
	/* Listening for echo pulse
	 * -------------------------------------------------------------------
	 */
	
	pinMode(pingPin, INPUT);
	pinValue = digitalRead(pingPin);
	while (pinValue == LOW && timeoutCounter < timeoutValue) {
        pinValue = digitalRead(pingPin);
        startTime = micros();
        timeoutCounter += 1;
	}

    if (timeoutCounter >= timeoutValue) {
        return;
    }
	
	while(pinValue == HIGH) {
        pinValue = digitalRead(pingPin);
	}
    oneWayTime = (micros() - startTime) / 2;
	
    Serial.print(pingUnit);
    Serial.print(',');
	Serial.println(oneWayTime * 0.0003432);

    return;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    ping(1, pingOne);
    delay(10);

    ping(2, pingTwo);
    delay(10);

    ping(3, pingThree);
    delay(10);

    ping(4, pingFour);
    delay(10);

    delay(1000);
} 
