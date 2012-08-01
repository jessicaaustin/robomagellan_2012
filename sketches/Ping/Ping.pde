/* Ultrasound Sensor
 *------------------
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

int interPingDelay = 150; /* microseconds */
int loopDelay = 0;     /* microseconds */

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
	Serial.println(oneWayTime * 0.0003461); /* meters per microsecond */

    return;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    ping(1, pingOne);
    delay(interPingDelay);

    ping(2, pingTwo);
    delay(interPingDelay);

    ping(3, pingThree);
    delay(interPingDelay);

    ping(4, pingFour);
    delay(interPingDelay);

    delay(loopDelay);
} 
