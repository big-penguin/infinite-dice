#include <Servo.h>

Servo j1;
Servo j2;
Servo j3;
Servo j4;

uint8_t msg[4];
uint8_t ctr = 0;

void setup() {
    // put your setup code here, to run once:
    j1.attach(9);
    j2.attach(10);
    j3.attach(11);
    j4.attach(6);

    Serial.begin(9600);
}

void loop() {
    if (Serial.available()) {
        msg[ctr] = Serial.read();
        Serial.println(msg[ctr]);
        ctr++;
        
        if (ctr > 3) {
            ctr = 0;
            Serial.read();
            setmotors();
        }
    }
}

void setmotors() {
    j1.write(msg[0]);
    j2.write(msg[1]);
    j3.write(msg[2]);
    j4.write(msg[3]);
}
