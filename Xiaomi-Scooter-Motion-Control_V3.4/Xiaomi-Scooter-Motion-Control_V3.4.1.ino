#include <Arduino.h>
#include <arduino-timer.h>
#include <SPI.h>

// WARNING: Always try to use the latest version available.
//          Check here: https://github.com/kearfy/Xiaomi-Scooter-Motion-Control

// +-============================================================================-+
// |-================================ SETTINGS ==================================-|
// +-============================================================================-+

//Defines how much km/u is detected as a kick.
const int speedBump = 3;

//Defines what the minimal increasement every kick should make.
const int minimumSpeedIncreasment = 7;

//From which speed should the minimumSpeedIncreasment be activated.
const int enforceMinimumSpeedIncreasmentFrom = 15;

//Defines how much percent the speedBump should go down per km in speed. (it's harder to speed up when driving 20km/u.)
const float lowerSpeedBump = 0.9875;

//Minimum and maximum speed of your scooter. (You can currently use this for just one mode)
const int minimumSpeed = 7;
const int maximumSpeed = 27;

//Minimum speed before throtteling
const int startThrottle = 5;

//Amount of kicks it takes to switch over the INCREASING state.
const int kicksBeforeIncreasment = 1;

//Don't touch unless you know what you are doing.
const int brakeTriggered = 47;

//Defines how long one kick should take.
const int drivingTime = 5000;

//Defines how much time the amount of kicks before increasment can take up.
const int kickResetTime = 2000;

//Defines how much time should be in between two kicks.
const int kickDelay = 300;

//Defines the amount of time the INCREASING state will wait for a new kick.
const int increasmentTime = 2000;

//used to calculate the average speed.
const int historySize = 20;

// Arduino pin where throttle is connected to (only pin 9 & 10 is ok to use)
const int THROTTLE_PIN = 10;
int LED_PCB = 13;


//END OF SETTINGS

// +-============================================================================-+
// |-=============================== VARIABLES ==================================-|
// +-============================================================================-+
// |-========================== !!! DO NOT EDIT !!! =============================-|
// +-============================================================================-+

unsigned long currentTime = 0;
unsigned long drivingTimer = 0;
unsigned long kickResetTimer = 0;
unsigned long kickDelayTimer = 0;
unsigned long increasmentTimer = 0;
bool kickAllowed = true;

bool isBraking = true;                  // Brake activated
int Speed; //current speed
int temporarySpeed = 0;
int expectedSpeed = 0;
int kickCount = 0;

int historyTotal = 0;
int history[historySize];
int historyIndex = 0;
int averageSpeed = 0;

//motionmodes
uint8_t State = 0;
uint8_t prevState = 0;
#define READYSTATE 0
#define INCREASINGSTATE 1
#define DRIVINGSTATE 2
#define BREAKINGSTATE 3
#define DRIVEOUTSTATE 4


// Protocol decoding
#define DASH2MOTOR 0x20
#define MOTOR2DASH 0x21
#define DASH2BATT  0x22
#define BATT2DASH  0x23
#define MOTOR2BATT 0x24
#define BATT2MOTOR 0x25
#define BRAKEVALUE 0x65
#define SPEEDVALUE 0x64
#define SERIALVALUE 0x31

auto kickTimer = timer_create_default();
auto kickDelayTimer1 = timer_create_default();
auto increasingTimer = timer_create_default();
auto drivingTimer1 = timer_create_default();

void logByteInHex(uint8_t val) {
    //  if(val < 16)
    //    Serial.print('0');
    //
    //  Serial.print(val, 16);
    //  Serial.print(' ');
}

uint8_t readBlocking() {
    while (!Serial1.available())
    delay(1);
    return Serial1.read();
}

void setup() {
    Serial.begin(115200); Serial1.begin(115200); // Start reading SERIAL_PIN at 115200 baud
    pinMode(9, OUTPUT);

    // initialize all the readings to 0:
    for (int i = 0; i < historySize; i++) {
        history[i] = 0;
    }

    digitalWrite(SS, HIGH); // disable Slave Select
    SPI.begin();


    ThrottleWrite(45);
}

uint8_t buff[256];
void loop() {
    int w = 0;
    while (readBlocking() != 0x55);

    if (readBlocking() != 0xAA) {
        return;
    }

    uint8_t len = readBlocking();
    buff[0] = len;

    if (len > 254) {
        return;
    }

    uint8_t addr = readBlocking();
    buff[1] = addr;

    uint16_t sum = len + addr;
    for (int i = 0; i < len; i++) {
        uint8_t curr = readBlocking();
        buff[i + 2] = curr;
        sum += curr;
    }

    uint16_t checksum = (uint16_t) readBlocking() | ((uint16_t) readBlocking() << 8);
    if (checksum != (sum ^ 0xFFFF)) {
        return;
    }

    for (int i = 0; i < len + 2; i++) {
        logByteInHex(buff[i]);
    }


    switch (buff[1]) {
        case DASH2MOTOR: // Destination: Dash to motor
            switch (buff[2]) {
                case BRAKEVALUE:
                    isBraking = (buff[6]>brakeTriggered);
            }
        case MOTOR2DASH:
            switch (buff[2]) {
                case SPEEDVALUE:
                    if (buff[8] != 0) {
                        Speed = buff[8];
                    }
            }
    }

    //Update the current index in the history and recalculate the average speed.

    historyTotal = historyTotal - history[historyIndex];
    history[historyIndex] = Speed;
    historyTotal = historyTotal + history[historyIndex];
    historyIndex = historyIndex++;

    if (historyIndex >= historySize) {
        historyIndex = 0;
    }

    //Recalculate the average speed.
    averageSpeed = historyTotal / historySize;

    motion_control();
    currentTime = millis();
    if (kickResetTimer != 0 && kickResetTimer + kickResetTime < currentTime && State == DRIVINGSTATE) resetKicks();
    // if (increasmentTimer != 0 && increasmentTimer + increasmentTime < currentTime && State == INCREASINGSTATE) endIncrease();
    // if (drivingTimer != 0 && drivingTimer + drivingTime < currentTime && State == DRIVINGSTATE) endDrive();
    if (kickDelayTimer == 0 || (kickDelayTimer != 0 && kickDelayTimer + kickDelay < currentTime)) {
        kickAllowed = true;
        kickDelayTimer = 0;
    } else {
        kickAllowed = false;
    }
    kickTimer.tick();
    kickDelayTimer.tick();
    increasingTmer.tick();
    drivingTimer.tick();
}

void motion_control() {

    // Speed is higher than 0, speed is higher than startThrottle and the user is not braking
    if ((Speed != 0) && (Speed < startThrottle) || isBraking) {
        // If speed is under 5 km/h, stop throttle
        ThrottleWrite(45); //  0% throttle
        kickTimer.cancel();
        increasingTimer.cancel();
        if (State != BREAKINGSTATE) {
            State = BREAKINGSTATE;
            drivingTimer = 0; kickResetTimer = 0; increasmentTimer = 0;
            Serial.println("BREAKING ~> Handle pulled.");
        }
    }

    switch(State) {
        case READYSTATE:
            if (Speed > startThrottle) {
                if (Speed > minimumSpeed) {
                    if (averageSpeed > Speed) {
                        temporarySpeed = averageSpeed;
                    } else {
                        temporarySpeed = Speed;
                    }
                } else {
                    temporarySpeed = minimumSpeed;
                }
                
                ThrottleSpeed(temporarySpeed);
                switchState(INCREASINGSTATE);
            }

            break;
        case INCREASINGSTATE:
            if (Speed >= temporarySpeed + calculateSpeedBump(temporarySpeed) && kickAllowed) {
                kickCount++;
                increasmentTimer = 0;
                kickDelayTimer = currentTime;
            } else if (averageSpeed >= Speed && kickCount < 1 && increasmentTimer == 0) {
                increasmentTimer = currentTime;
            }

            if (kickCount >= 1) {
                if (Speed > temporarySpeed + calculateMinimumSpeedIncreasment(Speed)) {
                    temporarySpeed = ValidateSpeed(Speed);
                } else {
                    temporarySpeed = ValidateSpeed(temporarySpeed + calculateMinimumSpeedIncreasment(Speed));
                }

                ThrottleSpeed(temporarySpeed);
            }

            kickCount = 0;
            break;
        case DRIVINGSTATE:
            if (Speed >= expectedSpeed + calculateSpeedBump(expectedSpeed) && kickAllowed) {
                kickCount++;
                drivingTimer = currentTime + increasmentTime;
                kickResetTimer = currentTime;
                kickDelayTimer = currentTime;
            }

            if (kickCount >= kicksBeforeIncreasment) {
                if (Speed > expectedSpeed + calculateMinimumSpeedIncreasment(Speed)) {
                    temporarySpeed = ValidateSpeed(Speed);
                } else {
                    temporarySpeed = ValidateSpeed(expectedSpeed + calculateMinimumSpeedIncreasment(Speed));
                }

                ThrottleSpeed(temporarySpeed);
                State = INCREASINGSTATE;
                drivingTimer = 0; kickResetTimer = 0;
                Serial.println("INCREASING ~> The least amount of kicks before increasment has been reached.");
            }

            break;
        case BREAKINGSTATE:
        case DRIVEOUTSTATE:
            if (isBraking) break;
            if (Speed < startThrottle) {
                switchState(READYSTATE);
                Serial.println("READY ~> Speed has dropped under the minimum throttle speed.");
            } else if (Speed >= averageSpeed + calculateSpeedBump(Speed)) {
                if (State == DRIVEOUTSTATE) {
                    if (Speed > averageSpeed + calculateMinimumSpeedIncreasment(Speed)) {
                        temporarySpeed = ValidateSpeed(Speed);
                    } else {
                        temporarySpeed = ValidateSpeed(expectedSpeed);
                    }
                } else {
                    temporarySpeed = ValidateSpeed(Speed);
                }

                kickDelayTimer = currentTime;
                ThrottleSpeed(temporarySpeed);
                State = INCREASINGSTATE;
                Serial.println("INCREASING ~> Break released and speed increased.");
            }
            
            break;
        default:
            ThrottleWrite(45);
            digitalWrite(LED_PCB, HIGH);
            switchState(BREAKINGSTATE);
            drivingTimer = 0; kickResetTimer = 0; increasmentTimer = 0;
            Serial.println("BREAKING ~> Unknown state detected");
    }

}


void switchState(int nextState){
    prevState = State;
    switch(nextState){
        case READYSTATE:
            State = nextState;
        case INCREASINGSTATE:
            State = nextState;
            increasingTimer.in(increasmentTime, switchState, DRIVINGSTATE);
        case DRIVINGSTATE:
            State = nextState;
            drivingTimer1.in(drivingTime, switchState, DRIVEOUTSTATE);
            if (prevState == INCREASINGSTATE){
                endIncrease(); // TODO: LEGACY FUNCTION, TO BE REMOVED
            }
        case DRIVEOUTSTATE:
            State = nextState;
            if (prevState == DRIVINGSTATE){
                endDrive(); // TODO: LEGACY FUNCTION, TO BE REMOVED
            }
    }




}
int resetKicks() {
    kickResetTimer = 0;
    kickCount = 0;
}

int endIncrease() {
    expectedSpeed = temporarySpeed;
    kickCount = 0;
    drivingTimer = currentTime;
    increasmentTimer = 0; kickResetTimer = 0;
    Serial.println("DRIVING ~> The speed has been stabilized.");
}

int endDrive() {
    drivingTimer = 0; kickResetTimer = 0;
    ThrottleWrite(45);
    Serial.println("READY ~> Speed has dropped under the minimum throttle speed.");
}

int calculateSpeedBump(int requestedSpeed) {
  return round(speedBump * pow(lowerSpeedBump, requestedSpeed));
}

int calculateMinimumSpeedIncreasment(int requestedSpeed) {
    return (requestedSpeed > enforceMinimumSpeedIncreasmentFrom ? minimumSpeedIncreasment : 0);
}

int ValidateSpeed(int requestedSpeed) {
    if (requestedSpeed < minimumSpeed) {
        return minimumSpeed;
    } else if (requestedSpeed > maximumSpeed) {
        return maximumSpeed;
    } else {
        return requestedSpeed;
    }
}

int ThrottleSpeed(int requestedSpeed) {
    if (requestedSpeed == 0) {
        ThrottleWrite(45);
    } else if (requestedSpeed == maximumSpeed) {
        ThrottleWrite(233);
    } else {
        int throttleRange = 233 - 45;
        float calculatedThrottle = ((requestedSpeed + 4) / (float) maximumSpeed * throttleRange);
        ThrottleWrite((int) calculatedThrottle);
    }
}

int ThrottleWrite(int value) {
    digitalWrite(9,LOW);
    SPI.transfer(0);
    if (value < 45) {
        SPI.transfer(233);
    } else if (value > 233) {
        SPI.transfer(233);
    } else {      
        SPI.transfer(value);
    }
    digitalWrite(9,HIGH);
    }