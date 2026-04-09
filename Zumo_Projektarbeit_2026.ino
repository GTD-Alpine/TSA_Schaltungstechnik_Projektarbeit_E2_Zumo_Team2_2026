// -------------------------------------------------------
// ZUMO SUMO-ROBOTER – Pololu Zumo Shield V1.3 + Elegoo UNO R3
// -------------------------------------------------------
//
// HARDWARE:
//   - Pololu Zumo Shield V1.3 mit Elegoo UNO R3 (Arduino-kompatibel)
//   - 2x Mikro-Getriebemotoren über ZumoMotors-Bibliothek
//   - 2x IR-Bodensensoren (ZumoReflectanceSensorArray, Pin 4 + 5)
//   - 3x HC-SR04 Ultraschallsensoren (vorne / hinten / Turm)
//   - 1x L3G Gyro-Sensor (I2C) für gradgenaue Drehungen
//   - 1x LSM6DS33 Accelerometer (I2C, via ZumoIMU) für Kollisionserkennung
//   - 1x SG90 Servo (Pin 6) für Turmdrehung – Timer2-ISR
//   - 1x Zumo-Taster zum Starten/Stoppen
//
// TIMER-BELEGUNG (ATmega328P):
//   Timer0 → millis(), micros() – Arduino-Core (NICHT ANFASSEN)
//   Timer1 → ZumoMotors (M1PWM/M2PWM) – BELEGT
//   Timer2 → Servo-ISR (NICHT ZumoBuzzer – Buzzer wird nicht verwendet)
//   Servo.h → VERBOTEN (würde Timer1 klauen → Konflikt mit ZumoMotors)
//
// ZUSTÄNDE:
//   SEARCH      – Fährt vorwärts, sucht den Gegner
//   REVERSE     – Rückwärts nach Randerkennung
//   TURN_LEFT   – Gyro-gesteuerte Linksdrehung (90°)
//   TURN_RIGHT  – Gyro-gesteuerte Rechtsdrehung (90°)
//   SPIN        – Gyro-gesteuerte 180°-Kehrtwende
//   CHARGE      – Vollgas vorwärts (Angriff, zeitgesteuert)
//   SWIVEL_LEFT - Gyro-gesteuerte Linksweiche (20°)
//   SWIVEL_RIGHT- Gyro-gesteuerte Rechtsweiche (20°)
//   DODGE       - Reaktion auf Kollisionserkennung
//   MATADOR     - Angriffsmaneuver
//
// AUTOREN: Henri Lechtenfeld, Timo Dennis Görlich
// SCHULE:  TSA – Elektrotechnik - E2
// -------------------------------------------------------

#include <Wire.h>
#include <ZumoMotors.h>
#include <ZumoShield.h>
#include <L3G.h>

// ===========================================================
// PINS
// ===========================================================

#define LED              13
#define SERVO_PIN         6

#define TRIG_PIN_FRONT    3
#define ECHO_PIN_FRONT   A0
#define TRIG_PIN_REAR    11
#define ECHO_PIN_REAR    A2
#define TRIG_PIN_TOWER    2
#define ECHO_PIN_TOWER   A3

// ===========================================================
// KONSTANTEN
// ===========================================================

#define QTR_THRESHOLD     1800   
#define TRIM_PERCENT_LOW   4    
#define TRIM_PERCENT_HIGH  0

#define REVERSE_SPEED    200
#define TURN_SPEED       400
#define FORWARD_SPEED    237
#define FULL_SPEED       400

#define REVERSE_DURATION 300     // ms
#define CHARGE_DURATION  500     // ms

#define RIGHT   1
#define LEFT   -1

// ===========================================================
// OBJEKTE
// ===========================================================

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

#define NUM_SENSORS 2
unsigned int sensor_values[NUM_SENSORS];
byte sensorPins[] = {4, 5};
ZumoReflectanceSensorArray sensors(sensorPins, NUM_SENSORS, 2000, 2);

// -----------Gyro
L3G   gyro;
float gyroOffsetZ    = 0;
float gyroAngle      = 0;
float gyroTarget     = 0;
unsigned long gyroLastMicros = 0;
bool  gyroTurnActive = false;

// -----------Accelerometer
ZumoIMU imu;
float accelOffsetX = 0;
float accelOffsetY = 0;
float accelOffsetZ = 0;

// ===========================================================
// STATE MACHINE
// ===========================================================

enum State
{
    STATE_SEARCH,
    STATE_REVERSE,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_SPIN,
    STATE_CHARGE,
    STATE_SWIVEL_LEFT,
    STATE_SWIVEL_RIGHT
};

State         currentState   = STATE_SEARCH;
unsigned long stateStartTime = 0;
int           turnDirection  = RIGHT;
int           zigzagSide     = RIGHT;

// ===========================================================
// SERVO – TIMER2-ISR
// ===========================================================

uint16_t volatile servoTime     = 0;     // 0,5µs-Einheiten
uint16_t volatile servoHighTime = 3000;  // 1500µs default
boolean  volatile servoHigh     = false;

ISR(TIMER2_COMPA_vect)
{
    servoTime += OCR2A + 1;

    static uint16_t highTimeCopy  = 3000;
    static uint8_t  interruptCount = 0;

    if (servoHigh)
    {
        if (++interruptCount == 2)
        {OCR2A = 255;}
        if (servoTime >= highTimeCopy)
        {digitalWrite(SERVO_PIN, LOW);servoHigh=false;interruptCount = 0;}
    }
    else
    {
        if (servoTime >= 40000)
        {
            highTimeCopy = servoHighTime;
            digitalWrite(SERVO_PIN, HIGH);
            servoHigh      = true;
            servoTime      = 0;
            interruptCount = 0;
            OCR2A = ((highTimeCopy % 256) + 256) / 2 - 1;
        }
    }
}

void servoInit()
{
    digitalWrite(SERVO_PIN, LOW);
    pinMode(SERVO_PIN, OUTPUT);

    // Timer2 CTC-Modus, 1:8 Prescaler → 0,5µs Auflösung
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS21);
    TCNT2  = 0;
    OCR2A  = 255;
    TIMSK2 |= (1 << OCIE2A);
    sei();
}

// Pulsbreite setzen (in Mikrosekunden, z.B. 1000–2000)
void servoSetPosition(uint16_t highTimeMicroseconds)
{TIMSK2 &= ~(1 << OCIE2A);servoHighTime = highTimeMicroseconds * 2;TIMSK2 |= (1 << OCIE2A);}

// ===========================================================
// TURM-SWEEP
// ===========================================================

float         turmAngle      = 180.0;
int           turmDirection  = RIGHT;
float         turmSpeed      = 10.0;
unsigned long turmLastUpdate = 0;
const unsigned long turmInterval = 10;

// Winkel → Pulsbreite (µs) für 270°-Servo
uint16_t angleToMicros(float deg)
{
    float value = 500.0 + (deg / 270.0) * 2000.0;
    if (value < 500)  value = 500;
    if (value > 2500) value = 2500;
    return (uint16_t)value;
}

// ===========================================================
// SETUP
// ===========================================================

void setup()
{
    pinMode(LED, OUTPUT);
    waitForButtonAndCountDown();

    Serial.begin(9600);

    // -----------Gyro initialisieren (L3G)
    Wire.begin();
    if (!gyro.init())
    {
        Serial.println("Gyro nicht gefunden!");
    }
    gyro.enableDefault();
    Serial.println("Kalibriere Gyro...");

    long sum = 0;
    const int samples = 500;
    for (int i = 0; i < samples; i++)
    {
        gyro.read();
        sum += gyro.g.z;
        delay(0.5);
    }
    gyroOffsetZ = sum / (float)samples;

    // -----------Accelerometer initialisieren (LSM6DS33 via ZumoIMU)
    if (!imu.init())
    {Serial.println("IMU nicht gefunden!"); }
    imu.enableDefault();
    Serial.println("Kalibriere Accelerometer...");

    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; i++)
    {
        imu.read();
        sumX += imu.a.x;
        sumY += imu.a.y;
        sumZ += imu.a.z;
        delay(0.5);
    }
    accelOffsetX = sumX / (float)samples;
    accelOffsetY = sumY / (float)samples;
    accelOffsetZ = sumZ / (float)samples;

    // -----------Servo initialisieren (Timer2-ISR)
    servoInit();
    servoSetPosition(angleToMicros(turmAngle));
    Serial.println("Servo gestartet (Timer2-ISR).");

    // -----------Ultraschall-Pins
    pinMode(TRIG_PIN_FRONT, OUTPUT);
    pinMode(TRIG_PIN_REAR,  OUTPUT);
    pinMode(TRIG_PIN_TOWER, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(ECHO_PIN_REAR,  INPUT);
    pinMode(ECHO_PIN_TOWER, INPUT);

    Serial.println("Setup abgeschlossen.");
}

// ===========================================================
// HAUPTROUTINE
// ===========================================================

void loop()
{
    // -----------Knopfdruck im Betrieb → Neustart
    if (button.isPressed())
    {
        motors.setSpeeds(0, 0);
        button.waitForRelease();
        waitForButtonAndCountDown();
        setState(STATE_SEARCH);
    }

    // -----------Bodensensoren einlesen
    sensors.read(sensor_values);
    bool borderLeft  = sensor_values[0] < QTR_THRESHOLD;   // Pin 4
    bool borderRight = sensor_values[1] < QTR_THRESHOLD;   // Pin 5

    // -----------Randerkennung
    if ((borderLeft || borderRight) &&
      currentState != STATE_REVERSE &&
      currentState != STATE_TURN_LEFT &&
      currentState != STATE_TURN_RIGHT &&
      currentState != STATE_SPIN)
    {
        turnDirection = borderLeft ? RIGHT : LEFT;
        setState(STATE_REVERSE);
    }

    // -----------Ultraschall: Entfernungsmessung
    if (currentState == STATE_SEARCH || 
        currentState == STATE_CHARGE ||
        currentState == STATE_SWIVEL_RIGHT ||
        currentState == STATE_SWIVEL_LEFT)
    {
        float distFront = distance_mm(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        if (distFront > 0 && distFront < 300)
        {setState(STATE_CHARGE);}

        float distRear = distance_mm(TRIG_PIN_REAR, ECHO_PIN_REAR);
        if (distRear > 0 && distRear < 100)
        {turnDirection = RIGHT;setState(STATE_SPIN);}

        float distTower = distance_mm(TRIG_PIN_TOWER, ECHO_PIN_TOWER);
        if (distTower > 50 && distTower < 200)
        {
            if (turmAngle > 180) {setState(STATE_SWIVEL_LEFT);}
            if (turmAngle <= 180) {setState(STATE_SWIVEL_RIGHT);}
        }
    }

    // -----------Kollisionserkennung über Accelerometer
    imu.readAcc();
    float ax = (imu.a.x - accelOffsetX) / 16384.0;
    float ay = (imu.a.y - accelOffsetY) / 16384.0;
    float az = (imu.a.z - accelOffsetZ) / 16384.0;
    float accelSum = sqrt((ax * ax) + (ay * ay) + (az * az));

    if (accelSum > 0.5 && (currentState == STATE_SEARCH))
    {setState(STATE_CHARGE);}
    
    // -----------Turm-Sweep
    float distTower = distance_mm(TRIG_PIN_TOWER, ECHO_PIN_TOWER);
    if ((distTower > 200)||(distTower < 10))
    {
        unsigned long nowMs = millis();
        if ((nowMs - turmLastUpdate) >= turmInterval)
        {
            turmLastUpdate = nowMs;
            turmAngle += (turmDirection * turmSpeed);
            if (turmAngle >= 300.0)
            {turmAngle     = 300.0;turmDirection = LEFT;}
            else if (turmAngle <= 80.0)
            {turmAngle     = 80.0;turmDirection = RIGHT;}
            servoSetPosition(angleToMicros(turmAngle));
        }
    }

    // -----------STATE MACHINE
    switch (currentState)
    {
        case STATE_SEARCH:
            drive(FORWARD_SPEED, FORWARD_SPEED);
            break;

        case STATE_REVERSE:
            drive(-REVERSE_SPEED, -REVERSE_SPEED);
            if (elapsed() >= REVERSE_DURATION)
            {setState(turnDirection == RIGHT ? STATE_TURN_RIGHT : STATE_TURN_LEFT);}
            break;

        case STATE_TURN_RIGHT:
            gyroUpdate();
            drive(TURN_SPEED, -TURN_SPEED);
            if (gyroTurnDone() || elapsed() >= 2000)
            {gyroTurnActive = false;motors.setSpeeds(0, 0);setState(STATE_SEARCH);}
            break;

        case STATE_TURN_LEFT:
            gyroUpdate();
            drive(-TURN_SPEED, TURN_SPEED);
            if (gyroTurnDone() || elapsed() >= 2000)
            {gyroTurnActive = false;motors.setSpeeds(0, 0);setState(STATE_SEARCH);}
            break;

        case STATE_SPIN:
            gyroUpdate();
            if (turnDirection == RIGHT)
            {drive(TURN_SPEED, -TURN_SPEED);}
            else
            {drive(-TURN_SPEED, TURN_SPEED);}
            if (gyroTurnDone() || elapsed() >= 2000)
            {gyroTurnActive  = false;motors.setSpeeds(0, 0);setState(STATE_CHARGE);}
            break;

        case STATE_CHARGE:
            drive(FULL_SPEED, FULL_SPEED);
            if (elapsed() >= CHARGE_DURATION)
            {setState(STATE_SEARCH);}
            break;

        case STATE_SWIVEL_LEFT:
            gyroUpdate();
            drive(0, FULL_SPEED);
            if (elapsed() >= 100)
            {gyroTurnActive = false;setState(STATE_CHARGE);}
            break;

        case STATE_SWIVEL_RIGHT:
            gyroUpdate();
            drive(FULL_SPEED, 0);
            if (elapsed() >= 100)
            {gyroTurnActive = false;setState(STATE_CHARGE);}
            break;

        default:
            drive(FORWARD_SPEED, FORWARD_SPEED);
            currentState = STATE_SEARCH;
            break;
    }
}

// ===========================================================
// FUNKTIONEN
// ===========================================================

// -----------Button
void waitForButtonAndCountDown()
{
    digitalWrite(LED, HIGH);
    button.waitForButton();
    digitalWrite(LED, LOW);
    setState(STATE_SEARCH);
    delay(1000);                         // 1s Startpause (Wettkampfregel)
}

// -----------State Machine
void setState(State newState)
{
    currentState   = newState;
    stateStartTime = millis();

    if ((newState == STATE_TURN_RIGHT || newState == STATE_TURN_LEFT)
        && !gyroTurnActive)
    {gyroStartTurn(90);}
    if (newState == STATE_SPIN && !gyroTurnActive)
    {gyroStartTurn(180);}
    if ((newState == STATE_SWIVEL_RIGHT || newState == STATE_SWIVEL_LEFT)
        && !gyroTurnActive)
    {gyroStartTurn(20);}
}

unsigned long elapsed()
{return millis() - stateStartTime;}

// -----------Ultraschall
float distance_mm(int Trigger, int echo)
{
    digitalWrite(Trigger, LOW);
    delayMicroseconds(10);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    float duration = pulseIn(echo, HIGH, 30000);
    float distance = duration * 0.34 / 2.0;        // Schallgeschwindigkeit ~0,34 mm/µs
    return distance;
}

// -----------Gyro
void gyroUpdate()
{
    if (!gyroTurnActive) return;

    gyro.read();
    unsigned long now = micros();
    float dt          = (now - gyroLastMicros) / 1e6;
    gyroLastMicros    = now;

    float z_dps = (gyro.g.z - gyroOffsetZ) * 0.00875;
    gyroAngle  += z_dps * dt;
}

void gyroStartTurn(float degrees)
{
    gyroAngle      = 0;
    gyroTarget     = degrees;
    gyroLastMicros = micros();
    gyroTurnActive = true;
}

bool gyroTurnDone()
{return fabs(gyroAngle) >= fabs(gyroTarget);}

// -----------Drallausgleich
void drive(int left, int right)
{
    if ((left > 0 && right > 0) || (left < 0 && right < 0))
    {
        int absSpeed = abs(left);
        int trim     = (absSpeed <= 250) ? TRIM_PERCENT_LOW : TRIM_PERCENT_HIGH;
        left  = left  - (left  * trim / 100);
        right = right + (right * trim / 100);
    }
    motors.setSpeeds(left, right);
}

// ===========================================================
// ENDE
// ===========================================================
