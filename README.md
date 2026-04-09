# Zumo Sumo-Roboter — Projektarbeit E2 Team 2 (2026)

**Technikerschule Allgäu - Kempten (Allgäu) — Schaltungstechnik / Elektrotechnik**

Autonomer Sumo-Kampfroboter auf Basis des Pololu Zumo Shield V1.2, entwickelt als Schulprojekt.

---

## Überblick

Der Roboter erkennt Gegner per Ultraschall, bleibt innerhalb des Rings durch Infrarot-Bodensensoren und trifft alle Entscheidungen über eine nicht-blockierende Zustandsmaschine (`millis()`-basiert, kein `delay()`).

**Teammitglieder:**
- Parabellum — Sensorik, Verkabelung, Zustandsmaschine, Firmware-Integration
- Henri Lechtenfeld — Gyro-Integration, IMU-Code

---

## Hardware

| Komponente | Details |
|---|---|
| Mikrocontroller | Elegoo UNO R3 (ATmega328P) |
| Shield | Pololu Zumo Shield V1.3 |
| Ultraschall (Front) | HC-SR04 — Trig D3 / Echo A0 |
| Ultraschall (Heck) | HC-SR04 — Trig D11 / Echo A2 |
| Ultraschall (Turm) | HC-SR04 — Trig D2 / Echo A3 |
| Servo-Turm | SG90 auf Pin 6 (Timer2-ISR, **kein** Servo.h) |
| Bodensensoren | ZumoReflectanceSensorArray — 2 Sensoren, Pins {4, 5} |
| Gyroskop | L3G über I2C |
| Beschleunigungssensor | LSM6DS33 über ZumoShield.h |
| Motoren | ZumoMotors (Timer1-PWM) |

---

## Timer-Belegung

| Timer | Verwendung |
|---|---|
| Timer0 | `millis()` / `micros()` (System) |
| Timer1 | ZumoMotors (PWM) |
| Timer2 | Servo-ISR (Buzzer-Jumper physisch umfunktioniert) |

> **Servo.h ist verboten** — kollidiert mit ZumoMotors über Timer1.  
> **ZumoBuzzer darf nie aufgerufen werden** — Jumper ist physisch umgelötet.

---

## Software-Architektur

- **Nicht-blockierende Zustandsmaschine** — alle Zustände verwenden `millis()`-basierte Zeitsteuerung
- **ISR-basierter Turm-Sweep** — Servo-Steuerung läuft komplett in der Timer2-ISR mit Integer-Arithmetik, unabhängig von Timer0
- **Non-blocking Ultraschall** — Struct-basiertes Polling mit `micros()` statt blockierendem `pulseIn()`
- **Echo-Active Guard** — Servo-Pulse werden während aktiver Ultraschall-Echo-Fenster unterdrückt

### Zustände (Auszug)

`STATE_COUNTDOWN` → `STATE_SEARCH` → `STATE_CHARGE` → ...  
`STATE_BORDER` → Randerkennung und Umkehr  
`STATE_MATADOR` (geplant) → Ausweichmanöver bei frontaler Gegnererkennung

---

## Pinbelegung

```
D2  — Turm-Ultraschall TRIG
D3  — Front-Ultraschall TRIG
D4  — Reflektanz-Sensor (rechts)
D5  — Reflektanz-Sensor (links)
D6  — Servo-Turm (PWM via Timer2-ISR)
D7  — ZumoMotors (Richtung rechts)
D8  — ZumoMotors (Richtung links)
D9  — ZumoMotors (PWM rechts, Timer1)
D10 — ZumoMotors (PWM links, Timer1)
D11 — Heck-Ultraschall TRIG
A0  — Front-Ultraschall ECHO
A2  — Heck-Ultraschall ECHO
A3  — Turm-Ultraschall ECHO
A4  — I2C SDA (IMU / Gyro)
A5  — I2C SCL (IMU / Gyro)
```

---

## Build & Upload

1. [Arduino IDE](https://www.arduino.cc/en/software) installieren
2. Pololu ZumoShield-Bibliothek installieren: **Sketch → Bibliothek einbinden → Bibliotheken verwalten → "ZumoShield"**
3. Board: **Arduino Uno** auswählen
4. `.ino`-Datei öffnen und hochladen

---

## Datenblätter & Dokumentation

- [Pololu Zumo Shield V1.3 — User Manual](https://www.pololu.com/docs/0J57)
- [HC-SR04 Ultraschallsensor — Datenblatt](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- [LSM6DS33 IMU — Datenblatt](https://www.pololu.com/file/0J1087/LSM6DS33.pdf)

---

## Lizenz

Schulprojekt — TSA Kempten 2026
