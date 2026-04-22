/*
  WirelessReceiver.h — Tug-side state machine and communications handler.

  This is the receiver-side counterpart to WirelessController. It manages:
  - Wireless communication with the handheld controller
  - State machine (BOOT, NORMAL, COMM_ERR, ESTOP, IDLE, PWR_OFF)
  - Motor controller commands (delegated through IMotorController interface)
  - Accessory I/O via MCP23X17 expanders
  - Directional indicators based on throttle/steering

  The motor controller type is abstracted — this class works with RoboteQ,
  Curtis 1229/1228, PWM, or passthrough hardware without modification.

  If the motor controller also implements IDriveModeController and/or
  IDiagnosticSource, those capabilities are automatically detected and used.

  Hardware pin assignments and I2C addresses are passed via the
  WirelessReceiverConfig struct — no project-local headers are included
  by this library.

  Refactored from: Firmware/Wireless Code/Wireless_Receiver/src/Tug.h
  Created by Eric Alsop, February 26, 2022.
  Refactored March 20, 2026.
  Copyright 2022-2026 Best Tugs, LLC
*/
#ifndef WirelessReceiver_H
#define WirelessReceiver_H

#include <Arduino.h>
#include <Wire.h>
#include <WirelessComm.h>
#include <Timer.h>
#include <Adafruit_MCP23X17.h>
#include <PushButton.h>
#include <IMotorController.h>
#include <BitMasker.h>
#include <AccessoriesEnum.h>
#include <Servo.h>

#define IDLE_TIMER_DURATION            1800000  // 30 minutes — STANDBY before deep sleep
#define LOST_TIMER_DURATION            120000   // 2 minutes — LOST before deep sleep
#define PWR_OFF_CONFIRMED_TIMER_DURATION 10000  // 10 seconds
#define ESTOP_MIN_POWER_OFF_MS         2000     // Min time system power stays off during ESTOP (for motor controller reset)

// ─── Hardware Configuration ─────────────────────────────────────────────────
// Passed by the project .ino so the library never includes ProjectDefines.h.

struct WirelessReceiverConfig
{
    // Board-level pins
    uint8_t boardPwrOffPin;
    uint8_t externalStatusLedPin;
    uint8_t tugBatPin;             // ADC pin for tug battery voltage divider (e.g. 15/A1)
    uint8_t lazySusanPwmPin;       // PWM pin for lazy susan servo/solenoid
    uint8_t lazySusanAngleClose;   // Servo angle when ROTATE_UNLOCK is inactive (locked)
    uint8_t lazySusanAngleOpen;    // Servo angle when ROTATE_UNLOCK is active (unlocked)
    uint8_t rotationLockInputPin;  // Microswitch: LOW = rotation unlocked
    uint8_t cradleLockInputPin;    // Microswitch: LOW = cradle locked, HIGH = cradle unlocked

    // I2C expander addresses and bus
    uint8_t outputExpanderAddr;
    uint8_t inputExpanderAddr;
    TwoWire *ioExpanderWire;          // e.g. &Wire2

    // Output-expander pin map (directly drives relays / LEDs on the tug)
    uint8_t systemPwrPin;
    uint8_t headlightsPin;
    uint8_t airCompressorPin;
    uint8_t rotateRelayPin;
    uint8_t ezLoadRelayPin;
    uint8_t underGlowPin;
    uint8_t dirIndFwdLedPin;
    uint8_t dirIndRvrsLedPin;
    uint8_t dirIndLeftLedPin;
    uint8_t dirIndRightLedPin;
    uint8_t winchOutPin;
    uint8_t winchInPin;
    uint8_t lWingUpPin;
    uint8_t lWingDownPin;
    uint8_t rWingUpPin;
    uint8_t rWingDownPin;
};

class WirelessReceiver
{
public:
    WirelessReceiver() {} // Do not use

    WirelessReceiver(
        WirelessComm &comm,
        IMotorController &motor,
        Adafruit_MCP23X17 &outputExpander,
        Adafruit_MCP23X17 &inputExpander,
        PushButton &eStopButton,
        const WirelessReceiverConfig &config);

    WirelessComm comm;
    sysState_e sysState;

    // ── Tug → Controller data ──
    int16_t torque;
    int16_t tugBatLvl;
    uint16_t accsStatus;

    // ── Controller → Tug data ──
    int16_t throttle;
    int16_t steering;
    uint16_t accsCmnds;
    sysState_e ctrlrState;

    // ── Local state ──
    bool systemReady;
    Timer pwrOffTimer;
    Timer pwrOffConfirmedTimer;

    // ── Motor diagnostics (populated if motor supports IDiagnosticSource) ──
    uint16_t motorErrorCode;
    uint16_t motorStatusFlags;

    void setup();
    void update();

private:
    WirelessReceiverConfig cfg;
    Adafruit_MCP23X17 outputExpander;
    Adafruit_MCP23X17 inputExpander;
    uint8_t maxPinNum;
    PushButton eStopButton;

    // Motor controller — accessed through the abstract interface.
    // Optional capability pointers are set at construction time.
    IMotorController *motor;
    IDriveModeController *driveModes;    // nullptr if motor doesn't support it
    IDiagnosticSource *diagnostics;      // nullptr if motor doesn't support it
    Servo lazySusanServo;

    void setOutputs();
    void handleComm();
    void ioExpanderSetAllPinModes(Adafruit_MCP23X17 &expander, uint8_t mode);
    void readHardware();
    void writeHardware();
    void loadPacketToTx();
    void extractReceivedData();
    void handleStateChanges();
    void systemPowerOff();
    void systemPowerOn();
    void setDirectionalIndicators();
    void boardPowerOff();
    void updateMotorDiagnostics();
    void readTugBattery();
    unsigned long lastBatReadTime;
    unsigned long eStopEnteredTime;

    // Previous accsCmnds value used to log only output transitions in writeHardware().
    uint16_t prevAccsCmndsLogged;
};

#endif // WirelessReceiver_H
