/*
  WirelessReceiver.cpp — Tug-side state machine and communications handler.

  Refactored from: Firmware/Wireless Code/Wireless_Receiver/src/Tug.cpp
  Key changes:
  - Motor controller accessed through IMotorController interface
    instead of concrete MotorController class. Optional IDriveModeController
    and IDiagnosticSource capabilities are auto-detected.
  - Hardware pin assignments and I2C addresses are received via
    WirelessReceiverConfig — no project-local headers are included.

  Created by Eric Alsop, February 26, 2022.
  Refactored March 20, 2026.
  Copyright 2022-2026 Best Tugs, LLC
*/
#include "WirelessReceiver.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
// #define SERIAL_DEBUG_LEVEL_2_ENABLED
// #define SERIAL_DEBUG_LEVEL_3_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

WirelessReceiver::WirelessReceiver(
    WirelessComm &comm,
    IMotorController &motor,
    Adafruit_MCP23X17 &outputExpander,
    Adafruit_MCP23X17 &inputExpander,
    PushButton &eStopButton,
    const WirelessReceiverConfig &config)
{
    this->comm = comm;
    this->motor = &motor;
    this->outputExpander = outputExpander;
    this->inputExpander = inputExpander;
    this->eStopButton = eStopButton;
    this->cfg = config;

    // Detect optional capabilities (no RTTI on Arduino, so use virtual query)
    this->driveModes = motor.asDriveModeController();
    this->diagnostics = motor.asDiagnosticSource();

    systemReady = false;
    throttle = NEUTRAL;
    steering = STRAIGHT;
    torque = 0xACED;       // TODO: not actually measured yet
    tugBatLvl = 0;
    lastBatReadTime = 0;
    accsCmnds = 0;
    accsStatus = 0;
    sysState = BOOT;
    ctrlrState = BOOT;
    motorErrorCode = 0;
    motorStatusFlags = 0;
    pwrOffTimer = Timer(IDLE_TIMER_DURATION, false);
    pwrOffConfirmedTimer = Timer(PWR_OFF_CONFIRMED_TIMER_DURATION, false);
    prevAccsCmndsLogged = 0;
}

void WirelessReceiver::setup()
{
    Serial.begin(115200);
    D1PRINTLN("Starting Best Tugs Receiver");

    if (outputExpander.begin_I2C(cfg.outputExpanderAddr, cfg.ioExpanderWire))
    {
        D1PRINTLN("Output io expander init: PASS\n");
        ioExpanderSetAllPinModes(outputExpander, OUTPUT);
    }
    else
    {
        ERRORPRINTLN("Output io expander init: FAIL\n");
    }

    if (inputExpander.begin_I2C(cfg.inputExpanderAddr, cfg.ioExpanderWire))
    {
        D1PRINTLN("Input io expander init: PASS\n");
        ioExpanderSetAllPinModes(inputExpander, INPUT);
    }
    else
    {
        ERRORPRINTLN("Input io expander init: FAIL\n");
    }

    accsCmnds = 0;
    eStopButton.init();
    pinMode(cfg.boardPwrOffPin, OUTPUT);
    digitalWrite(cfg.boardPwrOffPin, LOW);
    pinMode(cfg.externalStatusLedPin, OUTPUT);
    digitalWrite(cfg.externalStatusLedPin, LOW);

    pinMode(cfg.tugBatPin, INPUT);
    pinMode(cfg.rotationLockInputPin, INPUT_PULLUP);
    pinMode(cfg.cradleLockInputPin, INPUT_PULLUP);
    motor->init();
    lazySusanServo.attach(cfg.lazySusanPwmPin);
    lazySusanServo.write(cfg.lazySusanAngleClose);
    writeHardware();
    comm.setup();
    systemPowerOn();
    D1PRINTLN("WirelessReceiver setup complete");
}

void WirelessReceiver::readTugBattery()
{
    if (millis() - lastBatReadTime >= 5000)
    {
        lastBatReadTime = millis();
        tugBatLvl = analogRead(cfg.tugBatPin);
        D1PRINT("Tug battery ADC (pin ");
        D1PRINT(cfg.tugBatPin);
        D1PRINT("): ");
        D1PRINTLN(tugBatLvl);
    }
}

void WirelessReceiver::update()
{
    readTugBattery();
    handleComm();
    handleStateChanges();
    updateMotorDiagnostics();

    static unsigned long prevPrintTime = 0;
#ifdef SERIAL_DEBUG_LEVEL_1_ENABLED
    if (pwrOffTimer.isRunning())
    {
        if (millis() - prevPrintTime > 100)
        {
            D1PRINTVAR(pwrOffTimer.getTimerValue());
            D1PRINTVAR(pwrOffTimer.getInterval());
            prevPrintTime = millis();
        }
    }
#endif

    switch (sysState)
    {
    case BOOT:
        if (millis() - prevPrintTime > 10)
        {
            D1PRINTLN("System just entered state: BOOT");
            prevPrintTime = millis();
        }
        if (!pwrOffTimer.isRunning())
        {
            pwrOffTimer.start(IDLE_TIMER_DURATION);
        }

        if (XBEE_ERR_NO_ADDR == comm.getStatusCode())
        {
            ERRORPRINTLN("No XBee Address.");
            break;
        }

        systemReady = true;
        break;

    case NORMAL:
        if (millis() - prevPrintTime > 10)
        {
            D1PRINTLN("System just entered state: NORMAL");
            prevPrintTime = millis();
        }
        if (pwrOffTimer.isRunning())
        {
            pwrOffTimer.stop();
        }
        if (motor->isEStopped() || motor->isSafetyStopped())
        {
            D1PRINTLN("Releasing motor stop on entry to NORMAL");
            motor->releaseStop();
        }
        break;

    case COMM_ERR:
        if (millis() - prevPrintTime > 10)
        {
            D1PRINTLN("System just entered state: COMM_ERR");
#ifdef SERIAL_DEBUG_LEVEL_1_ENABLED
            comm.printCommStatus();
#endif
            prevPrintTime = millis();
        }
        if (CONN_LOST == comm.getStatusCode() && !pwrOffTimer.isRunning())
        {
            pwrOffTimer.start(LOST_TIMER_DURATION);
        }
        throttle = NEUTRAL;
        steering = STRAIGHT;
        motor->safetyStop();
        setOutputs();
        break;

    case ESTOP:
        if (millis() - prevPrintTime > 10)
        {
            D1PRINTLN("System just entered state: ESTOP");
            prevPrintTime = millis();
        }
        throttle = NEUTRAL;
        steering = STRAIGHT;
        motor->eStop();
        systemPowerOff();
        if (!pwrOffTimer.isRunning())
        {
            pwrOffTimer.start(IDLE_TIMER_DURATION);
        }
        break;

    case IDLE:
        if (millis() - prevPrintTime > 10)
        {
            D1PRINTLN("System just entered state: IDLE");
            prevPrintTime = millis();
        }
        if (!pwrOffTimer.isRunning() || IDLE_TIMER_DURATION != pwrOffTimer.getInterval())
        {
            pwrOffTimer.start(IDLE_TIMER_DURATION);
        }
        break;

    case PWR_OFF:
        motor->eStop();
        if (millis() - prevPrintTime > 50)
        {
            D1PRINTLN("System just entered state: PWR_OFF");
            prevPrintTime = millis();
        }
        if (pwrOffTimer.isRunning())
        {
            pwrOffTimer.stop();
        }

        if (!pwrOffConfirmedTimer.isRunning())
        {
            D1PRINTLN("Cut off the main battery power");
            pwrOffConfirmedTimer.start();
        }
#ifdef SERIAL_DEBUG_LEVEL_1_ENABLED
        if (pwrOffConfirmedTimer.isRunning())
        {
            if (millis() - prevPrintTime > 20)
            {
                D1PRINTVAR(pwrOffConfirmedTimer.getTimerValue());
                prevPrintTime = millis();
            }
        }
#endif

        if (PWR_OFF == ctrlrState || pwrOffConfirmedTimer.isFinished())
        {
            if (PWR_OFF == ctrlrState)
            {
                D1PRINTLN("Power off command acknowledged");
            }
            systemPowerOff();
            boardPowerOff();
        }
        break;

    default:
        ERRORPRINTLN("WirelessReceiver sysState is in an unknown state");
        break;
    }
}

void WirelessReceiver::ioExpanderSetAllPinModes(Adafruit_MCP23X17 &expander, uint8_t mode)
{
    for (uint8_t pin = 0; pin <= 15; pin++)
    {
        expander.pinMode(pin, mode);
    }
}

void WirelessReceiver::handleComm()
{
    readHardware();
    loadPacketToTx();
    comm.update();
    if (comm.rxdDataReadyToUse)
    {
        extractReceivedData();
        setOutputs();
        comm.rxdDataReadyToUse = false;
    }
}

void WirelessReceiver::readHardware()
{
    D2PRINTLN("Entering readHardware");

    // R04D output-state feedback via U6 (input expander) sense lines. Each BDEX output's connector
    // pin is divided + buffered to a U6 channel: ~HIGH when the output is OFF (pin near +12V), LOW
    // when ACTIVE (pin switched to ground). So active = (read == LOW). [Bench-verify polarity.]
    BitMasker::setBit(accsStatus, TUG_SYSTEM_PWR,   inputExpander.digitalRead(cfg.ksiInPin)           == LOW); // KSI Out
    BitMasker::setBit(accsStatus, HEADLIGHTS,       inputExpander.digitalRead(cfg.headlightsInPin)    == LOW);
    BitMasker::setBit(accsStatus, AIR_COMPRESSOR,   inputExpander.digitalRead(cfg.airCompressorInPin) == LOW);
    BitMasker::setBit(accsStatus, FORWARD_LIGHT,    inputExpander.digitalRead(cfg.dirIndFwdInPin)     == LOW);
    BitMasker::setBit(accsStatus, BACKWARD_LIGHT,   inputExpander.digitalRead(cfg.dirIndRvrsInPin)    == LOW);
    BitMasker::setBit(accsStatus, LEFT_TURN_LIGHT,  inputExpander.digitalRead(cfg.dirIndLeftInPin)    == LOW);
    BitMasker::setBit(accsStatus, RIGHT_TURN_LIGHT, inputExpander.digitalRead(cfg.dirIndRightInPin)   == LOW);
    BitMasker::setBit(accsStatus, UNDER_GLOW,       inputExpander.digitalRead(cfg.underGlowInPin)     == LOW);

    // Lock state from the microswitches (Teensy GPIO, not the expander):
    // rotationLockInputPin LOW = rotation unlocked; cradleLockInputPin HIGH = cradle unlocked.
    BitMasker::setBit(accsStatus, ROTATE_UNLOCK,  digitalRead(cfg.rotationLockInputPin) == LOW);
    BitMasker::setBit(accsStatus, EZ_LOAD_UNLOCK, digitalRead(cfg.cradleLockInputPin) == HIGH);

    // Winch + wings are on HB half-bridges (no U6 sense line) -> echo the commanded state.
    BitMasker::setBit(accsStatus, WINCH_OUT,   BitMasker::getIsActive(accsCmnds, WINCH_OUT));
    BitMasker::setBit(accsStatus, WINCH_IN,    BitMasker::getIsActive(accsCmnds, WINCH_IN));
    BitMasker::setBit(accsStatus, L_WING_UP,   BitMasker::getIsActive(accsCmnds, L_WING_UP));
    BitMasker::setBit(accsStatus, L_WING_DOWN, BitMasker::getIsActive(accsCmnds, L_WING_DOWN));
    BitMasker::setBit(accsStatus, R_WING_UP,   BitMasker::getIsActive(accsCmnds, R_WING_UP));
    BitMasker::setBit(accsStatus, R_WING_DOWN, BitMasker::getIsActive(accsCmnds, R_WING_DOWN));

    D2PRINTLN("Leaving readHardware");
}

void WirelessReceiver::loadPacketToTx()
{
    D2PRINTLN("Entering loadPacketToTx");

    comm.activePacket.item1 = torque;
    comm.activePacket.item2 = tugBatLvl;
    comm.activePacket.sysState = sysState;
    comm.nullPacket.sysState = sysState;
    comm.activePacket.accsData = accsStatus;

    comm.activePacket.setMotorErrorCode(motorErrorCode);
    comm.activePacket.setMotorStatusFlags(motorStatusFlags);
}

void WirelessReceiver::extractReceivedData()
{
    throttle = comm.receivedPacket.getItem1();
    steering = comm.receivedPacket.getItem2();
    ctrlrState = comm.receivedPacket.getSysState();
    bool tugSysPwrIsOn = BitMasker::getIsActive(accsCmnds, TUG_SYSTEM_PWR);
    accsCmnds = comm.receivedPacket.getAccsData();
    BitMasker::setBit(accsCmnds, TUG_SYSTEM_PWR, tugSysPwrIsOn);

    // TODO: Extract drive mode change request from controller packet if applicable
    // if (driveModes && driveModeChangeRequested) {
    //     driveModes->setDriveMode(requestedMode);
    // }
}

void WirelessReceiver::setOutputs()
{
    D2PRINTLN("Just entered setOutputs");
    if (NORMAL == sysState)
    {
        D1PRINTVAR(throttle);
        motor->setThrottle(throttle);
        D1PRINTVAR(steering);
        motor->setSteering(steering);
    }
    else
    {
        motor->setSteering(STRAIGHT);
        motor->setThrottle(NEUTRAL);
    }
    motor->update();
    setDirectionalIndicators();
    writeHardware();
}

void WirelessReceiver::setDirectionalIndicators()
{
    if (steering > 0)
    {
        BitMasker::setBit(accsCmnds, RIGHT_TURN_LIGHT, HIGH);
        BitMasker::setBit(accsCmnds, LEFT_TURN_LIGHT, LOW);
    }
    else if (steering < 0)
    {
        BitMasker::setBit(accsCmnds, LEFT_TURN_LIGHT, HIGH);
        BitMasker::setBit(accsCmnds, RIGHT_TURN_LIGHT, LOW);
    }
    else if (0 == steering || motor->isSafetyStopped())
    {
        BitMasker::setBit(accsCmnds, LEFT_TURN_LIGHT, LOW);
        BitMasker::setBit(accsCmnds, RIGHT_TURN_LIGHT, LOW);
    }

    if (throttle > 0)
    {
        BitMasker::setBit(accsCmnds, FORWARD_LIGHT, HIGH);
        BitMasker::setBit(accsCmnds, BACKWARD_LIGHT, LOW);
    }
    else if (throttle < 0)
    {
        BitMasker::setBit(accsCmnds, BACKWARD_LIGHT, HIGH);
        BitMasker::setBit(accsCmnds, FORWARD_LIGHT, LOW);
    }
    else if (0 == throttle || motor->isSafetyStopped())
    {
        BitMasker::setBit(accsCmnds, BACKWARD_LIGHT, LOW);
        BitMasker::setBit(accsCmnds, FORWARD_LIGHT, LOW);
    }
}

void WirelessReceiver::writeHardware()
{
    D2PRINTLN("Entering writeHardware");

#ifdef SERIAL_DEBUG_LEVEL_1_ENABLED
    {
        uint16_t changed = accsCmnds ^ prevAccsCmndsLogged;
        if (changed)
        {
            // Names indexed by accsBits_e. Matches AccessoriesEnum.h bit order.
            static const char *const kAccsNames[16] = {
                "TUG_SYSTEM_PWR",   "HEADLIGHTS",       "AIR_COMPRESSOR", "ROTATE_UNLOCK",
                "EZ_LOAD_UNLOCK",   "UNDER_GLOW",       "FORWARD_LIGHT",  "BACKWARD_LIGHT",
                "LEFT_TURN_LIGHT",  "RIGHT_TURN_LIGHT", "WINCH_OUT",      "WINCH_IN",
                "L_WING_UP",        "L_WING_DOWN",      "R_WING_UP",      "R_WING_DOWN"
            };
            for (uint8_t i = 0; i < 16; ++i)
            {
                if (changed & (uint16_t)(1u << i))
                {
                    bool nowOn = (accsCmnds & (uint16_t)(1u << i)) != 0;
                    D1PRINT("Accessory ");
                    D1PRINT(kAccsNames[i]);
                    D1PRINT(" -> ");
                    D1PRINTLN(nowOn ? "ON" : "OFF");
                }
            }
            prevAccsCmndsLogged = accsCmnds;
        }
    }
#endif

    // R04D: low-side ground-switching outputs are ACTIVE-HIGH from the Teensy (expander HIGH ->
    // DGD0216 -> N-FET ON -> switches the device's ground -> device ON). When the Teensy is
    // unpowered or the expander floats, gate/input pulldowns hold the FET OFF -> device OFF
    // (fail-safe, per Eric's design requirement). The connector pin reads ~12V when off and is
    // pulled to ground when active -- that is the "default HIGH, LOW when active" behavior Nathan
    // describes at the OUTPUT pin; the control sense here at the expander stays active-HIGH.
    // KSI Out replaces the old system-power relay.
    outputExpander.digitalWrite(cfg.systemPwrPin,      BitMasker::getIsActive(accsCmnds, TUG_SYSTEM_PWR)); // KSI Out
    outputExpander.digitalWrite(cfg.headlightsPin,     BitMasker::getIsActive(accsCmnds, HEADLIGHTS));
    outputExpander.digitalWrite(cfg.airCompressorPin,  BitMasker::getIsActive(accsCmnds, AIR_COMPRESSOR));
    outputExpander.digitalWrite(cfg.underGlowPin,      BitMasker::getIsActive(accsCmnds, UNDER_GLOW));
    outputExpander.digitalWrite(cfg.dirIndFwdLedPin,   BitMasker::getIsActive(accsCmnds, FORWARD_LIGHT));
    outputExpander.digitalWrite(cfg.dirIndRvrsLedPin,  BitMasker::getIsActive(accsCmnds, BACKWARD_LIGHT));
    outputExpander.digitalWrite(cfg.dirIndLeftLedPin,  BitMasker::getIsActive(accsCmnds, LEFT_TURN_LIGHT));
    // (cfg.dirIndRightLedPin / pin 24 is NOT driven from RIGHT_TURN_LIGHT anymore -- repurposed to the
    //  lazy-susan unlock solenoid, driven to match pin 6 in the rotation-lock block below.)
    // NOTE: winch in/out are on HB half-bridge (push-pull) pins, not DGD0216 low-side channels.
    // Driven active-HIGH here for consistency; confirm winch motor wiring + fail-safe with Nathan.
    outputExpander.digitalWrite(cfg.winchOutPin,       BitMasker::getIsActive(accsCmnds, WINCH_OUT));
    outputExpander.digitalWrite(cfg.winchInPin,        BitMasker::getIsActive(accsCmnds, WINCH_IN));

    // R04D: EZ-load and rotation locks are H-bridges driven as an opposed pair.
    // Rest/locked = bridgeA HIGH, bridgeB LOW. Active/unlock ("Load" selected) = bridgeA LOW, bridgeB HIGH.
    bool ezLoadUnlock = BitMasker::getIsActive(accsCmnds, EZ_LOAD_UNLOCK);
    outputExpander.digitalWrite(cfg.ezLoadBridgeAPin, ezLoadUnlock ? LOW : HIGH);
    outputExpander.digitalWrite(cfg.ezLoadBridgeBPin, ezLoadUnlock ? HIGH : LOW);
    bool rotateUnlock = BitMasker::getIsActive(accsCmnds, ROTATE_UNLOCK);
    outputExpander.digitalWrite(cfg.rotateBridgeAPin, rotateUnlock ? LOW : HIGH);
    outputExpander.digitalWrite(cfg.rotateBridgeBPin, rotateUnlock ? HIGH : LOW);
    // Pin 24 (BDEX_16) drives the 16" lazy-susan unlock 24V solenoid directly (low-side, since the
    // H-bridge pin can't), matching the rotation-lock bridge-A pin (pin 6) so they track together.
    outputExpander.digitalWrite(cfg.dirIndRightLedPin, rotateUnlock ? LOW : HIGH);
    // Helipad wings (L/R) reuse the lock H-bridge pins on Helipad builds; not driven on Romeo.

    // Lazy-susan LOAD/UNLOAD servo PWM. Per Nathan (R04D): the servo is the load/unload
    // mechanism on small lazy susans, and it must trigger TOGETHER with the EZ-load
    // H-bridge actuators (large lazy susans) — same EZ_LOAD_UNLOCK command drives both,
    // so one "Load" control works for both unit sizes. (Was previously on ROTATE_UNLOCK.)
    if (ezLoadUnlock)
    {
        lazySusanServo.write(cfg.lazySusanAngleOpen);
        D1PERIODICPRINTLN(1000, "Lazy susan load/unload servo -> OPEN");
    }
    else
    {
        lazySusanServo.write(cfg.lazySusanAngleClose);
        D1PERIODICPRINTLN(1000, "Lazy susan load/unload servo -> CLOSE");
    }

    D2PRINTLN("Leaving writeHardware");
}

void WirelessReceiver::handleStateChanges()
{
    if (PWR_OFF == sysState)
    {
        return;
    }

    if (pwrOffTimer.isFinished())
    {
        D1PRINTLN("Timer finished. Setting sysState to PWR_OFF");
        sysState = PWR_OFF;
        return;
    }

    if (BOOT == sysState)
    {
        if (COMM_NORMAL == comm.getStatusCode() && systemReady)
        {
            D1PRINTLN("Comm is normal and system is ready. Leaving BOOT and setting sysState to NORMAL");
            sysState = NORMAL;
        }
        return;
    }

    if (eStopButton.isPressed() || ESTOP == ctrlrState)
    {
        // Local e-stop button or controller reporting ESTOP — enter/stay in ESTOP
        if (ESTOP != sysState)
        {
            D1PRINTLN("Entering ESTOP");
            eStopEnteredTime = millis();
        }
        sysState = ESTOP;
        return;
    }

    if (ESTOP == sysState)
    {
        // We're in ESTOP but neither the local button nor the controller is
        // in ESTOP anymore — recover. Controller e-stop release is the
        // acknowledgment that clears the system.
        // System power (pin #3) has been off the entire time we were in ESTOP.
        // If we've been in ESTOP for at least 2 seconds, motor controllers
        // have had enough time to fully reset (required for Curtis 1229).
        unsigned long timeInEStop = millis() - eStopEnteredTime;
        if (timeInEStop < ESTOP_MIN_POWER_OFF_MS)
        {
            D1PRINT("ESTOP clear pending — waiting for motor controller reset: ");
            D1PRINTLN(ESTOP_MIN_POWER_OFF_MS - timeInEStop);
            return;
        }
        D1PRINTLN("ESTOP cleared — recovering");
        sysState = NORMAL;
        systemPowerOn();
        motor->releaseStop();
        pwrOffTimer.stop();
        // Fall through to normal state checks below
    }

    if (COMM_NORMAL != comm.getStatusCode())
    {
        if (COMM_ERR != sysState)
        {
            D1PRINTLN("Status code is not Normal. Setting sysState to COMM_ERR");
        }
        sysState = COMM_ERR;
        return;
    }

    switch (ctrlrState)
    {
    case IDLE:
        if (IDLE != sysState)
        {
            D1PRINTLN("Ctrlr is IDLE. Setting sysState to IDLE");
            sysState = IDLE;
        }
        break;
    case PWR_OFF:
        if (IDLE == sysState)
        {
            D1PRINTLN("Got a pwr_off cmnd while in IDLE. Changing sysState to PWR_OFF");
            sysState = PWR_OFF;
        }
        break;
    case ESTOP:
        // Handled above before the switch — controller ESTOP is caught
        // in the eStopButton.isPressed() || ESTOP == ctrlrState check.
        break;
    case NORMAL:
        if (COMM_NORMAL == comm.getStatusCode() && NORMAL != sysState)
        {
            D1PRINTLN("Ctrlr and statusCode are Normal. Setting sysState to NORMAL");
            sysState = NORMAL;
            return;
        }
        break;
    case COMM_ERR:
        break;
    case BOOT:
        break;
    default:
        ERRORPRINTLN("Unexpected value in receivedPacket syst_state");
        break;
    }
}

void WirelessReceiver::updateMotorDiagnostics()
{
    if (diagnostics)
    {
        motorErrorCode = diagnostics->getErrorCode();
        motorStatusFlags = diagnostics->getStatusFlags();
    }
    else
    {
        motorErrorCode = 0;
        motorStatusFlags = 0;
    }
}

void WirelessReceiver::systemPowerOff()
{
    D1PRINTLN("Shutting OFF power now");
    BitMasker::setBit(accsCmnds, TUG_SYSTEM_PWR, LOW);
    outputExpander.digitalWrite(cfg.systemPwrPin, LOW);
    digitalWrite(cfg.externalStatusLedPin, LOW);
}

void WirelessReceiver::systemPowerOn()
{
    D1PRINTLN("Turning ON power now");
    BitMasker::setBit(accsCmnds, TUG_SYSTEM_PWR, HIGH);
    outputExpander.digitalWrite(cfg.systemPwrPin, HIGH);
    digitalWrite(cfg.externalStatusLedPin, HIGH);
}

void WirelessReceiver::boardPowerOff()
{
    D1PRINTLN("Shutting off power now");
    digitalWrite(cfg.boardPwrOffPin, HIGH);
}
