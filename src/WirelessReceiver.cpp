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
    PushButton &pairButton,
    const WirelessReceiverConfig &config)
{
    this->comm = comm;
    this->motor = &motor;
    this->outputExpander = outputExpander;
    this->inputExpander = inputExpander;
    this->eStopButton = eStopButton;
    this->pairButton = pairButton;
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
    eStopLatched = false;
    pwrOffTimer = Timer(IDLE_TIMER_DURATION, false);
    pwrOffConfirmedTimer = Timer(PWR_OFF_CONFIRMED_TIMER_DURATION, false);
    prevAccsCmndsLogged = 0;

    pairWindowTimer = Timer(PAIR_WINDOW_DURATION_MS, false);
    pairingWindowOpen = false;
    pairButtonWasPressed = false;
    pairLedMode = PAIR_LED_NORMAL;
    pairLedState = false;
    pairLedToggleTime = 0;
    pairResultHoldUntil = 0;
}

void WirelessReceiver::setup()
{
    Serial.begin(115200);
    D1PRINTLN("Starting Best Tugs Receiver");

    accsCmnds = 0;

    if (outputExpander.begin_I2C(cfg.outputExpanderAddr, cfg.ioExpanderWire))
    {
        D1PRINTLN("Output io expander init: PASS\n");
        // Pre-load idle levels on the H-bridge pins BEFORE flipping IODIR to OUTPUT: the
        // MCP23017's output latch resets to 0, so going straight to OUTPUT drives every HB pin
        // LOW for the window until writeHardware() runs (on the lock bridges that yanks the
        // rest pair away; on the winch pair it turns the outputs on). Writing
        // OLAT while the pins are still inputs makes the OUTPUT transition glitchless. Winch
        // idles HIGH/HIGH; the lock bridges idle at their rest pair, or all-HIGH (motors off)
        // when the bridges drive the Papa/Helipad wings.
        outputExpander.digitalWrite(cfg.winchOutPin, HIGH);
        outputExpander.digitalWrite(cfg.winchInPin, HIGH);
        outputExpander.digitalWrite(cfg.ezLoadBridgeAPin, cfg.wingOutputsOnLockBridges ? LOW : HIGH);
        outputExpander.digitalWrite(cfg.ezLoadBridgeBPin, LOW);
        outputExpander.digitalWrite(cfg.rotateBridgeAPin, cfg.wingOutputsOnLockBridges ? LOW : HIGH);
        outputExpander.digitalWrite(cfg.rotateBridgeBPin, LOW);
        ioExpanderSetAllPinModes(outputExpander, OUTPUT);

        // Turn KSI on as early as possible — the instant the output expander (which owns the KSI
        // pin) is up. This lets the Curtis controllers begin powering up and booting while the
        // rest of setup() runs (input expander, motor CAN/NMT init, XBee), instead of waiting for
        // the systemPowerOn() at the end. systemPowerOn() still runs later and re-asserts KSI +
        // the status LED. KSI (systemPwrPin) is active-high at the expander.
        BitMasker::setBit(accsCmnds, TUG_SYSTEM_PWR, HIGH);
        outputExpander.digitalWrite(cfg.systemPwrPin, HIGH);
        D1PRINTLN("KSI ON (early) — Curtis controllers powering up");
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

    eStopButton.init();
    if (cfg.pairButtonPin != 0xFF)
    {
        pairButton.init();
    }
    comm.setPairingEnabled(false); // tug won't accept pairing until the operator opens the window
    pinMode(cfg.boardPwrOffPin, OUTPUT);
    digitalWrite(cfg.boardPwrOffPin, LOW);
    pinMode(cfg.externalStatusLedPin, OUTPUT);
    digitalWrite(cfg.externalStatusLedPin, LOW);

    pinMode(cfg.tugBatPin, INPUT);
    // Plain INPUT: the PDB's on-board 10k/2.37k dividers define the level on the AIEX inputs
    // (an internal pull-up can't win against the 2.37k bottom leg), and the lock-switch reads
    // are analog-threshold (see readHardware).
    pinMode(cfg.rotationLockInputPin, INPUT);
    pinMode(cfg.cradleLockInputPin, INPUT);
    if (cfg.breakerSensePin != 0)
    {
        pinMode(cfg.breakerSensePin, INPUT);
    }
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
    // Diagnostic heartbeat on the comm CAN bus so an isolated PDB is scope-visible
    // (rate-limited to 1 Hz inside CANComm).
    comm.canComm.txHeartbeat((uint8_t)sysState, eStopButton.isPressed() ? 0x01 : 0x00);

    readTugBattery();
    handleComm();
    handlePairing();
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
        // Release the E-STOP only. The safety stop is deliberately NOT cleared here: it releases
        // itself inside the motor controller's setThrottle() once the stick is back within the
        // ±300 deadband, which is the whole point of it — hold the tug stopped until the operator
        // centres the stick. Clearing it here undid that on the very next loop, so a quick-reversal
        // safety stop only ever lasted one iteration and the ramp to neutral never finished.
        // (Both live implementations, RoboteQ and Curtis 1229, self-clear. Any new
        // IMotorController must do the same or its safety stop will latch.)
        if (motor->isEStopped())
        {
            D1PRINTLN("Releasing motor e-stop on entry to NORMAL");
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
        // Powered e-stop: command NEUTRAL and let the motor controller ramp to a stop at
        // its e-stop deceleration rate. Do NOT cut KSI here — leaving the controllers
        // powered gives a controlled powered stop instead of a coast. The tug's physical
        // e-stop button has its own independent hardware motor cutoff for firmware faults.
        motor->eStop();
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

    // Drive the motor controller every loop, NOT only when a packet lands. update() is what
    // actually transmits (the Curtis RPDO stream, the RoboteQ CANopen tick); it self-rate-limits
    // internally, so calling it unconditionally just makes the stream time-driven instead of
    // packet-driven.
    //
    // It used to live in setOutputs(), which only runs on a fresh XBee packet (handleComm) or in
    // COMM_ERR — so any gap in controller packets stopped CAN transmission outright and the Curtis
    // raised PDO Timeout. That is the common cause behind all three of Nathan's repros: switching
    // the remote between the settings and accessories pages (EVE redraw stalls the handheld's TX),
    // random dropouts while driving, and plugging/unplugging USB at the PDB. States that never call
    // setOutputs at all (ESTOP, IDLE, BOOT) were guaranteed to time out.
    //
    // Throttle/steering still only change when a packet arrives or a state forces neutral; between
    // packets this re-sends the last applied values, which is what the Curtis was holding anyway —
    // and a genuine comm loss still ramps to neutral via COMM_ERR.
    motor->update();
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

// OTA pairing (PDB side). The pair button opens a single-accept window during which the tug will
// honor a controller's pairing handshake (gated in WirelessComm::setStateGlobal). The existing
// PAIR_RESPONSE/PAIR_END exchange does the actual address swap; here we just manage the window,
// the single-accept behavior, and the status-LED feedback. See OTA_Radio_Pairing_Design.md.
void WirelessReceiver::handlePairing()
{
    if (cfg.pairButtonPin == 0xFF)
    {
        return; // no pairing button fitted on this build
    }

    // Rising-edge detect: arm (or re-arm) the window on a fresh press. A second press restarts the
    // 60 s window — stop() first because Timer::start() is a no-op while already running.
    bool pressedNow = pairButton.isPressed();
    if (pressedNow && !pairButtonWasPressed)
    {
        pairWindowTimer.stop();
        pairWindowTimer.start(PAIR_WINDOW_DURATION_MS);
        pairingWindowOpen = true;
        comm.setPairingEnabled(true);
        comm.setStatusCode(RADIO_PAIRING); // clear any stale PAIRING_SUCCESS from a prior pairing
        pairLedMode = PAIR_LED_WINDOW;
        // Defense-in-depth: force motion to neutral the instant the pairing window opens, so a
        // non-centered stick can't keep the tug moving during the handshake. safetyStop() latches
        // until the stick is back inside the deadband once pairing is done — the operator has to
        // centre it before drive resumes. (The COMM_ERR path also enforces neutral every cycle
        // while pairing, since status != COMM_NORMAL — this makes the intent explicit here.)
        throttle = NEUTRAL;
        steering = STRAIGHT;
        motor->safetyStop();
        D1PRINTLN("Pairing window OPEN (60 s) — waiting for controller");
    }
    pairButtonWasPressed = pressedNow;

    if (pairingWindowOpen)
    {
        if (PAIRING_SUCCESS == comm.getStatusCode())
        {
            // Single-accept: first successful pairing closes the window.
            pairingWindowOpen = false;
            comm.setPairingEnabled(false);
            pairWindowTimer.stop();
            pairLedMode = PAIR_LED_SUCCESS;
            pairResultHoldUntil = millis() + PAIR_LED_RESULT_HOLD_MS;
            D1PRINTLN("Pairing SUCCESS — window closed");
        }
        else if (pairWindowTimer.isFinished())
        {
            pairingWindowOpen = false;
            comm.setPairingEnabled(false);
            pairLedMode = PAIR_LED_TIMEOUT;
            pairResultHoldUntil = millis() + PAIR_LED_RESULT_HOLD_MS;
            D1PRINTLN("Pairing window TIMED OUT — no controller paired");
        }
    }

    updatePairingLed();
}

// Non-blocking status-LED driver for pairing. Owns externalStatusLedPin only while a window is open
// or a result indication is showing; otherwise leaves it to the normal power-state control.
void WirelessReceiver::updatePairingLed()
{
    unsigned long now = millis();
    switch (pairLedMode)
    {
    case PAIR_LED_WINDOW: // fast blink ~5 Hz while waiting
        if (now - pairLedToggleTime >= PAIR_LED_FAST_BLINK_MS)
        {
            pairLedToggleTime = now;
            pairLedState = !pairLedState;
            digitalWrite(cfg.externalStatusLedPin, pairLedState);
        }
        break;

    case PAIR_LED_SUCCESS: // solid ON, then hand back to normal
        digitalWrite(cfg.externalStatusLedPin, HIGH);
        if ((long)(now - pairResultHoldUntil) >= 0)
        {
            pairLedMode = PAIR_LED_NORMAL;
            digitalWrite(cfg.externalStatusLedPin, HIGH); // restore powered (LED on) state
        }
        break;

    case PAIR_LED_TIMEOUT: // slow blink, then hand back to normal
        if (now - pairLedToggleTime >= PAIR_LED_SLOW_BLINK_MS)
        {
            pairLedToggleTime = now;
            pairLedState = !pairLedState;
            digitalWrite(cfg.externalStatusLedPin, pairLedState);
        }
        if ((long)(now - pairResultHoldUntil) >= 0)
        {
            pairLedMode = PAIR_LED_NORMAL;
            digitalWrite(cfg.externalStatusLedPin, HIGH); // restore powered (LED on) state
        }
        break;

    case PAIR_LED_NORMAL:
    default:
        // Not pairing: leave the LED under normal power-state control (systemPowerOn/Off).
        break;
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

    // Lock state from the microswitches (Teensy analog pins, not the expander):
    // rotation line low = rotation unlocked; cradle line high = cradle unlocked.
    // Analog-threshold reads: the AIEX inputs run through the PDB's on-board 10k/2.37k
    // divider (12 V-level), which lands a 12 V switch signal ~2.3 V at the pin — right at the
    // digital VIH threshold — so digitalRead was marginal. 310 counts ~= 1.0 V at the pin.
    BitMasker::setBit(accsStatus, ROTATE_UNLOCK,  analogRead(cfg.rotationLockInputPin) <= AIEX_DIGITAL_ON_THRESHOLD);
    BitMasker::setBit(accsStatus, EZ_LOAD_UNLOCK, analogRead(cfg.cradleLockInputPin) > AIEX_DIGITAL_ON_THRESHOLD);

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

    // Main breaker monitor: the breaker interrupts ground for everything except the PDB; the
    // sense line sits on the protected ground (low) and gets pulled to +12V by a harness
    // resistor when the breaker opens (the AIEX divider makes an internal pull-up useless).
    // Qualified 250 ms so a transient can't flash the "Check Breaker" message.
    uint16_t flags = 0;
    if (eStopLatched)
    {
        flags |= TUG_FLAG_ESTOP_LATCHED;
    }
    if (cfg.breakerSensePin != 0)
    {
        static unsigned long breakerHighSinceMs = 0;

        // Raw counts every 5 s. This monitor shipped with NO logging at all, which is why three
        // bench rounds on Romeo could not separate "the PDB never sees the breaker open" from
        // "the PDB sees it but the handheld never shows it" — the serial capture simply had
        // nothing about the breaker in it either way. Low counts = breaker OK (line grounded),
        // above AIEX_DIGITAL_ON_THRESHOLD = open (harness pull-up to +12V winning).
        const int breakerRaw = analogRead(cfg.breakerSensePin);
        D1PERIODICPRINTVAR(5000, breakerRaw);

        if (breakerRaw > AIEX_DIGITAL_ON_THRESHOLD)
        {
            if (0 == breakerHighSinceMs)
            {
                breakerHighSinceMs = millis();
                if (0 == breakerHighSinceMs) breakerHighSinceMs = 1;
            }
        }
        else
        {
            breakerHighSinceMs = 0;
        }
        if ((0 != breakerHighSinceMs) && ((millis() - breakerHighSinceMs) >= 250))
        {
            flags |= TUG_FLAG_BREAKER_BLOWN;
        }

        // Edge-triggered so it is loud when it matters but silent otherwise (a per-loop print
        // here would stall the loop, which is what was causing the Curtis PDO timeouts).
        static bool prevBreakerBlown = false;
        const bool breakerBlownNow = (0 != (flags & TUG_FLAG_BREAKER_BLOWN));
        if (breakerBlownNow != prevBreakerBlown)
        {
            prevBreakerBlown = breakerBlownNow;
            D1PRINT("Main breaker ");
            D1PRINT(breakerBlownNow ? "OPEN" : "restored");
            D1PRINT(" (raw ");
            D1PRINT(breakerRaw);
            D1PRINTLN(") — sending TUG_FLAG_BREAKER_BLOWN to the controller");
        }
    }
    comm.activePacket.setTugFlags(flags);
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
    // motor->update() deliberately NOT called here — it runs once per loop at the end of
    // update() so the CAN stream never stops when packets stop arriving. See the note there.
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
    // Winch in/out are an ACTIVE-LOW (switched-ground) HB H-bridge pin pair: the pin idles HIGH
    // (device OFF) and is pulled LOW to activate. So write the INVERSE of the commanded state —
    // getIsActive() == false -> HIGH (off, the default), true -> LOW (on when the button is held).
    // (Verified on the LED test fixture: without the invert both winch outputs sat ON by default.)
    // Straightforward command->pin mapping: WINCH_IN -> winchInPin, WINCH_OUT -> winchOutPin.
    // Physical travel direction is set by the H-bridge wiring — if a tug's winch runs backwards,
    // fix it at the motor leads / harness, NOT with a firmware swap (this mapping is shared by all
    // wireless receivers and must match consistent wiring across tugs).
    outputExpander.digitalWrite(cfg.winchOutPin,       !BitMasker::getIsActive(accsCmnds, WINCH_OUT));
    outputExpander.digitalWrite(cfg.winchInPin,        !BitMasker::getIsActive(accsCmnds, WINCH_IN));

    bool ezLoadUnlock = BitMasker::getIsActive(accsCmnds, EZ_LOAD_UNLOCK);
    bool rotateUnlock = BitMasker::getIsActive(accsCmnds, ROTATE_UNLOCK);
    if (cfg.wingOutputsOnLockBridges)
    {
        // Papa/Helipad: the four lock H-bridge pins drive the WINGS. The HB output stage is
        // NON-inverting (bench-verified: expander HIGH reads HIGH at the connector), so idle =
        // expander LOW on all four -> connector pins 2/4/6/8 low, motors off, the required
        // power-up default. A wing command drives exactly its own pin HIGH while the button is
        // held; the opposite bridge pin stays LOW, which sets the motor's direction. (The first
        // cut idled HIGH/drove LOW on an inverted polarity model -- that both defaulted the
        // connector high AND reversed up/down, because the opposite pin was the one left
        // driving.) Rotate bridge = RIGHT wing (conn 6 up / 8 down); EZ-load bridge = LEFT wing
        // (conn 2 up / 4 down). UP and DOWN together for the same wing drives neither.
        bool lUp   = BitMasker::getIsActive(accsCmnds, L_WING_UP);
        bool lDown = BitMasker::getIsActive(accsCmnds, L_WING_DOWN);
        bool rUp   = BitMasker::getIsActive(accsCmnds, R_WING_UP);
        bool rDown = BitMasker::getIsActive(accsCmnds, R_WING_DOWN);
        if (lUp && lDown)
        {
            ERRORPRINTLN("L wing UP+DOWN commanded together - driving neither");
            lUp = false; lDown = false;
        }
        if (rUp && rDown)
        {
            ERRORPRINTLN("R wing UP+DOWN commanded together - driving neither");
            rUp = false; rDown = false;
        }
        outputExpander.digitalWrite(cfg.ezLoadBridgeAPin, lUp   ? HIGH : LOW);
        outputExpander.digitalWrite(cfg.ezLoadBridgeBPin, lDown ? HIGH : LOW);
        outputExpander.digitalWrite(cfg.rotateBridgeAPin, rUp   ? HIGH : LOW);
        outputExpander.digitalWrite(cfg.rotateBridgeBPin, rDown ? HIGH : LOW);

        // Troubleshooting print: logical wing state + what the connector pins should read, on
        // any change only (a per-loop print here would stall the loop, which is what trips the
        // Curtis PDO timeouts).
        static uint8_t prevWingState = 0xFF;
        uint8_t wingState = (uint8_t)((lUp ? 1 : 0) | (lDown ? 2 : 0) | (rUp ? 4 : 0) | (rDown ? 8 : 0));
        if (wingState != prevWingState)
        {
            prevWingState = wingState;
            D1PRINT("Wings: L=");
            D1PRINT(lUp ? "UP" : (lDown ? "DOWN" : "off"));
            D1PRINT(" R=");
            D1PRINT(rUp ? "UP" : (rDown ? "DOWN" : "off"));
            D1PRINT("  conn pins [2,4,6,8] = ");
            D1PRINT(lUp);   D1PRINT(",");
            D1PRINT(lDown); D1PRINT(",");
            D1PRINT(rUp);   D1PRINT(",");
            D1PRINTLN(rDown);
        }
    }
    else
    {
        // R04D Romeo: EZ-load and rotation locks are H-bridges driven as an opposed pair.
        // Rest/locked = bridgeA HIGH, bridgeB LOW. Active/unlock (Load selected) = bridgeA LOW, bridgeB HIGH.
        outputExpander.digitalWrite(cfg.ezLoadBridgeAPin, ezLoadUnlock ? LOW : HIGH);
        outputExpander.digitalWrite(cfg.ezLoadBridgeBPin, ezLoadUnlock ? HIGH : LOW);
        outputExpander.digitalWrite(cfg.rotateBridgeAPin, rotateUnlock ? LOW : HIGH);
        outputExpander.digitalWrite(cfg.rotateBridgeBPin, rotateUnlock ? HIGH : LOW);
        // Pin 24 (BDEX_16) drives the 16-inch lazy-susan unlock 24V solenoid directly (low-side,
        // since the H-bridge pin cannot), matching rotation bridge A (pin 6) so they track together.
        outputExpander.digitalWrite(cfg.dirIndRightLedPin, rotateUnlock ? LOW : HIGH);
    }

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
        // Held in ESTOP past the window -> latch until power cycle (see ESTOP_LATCH_AFTER_MS).
        // The minute counts from ESTOP entry; a tug that boots into ESTOP gets a fresh minute
        // because eStopEnteredTime is stamped on the post-boot entry.
        if (!eStopLatched && (millis() - eStopEnteredTime >= ESTOP_LATCH_AFTER_MS))
        {
            eStopLatched = true;
            D1PRINTLN("ESTOP held > 1 min — LATCHED until tug power cycle");
        }
        return;
    }

    if (ESTOP == sysState)
    {
        if (eStopLatched)
        {
            // Latched: the normal recovery below (controller acknowledged and released) is
            // deliberately unreachable. Stay in ESTOP; only a tug power cycle clears this.
            // The pwrOffTimer keeps running, so the 30-minute idle power-off still fires
            // from a latched ESTOP — same behaviour Bravo already has.
            D1PERIODICPRINTLN(400, "ESTOP latched — cycle tug power to clear");
            return;
        }
        // We're in ESTOP but neither the local button nor the controller is
        // in ESTOP anymore — recover. Controller e-stop release is the
        // acknowledgment that clears the system.
        // KSI is left ON during ESTOP now (powered stop via the motor controller's
        // e-stop deceleration), so the controllers were never power-cycled and there
        // is no reset delay to wait out — recover immediately.
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

    // Edge-triggered: says what the PDB is actually putting in the packet. Without this the only
    // way to tell "the motor controller never reported a fault" from "it reported one and the
    // handheld did not show it" was to guess. Prints on any change, including back to zero.
    static uint16_t prevErr = 0;
    static uint16_t prevFlags = 0;
    if (motorErrorCode != prevErr || motorStatusFlags != prevFlags)
    {
        prevErr = motorErrorCode;
        prevFlags = motorStatusFlags;
        D1PRINT("Motor diagnostics -> packet: errorCode 0x");
        D1PRINT(motorErrorCode, HEX);
        D1PRINT(", statusFlags 0x");
        D1PRINTLN(motorStatusFlags, HEX);
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
