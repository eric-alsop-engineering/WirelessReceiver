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

    // Detect optional capabilities via dynamic_cast
    this->driveModes = dynamic_cast<IDriveModeController*>(&motor);
    this->diagnostics = dynamic_cast<IDiagnosticSource*>(&motor);

    systemReady = false;
    throttle = NEUTRAL;
    steering = STRAIGHT;
    torque = 0xACED;       // TODO: not actually measured yet
    tugBatLvl = 0xBAFF;   // TODO: not actually measured yet
    accsCmnds = TugAccessories();
    accsStatus = TugAccessories();
    sysState = BOOT;
    ctrlrState = BOOT;
    motorErrorCode = 0;
    motorStatusFlags = 0;
    pwrOffTimer = Timer(IDLE_TIMER_DURATION, false);
    pwrOffConfirmedTimer = Timer(PWR_OFF_CONFIRMED_TIMER_DURATION, false);
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

    accsCmnds.setAll(0);
    eStopButton.init();
    pinMode(cfg.boardPwrOffPin, OUTPUT);
    digitalWrite(cfg.boardPwrOffPin, LOW);
    pinMode(cfg.externalStatusLedPin, OUTPUT);
    digitalWrite(cfg.externalStatusLedPin, LOW);

    motor->init();
    writeHardware();
    comm.setup();
    systemPowerOn();
    D1PRINTLN("WirelessReceiver setup complete");
}

void WirelessReceiver::update()
{
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
        accsCmnds.unlock();
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

    accsStatus.set(TUG_SYSTEM_PWR, inputExpander.digitalRead(cfg.systemPwrPin));
    accsStatus.set(HEADLIGHTS, inputExpander.digitalRead(cfg.headlightsPin));
    accsStatus.set(AIR_COMPRESSOR, inputExpander.digitalRead(cfg.airCompressorPin));
    accsStatus.set(ROTATE_UNLOCK, inputExpander.digitalRead(cfg.rotateRelayPin));
    accsStatus.set(EZ_LOAD_UNLOCK, inputExpander.digitalRead(cfg.ezLoadRelayPin));
    accsStatus.set(UNDER_GLOW, inputExpander.digitalRead(cfg.underGlowPin));
    accsStatus.set(FORWARD_LIGHT, inputExpander.digitalRead(cfg.dirIndFwdLedPin));
    accsStatus.set(BACKWARD_LIGHT, inputExpander.digitalRead(cfg.dirIndRvrsLedPin));
    accsStatus.set(LEFT_TURN_LIGHT, inputExpander.digitalRead(cfg.dirIndLeftLedPin));
    accsStatus.set(RIGHT_TURN_LIGHT, inputExpander.digitalRead(cfg.dirIndRightLedPin));
    accsStatus.set(WINCH_OUT, inputExpander.digitalRead(cfg.winchOutPin));
    accsStatus.set(WINCH_IN, inputExpander.digitalRead(cfg.winchInPin));
    accsStatus.set(L_WING_UP, inputExpander.digitalRead(cfg.lWingUpPin));
    accsStatus.set(L_WING_DOWN, inputExpander.digitalRead(cfg.lWingDownPin));
    accsStatus.set(R_WING_UP, inputExpander.digitalRead(cfg.rWingUpPin));
    accsStatus.set(R_WING_DOWN, inputExpander.digitalRead(cfg.rWingDownPin));

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
    bool tugSysPwrIsOn = accsCmnds.getIsActive(TUG_SYSTEM_PWR);
    accsCmnds.setAll(comm.receivedPacket.accsData.getAll());
    accsCmnds.set(TUG_SYSTEM_PWR, tugSysPwrIsOn);

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
        accsCmnds.set(RIGHT_TURN_LIGHT, HIGH);
        accsCmnds.set(LEFT_TURN_LIGHT, LOW);
    }
    else if (steering < 0)
    {
        accsCmnds.set(LEFT_TURN_LIGHT, HIGH);
        accsCmnds.set(RIGHT_TURN_LIGHT, LOW);
    }
    else if (0 == steering || motor->isSafetyStopped())
    {
        accsCmnds.set(LEFT_TURN_LIGHT, LOW);
        accsCmnds.set(RIGHT_TURN_LIGHT, LOW);
    }

    if (throttle > 0)
    {
        accsCmnds.set(FORWARD_LIGHT, HIGH);
        accsCmnds.set(BACKWARD_LIGHT, LOW);
    }
    else if (throttle < 0)
    {
        accsCmnds.set(BACKWARD_LIGHT, HIGH);
        accsCmnds.set(FORWARD_LIGHT, LOW);
    }
    else if (0 == throttle || motor->isSafetyStopped())
    {
        accsCmnds.set(BACKWARD_LIGHT, LOW);
        accsCmnds.set(FORWARD_LIGHT, LOW);
    }
}

void WirelessReceiver::writeHardware()
{
    D2PRINTLN("Entering writeHardware");

    outputExpander.digitalWrite(cfg.systemPwrPin, accsCmnds.getIsActive(TUG_SYSTEM_PWR));
    outputExpander.digitalWrite(cfg.headlightsPin, accsCmnds.getIsActive(HEADLIGHTS));
    outputExpander.digitalWrite(cfg.airCompressorPin, accsCmnds.getIsActive(AIR_COMPRESSOR));
    outputExpander.digitalWrite(cfg.rotateRelayPin, accsCmnds.getIsActive(ROTATE_UNLOCK));
    outputExpander.digitalWrite(cfg.ezLoadRelayPin, accsCmnds.getIsActive(EZ_LOAD_UNLOCK));
    outputExpander.digitalWrite(cfg.underGlowPin, accsCmnds.getIsActive(UNDER_GLOW));
    outputExpander.digitalWrite(cfg.dirIndFwdLedPin, accsCmnds.getIsActive(FORWARD_LIGHT));
    outputExpander.digitalWrite(cfg.dirIndRvrsLedPin, accsCmnds.getIsActive(BACKWARD_LIGHT));
    outputExpander.digitalWrite(cfg.dirIndLeftLedPin, accsCmnds.getIsActive(LEFT_TURN_LIGHT));
    outputExpander.digitalWrite(cfg.dirIndRightLedPin, accsCmnds.getIsActive(RIGHT_TURN_LIGHT));
    outputExpander.digitalWrite(cfg.winchOutPin, accsCmnds.getIsActive(WINCH_OUT));
    outputExpander.digitalWrite(cfg.winchInPin, accsCmnds.getIsActive(WINCH_IN));
    outputExpander.digitalWrite(cfg.lWingUpPin, accsCmnds.getIsActive(L_WING_UP));
    outputExpander.digitalWrite(cfg.lWingDownPin, accsCmnds.getIsActive(L_WING_DOWN));
    outputExpander.digitalWrite(cfg.rWingUpPin, accsCmnds.getIsActive(R_WING_UP));
    outputExpander.digitalWrite(cfg.rWingDownPin, accsCmnds.getIsActive(R_WING_DOWN));

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
        D1PRINTLN("E-Stop button pressed!");
        sysState = ESTOP;
        return;
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
        if (ESTOP != sysState)
        {
            D1PRINTLN("Ctrlr is ESTOP. Setting sysState to ESTOP");
            sysState = ESTOP;
        }
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
    accsCmnds.set(TUG_SYSTEM_PWR, LOW);
    outputExpander.digitalWrite(cfg.systemPwrPin, LOW);
    digitalWrite(cfg.externalStatusLedPin, LOW);
}

void WirelessReceiver::systemPowerOn()
{
    D1PRINTLN("Turning ON power now");
    accsCmnds.set(TUG_SYSTEM_PWR, HIGH);
    outputExpander.digitalWrite(cfg.systemPwrPin, HIGH);
    digitalWrite(cfg.externalStatusLedPin, HIGH);
}

void WirelessReceiver::boardPowerOff()
{
    D1PRINTLN("Shutting off power now");
    digitalWrite(cfg.boardPwrOffPin, HIGH);
}
