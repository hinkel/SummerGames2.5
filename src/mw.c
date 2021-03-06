#include "board.h"
#include "mw.h"

// May 2013    Crashpilot 1000
// Based on the Timecop / Mwii Port
flags_t  f;
int16_t  debug[4];
uint8_t  toggleBeep    = 0;
uint32_t currentTime   = 0;
uint32_t currentTimeMS = 0;
uint32_t previousTime  = 0;
uint16_t cycleTime     = 0;                                          // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint8_t  vbat;                                                       // battery voltage in 0.1V steps
int16_t  telemTemperature1;                                          // gyro sensor temperature
uint16_t failsafeCnt   = 0;
float    TiltValue;                                                  // 1.0 is horizontal 0 is vertical, minus is upsidedown
int16_t  rcData[MAX_RC_CHANNELS];                                    // int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t  rcDataSAVE [MAX_RC_CHANNELS];
uint8_t  rssi;                                                       // 0 - 255 = 0%-100%
int16_t  rcCommand[4];                                               // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t  lookupPitchRollRC[6];                                       // lookup table for expo & RC rate PITCH+ROLL
int16_t  lookupThrottleRC[11];                                       // lookup table for expo & mid THROTTLE
rcReadRawDataPtr rcReadRawFunc = NULL;                               // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)
float    dynP8[3], dynD8[3];  //dynI8[3],
uint8_t  rcOptions[CHECKBOXITEMS];
int16_t  axisPID[3];
float    newpidimax;

// **********************
// MAG
// **********************
float    headFreeModeHold;
float    heading;
float    magHold;
float    magneticDeclination;

// **********************
// SONAR /BARO /AUTOLAND
// **********************
uint8_t  SonarStatus = 0;                                            // 0 = no contact, 1 = made contact, 2 = steady contact
uint8_t  SonarBreach = 0;                                            // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
uint16_t LandDetectMinThr = 0;                                       // Is set upon Baro initialization in sensors/sensorsAutodetect
float    pressure;

// **********************
// GPS
// **********************
int32_t  GPS_coord[2];                                               // They contain some ins as well
int32_t  Real_GPS_coord[2];                                          // RAW GPS Coords
int32_t  GPS_home[2];
int32_t  GPS_WP[2];                                                  // Currently used WP
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                                         // distance to home point in meters
int16_t  GPS_directionToHome;                                        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;                                    // altitude in m and speed in 0.1m/s
uint8_t  GPS_update = 0;                                             // it's a binary toogle to distinct a GPS position update
float    GPS_angle[2] = { 0, 0 };                                    // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;                                      // degrees * 10
uint8_t  GPS_Present = 0;                                            // Checksum from Gps serial
uint8_t  GPS_Enable = 0;
float    nav[2];
float    nav_rated[2];                                               // Adding a rate controller to the navigation to make it smoother
int8_t   nav_mode = NAV_MODE_NONE;                                   // Navigation mode
int8_t   wp_status = WP_STATUS_NONE;                                 // Waypoint status
int8_t   ph_status;
int32_t  WP_Target_Alt;
int16_t  WP_Desired_Climbrate;                                       // Climbrate in cm/s
bool     WP_Fastcorner;                                              // Dont decrease Speed at Target
float    sin_yaw_y;
float    cos_yaw_x;

// *************************
// Automatic ACC Calibration
// *************************
uint16_t InflightcalibratingA = 0;
int16_t  AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// **********************
// Battery monitoring
// **********************
uint8_t  batteryCellCount = 3;                                       // cell count
uint16_t batteryWarningVoltage;                                      // annoying buzzer after this one, battery ready to be dead

// **********************
// MWCRGB
// **********************
uint32_t LED_Value = 1500;
uint8_t  LED_Value_Delay = 8;

// **********************
// EEPROM
// **********************
uint32_t ScheduleEEPROMwriteMS = 0;

// Crashpilot LED Inverter stuff
void LD0_OFF(void)
{
    if (cfg.LED_invert == 0) LED0_OFF;
    if (cfg.LED_invert == 1) LED0_ON;
}
void LD0_ON(void)
{
    if (cfg.LED_invert == 0) LED0_ON;
    if (cfg.LED_invert == 1) LED0_OFF;
}
void LD1_OFF(void)
{
    if (cfg.LED_invert == 0) LED1_OFF;
    if (cfg.LED_invert == 1) LED1_ON;
}
void LD1_ON(void)
{
    if (cfg.LED_invert == 0) LED1_ON;
    if (cfg.LED_invert == 1) LED1_OFF;
}

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++)
    {
        for (i = 0; i < num; i++)
        {
            LED0_TOGGLE;                                             // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead)                  // Actually needed for additional GPS deadband
{
    if (abs(rcvalue) < rcdead) rcvalue = 0;
    else if (rcvalue > 0) rcvalue = rcvalue - (int16_t)rcdead;
    else rcvalue = rcvalue + (int16_t)rcdead;
    return rcvalue;
}

#define BREAKPOINT 1500

// this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
void annexCode(void)
{
    static uint32_t calibratedAccTime;
    static uint8_t  buzzerFreq, vbatTimer = 0, ind = 0;
    static uint16_t vbatRawArray[8];  
    uint16_t        tmp, tmp2,vbatRaw = 0;
    uint8_t         axis, prop1, prop2, i;
    float           cosDiff, sinDiff, radDiff;
    int16_t         rcCommand_PITCH;    

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < BREAKPOINT)
    {
        prop2 = 100;
    }
    else
    {
        if (rcData[THROTTLE] < 2000)
        {
            prop2 = 100 - (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
        }
        else
        {
            prop2 = 100 - cfg.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++)
    {
        tmp = min(abs(rcData[axis] - cfg.midrc), 500);
        if (axis != 2)                                               // ROLL & PITCH
        {
            if (cfg.deadband)
            {
                if (tmp > cfg.deadband) tmp -= cfg.deadband;
                else tmp = 0;
            }
            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
            prop1 = (uint16_t) prop1 *prop2 / 100;
        }
        else                                                         // YAW
        {
            if (cfg.yawdeadband)
            {
                if (tmp > cfg.yawdeadband) tmp -= cfg.yawdeadband;
                else tmp = 0;
            }
            rcCommand[axis] = tmp;
            prop1 = 100 - (uint16_t)cfg.yawRate * tmp / 500;
        }
        dynP8[axis] = (uint16_t)cfg.P8[axis] * prop1 / 100;          // dynI8[axis] = (uint16_t) cfg.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < cfg.midrc) rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], cfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck);// [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if(f.HEADFREE_MODE)
    {
        radDiff = heading - headFreeModeHold;                        // let the user adjust headfree phase
        if (radDiff > 180.0f)       radDiff = radDiff - 360.0f;      // Wrap to -180 0 +180 Degree
        else if (radDiff < -180.0f) radDiff = radDiff + 360.0f;
        radDiff = radDiff * RADX;                                    // Degree to RAD
        cosDiff = cosf(radDiff);
        sinDiff = sinf(radDiff);
        rcCommand_PITCH  = (float)rcCommand[PITCH] * cosDiff + (float)rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL]  = (float)rcCommand[ROLL] * cosDiff - (float)rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
    
    if (feature(FEATURE_VBAT))
    {
        if (!(++vbatTimer % VBATFREQ))
        {
            vbatRawArray[(ind++) % 8] = adcGetChannel(ADC_BATTERY);
            for (i = 0; i < 8; i++) vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) buzzerFreq = 0;  // VBAT ok, buzzer off
        else buzzerFreq = 4;                                         // low battery
    }

    buzzer(buzzerFreq);                                              // external buzzer routine that handles buzzer events globally now

    if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0))
    {
        LED0_TOGGLE;                                                 // Calibration phasis
    }
    else
    {
        if (f.ACC_CALIBRATED) LD0_OFF();                             // Crashpilot LED0_OFF;
        if (f.ARMED) LD0_ON();                                       // Crashpilot LED0_ON;
      
        switch(cfg.tele_prot)                                        // 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)
        {
        case 1:
            initFRSKYTelemetry(f.ARMED);                             // This will switch to/from 9600 or 115200 baud depending on state.
            break;
        case 3:                                                      // 3 = Mavlink @57KBaud
            initMinimOSDTelemetry(f.ARMED);
        default:
            break;
        }
    }

#ifdef LEDRING
    if (feature(FEATURE_LED) && (cfg.LED_Type == 3))
    {
        static uint32_t LEDTime;
        if (currentTimeMS >= LEDTime)
        {
            LEDTime = currentTimeMS + 50;
            ledringState();
        }
    }
#endif
    if (feature(FEATURE_LED))
    {
        if (cfg.LED_Type == 1)  	                                   // MWCRGB
        {
            if (failsafeCnt > 5) LED_Value = 2100;
            else if (buzzerFreq > 3) LED_Value = 2200;               // Changed http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=1660#p33083
            else
            {
                if ((cfg.LED_Armed == 1) || f.ARMED)
                {
                    if (rcData[cfg.LED_ControlChannel - 1] < 1300) LED_Value = cfg.LED_Pattern1;
                    else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700)) LED_Value = cfg.LED_Pattern2;
                    else LED_Value = cfg.LED_Pattern3;
                }
                else LED_Value = 1000;
            }
        }
        if (cfg.LED_Type == 2)                                       // MONO_LED
        {
           if (failsafeCnt > 5)                                      // Update the leds every frame, must also be called if leds are disabled now
           {
              LED_Value = 2100;
              LED_Value_Delay = 10;
           }
           else if (buzzerFreq > 3)
           {
              LED_Value = 2200;
              LED_Value_Delay = 10;
           }
           else
           {
              if ((cfg.LED_Armed == 1) || f.ARMED)
              {
                 if (rcData[cfg.LED_ControlChannel - 1] < 1300)
                 {
                    LED_Value = cfg.LED_Pattern1;
                    LED_Value_Delay=cfg.LED_Toggle_Delay1;
                 }
                 else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700))
                 {
                    LED_Value = cfg.LED_Pattern2;
                    LED_Value_Delay=cfg.LED_Toggle_Delay2;
                 }
                 else
                 {
                    LED_Value = cfg.LED_Pattern3;
                    LED_Value_Delay=cfg.LED_Toggle_Delay3;
                 }
              }
              else
              {
                 LED_Value = 1000;
                 LED_Value_Delay=10;
              }
           }
           ledToggleUpdate(true);
        }
        else ledToggleUpdate(false);        
    }

    if (currentTimeMS >= calibratedAccTime)
    {
        if (!f.SMALL_ANGLES_25)
        {
            f.ACC_CALIBRATED = 0;                                    // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTimeMS + 500;
        }
        else f.ACC_CALIBRATED = 1;
    }

//    serialCom(); Move the serialstuff outside the timecritical IMU loop. Gui might slightly stutter, who cares?

    if (sensors(SENSOR_GPS))                                         // Crashpilot Show number of Sats by Blinking
    {
        static uint32_t Timebase;
        static uint8_t  cnt,blinkcase,blinkblock;

        if (GPS_numSat < 5 || !f.GPS_FIX) blinkcase = 0;
        if (f.GPS_FIX && GPS_numSat >= 5 && blinkcase == 0)
        {
            blinkcase = 1;
            blinkblock = GPS_numSat-4;                               // Change suggested by Hinkel So 5 Sats will blink one time
            Timebase = 0;
        }
        if (currentTimeMS >= Timebase)
        {
            Timebase = currentTimeMS + 10;                           // Timebase 10 ms
            if (blinkcase > 0) LD1_OFF();                            // LED1_OFF;
            switch(blinkcase)
            {
            case 0:
                break;
            case 1:
                cnt = 30;                                            // 300 ms on
                blinkcase++;
                break;
            case 2:
                LED1_TOGGLE;
                cnt--;
                if (cnt == 0) blinkcase++;
                break;
            case 3:
                cnt = 70;                                            // 700 ms off
                blinkcase++;
                break;
            case 4:
                cnt--;
                if (cnt == 0) blinkcase++;
                break;
            case 5:
                blinkblock--;
                if (blinkblock == 0) blinkcase++;
                else blinkcase = 1;
                break;
            case 6:
                cnt = 200;                                           // 2.0s Break before next flickerblock
                blinkcase++;
                break;
            case 7:
                cnt--;
                if (cnt == 0) blinkcase=0;
                break;
            }
        }
    }

    if (gyro.temperature && !MpuSpecial) gyro.temperature(&telemTemperature1);// Read out gyro temperature.
    else
    {
                                                                     // TODO MCU temp
    }
}                                                                    // END OF annexCode

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;
    if (chan > 7) data = pwmRead(chan);
    else data = pwmRead(cfg.rcmap[chan]);
    if (data < 750 || data > 2250) data = cfg.midrc;
    return data;
}

void computeRC(void)                                                 // Just harvest RC Data
{
    static int16_t rcData4Values[MAX_RC_CHANNELS][4], rcDataMean[MAX_RC_CHANNELS];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < cfg.auxChannels + 4; chan++)
    {
        rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(chan);
        rcDataMean[chan] = 0;
        for (a = 0; a < 4; a++) rcDataMean[chan] += rcData4Values[chan][a];
        rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
        if (rcDataMean[chan] < rcDataSAVE[chan] - 3) rcDataSAVE[chan] = rcDataMean[chan] + 2;
        if (rcDataMean[chan] > rcDataSAVE[chan] + 3) rcDataSAVE[chan] = rcDataMean[chan] - 2;
    }
}

void GetActualRCdataOutRCDataSave(void)                              // Make RC Data available
{
    uint8_t i;
    for (i = 0; i < MAX_RC_CHANNELS; i++) rcData[i] = rcDataSAVE[i];
}

bool DeadPilot(void)
{
    static int16_t  lastcheksum = 0;
    static uint32_t deadtimer = 0;
    bool            output;
    int16_t         cheksum;
    uint8_t         i;

    if (!f.ARMED || cfg.failsafe_deadpilot == 0)
    {
        deadtimer = 0;
        return false;
    }
    cheksum = 0;
    for (i = 0; i < 4; i++) cheksum = cheksum + rcData[i];
    if (cheksum != lastcheksum)
    {
        deadtimer = 0;
        output = false;
    }
    else if(deadtimer == 0) deadtimer = currentTimeMS + (uint32_t)cfg.failsafe_deadpilot * 1000;
    lastcheksum = cheksum;
    if (currentTimeMS > deadtimer && deadtimer != 0) output = true;
    return output;
}

void DisArmCopter(void)
{
    f.ARMED     = 0;
    f.OK_TO_ARM = 0;
}

void pass(void)                                                      // Crashpilot Feature pass
{
    static uint32_t blinktime, rctimer;
    static uint8_t  lastpassmotor;
    uint32_t timetmp;
    computeIMU();
    timetmp = micros();
    if (spektrumFrameComplete()) computeRC();                        // Generates no rcData yet, but rcDataSAVE
    if ((int32_t)(timetmp - rctimer) >= 0)                           // 50Hz
    {
        rctimer = timetmp + 20000;
        if (!feature(FEATURE_SPEKTRUM)) computeRC();
        GetActualRCdataOutRCDataSave();                              // Now we have new rcData to deal and MESS with
        if (failsafeCnt > 2)
        {
            rcData[THROTTLE] = cfg.mincommand;                       // Motor off
            writeAllMotors(rcData[THROTTLE]);                        // Set all motors to zero just to be sure if user is messing in cli without saving
        }
        if (lastpassmotor != cfg.passmotor) writeAllMotors(cfg.mincommand); // Motonumber was changed in cli without saving
        lastpassmotor = cfg.passmotor;
        if (cfg.passmotor == 0) writeAllMotors(rcData[THROTTLE]);    // All Motors?
        else pwmWriteMotor(cfg.passmotor-1, rcData[THROTTLE]);       // Specific Motor?
        failsafeCnt++;
    }
    f.ARMED = 0;                                                     // Always set this as a dummy so serial com accepts "#" and "R"
    serialCom();
    if ((int32_t)(timetmp - blinktime) >= 0)
    {
        blinktime = timetmp + 150000;
        LED1_TOGGLE;
        LED0_TOGGLE;
    }
}

void loop(void)
{
    static uint8_t  rcDelayCommand;                                  // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t  GoodRCcnt;                                       // Number of good consecutive Signals before arming
    static uint32_t Failsafetimer, Killtimer, RTLGeneralTimer, AltRCTimer0, LastLoopTime, rcTime = 0, AutolandGeneralTimer;
    static uint32_t GPSlogTimer = 0, LastSerialTimeMS;
    float           error, errorAngle, AngleRateTmp, RateError, delta, deltaSum;
    float           PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t  lastGyro[3] = { 0, 0, 0 };
    static float    delta1[3], delta2[3];
    static float    errorGyroI[3] = { 0, 0, 0 }, errorAngleI[2] = { 0, 0 };
    static float    lastError[3]  = { 0, 0, 0 }, lastDTerm[3]   = { 0, 0, 0 }; // pt1 element http://www.multiwii.com/forum/viewtopic.php?f=23&t=2624;
    static int16_t  initialThrottleHold;
    float           prop;
    static uint8_t  ThrFstTimeCenter, AutolandState, HoverThrcnt, RTLstate;
    static int8_t   Althightchange;
    static int16_t  LastAltThrottle;
    static uint16_t HoverThrottle;
    static int16_t  BaroLandThrlimiter, SnrLandThrlimiter;
    static int16_t  DistanceToHomeMetersOnRTLstart;
    static uint8_t  PHminSat;
    float           CosYawxPhase, SinYawyPhase, TmpPhase, tmp0flt, dT, MwiiTimescale;
    int16_t         tmp0, thrdiff;
    uint32_t        auxState = 0, auxStateTMP;
    uint8_t         axis, i;    

    // this will return false if spektrum is disabled. shrug.
    if (spektrumFrameComplete()) computeRC();                        // Generates no rcData yet, but rcDataSAVE
    if ((currentTime - rcTime) >= 20000)                             // 50Hz
    {
        rcTime = currentTime;

        if (!feature(FEATURE_SPEKTRUM)) computeRC();
        GetActualRCdataOutRCDataSave();                              // Now we have new rcData to deal and MESS with

        if ((rcData[THROTTLE] < cfg.mincheck) && AutolandState == 0) // Crashpilot
        {
            errorGyroI[ROLL]   = 0;
            errorGyroI[PITCH]  = 0;
            errorGyroI[YAW]    = 0;
            errorAngleI[ROLL]  = 0;
            errorAngleI[PITCH] = 0;
            rcDelayCommand++;
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck && !f.ARMED)
            {
                if (rcDelayCommand == 20)
                {
                    calibratingG = GyroCalibSamples;
                    if (feature(FEATURE_GPS)) GPS_reset_home_position();
                }
            }
            else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (!f.ARMED && rcData[YAW] < cfg.mincheck && rcData[PITCH] > cfg.maxcheck && rcData[ROLL] > cfg.maxcheck))
            {
                if (rcDelayCommand == 20)
                {
                    if (AccInflightCalibrationMeasurementDone)       // trigger saving into eeprom after landing
                    {
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    }
                    else
                    {
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) toggleBeep = 2;
                        else toggleBeep = 3;
                    }
                }
            }
            else if (cfg.activate[BOXARM] > 0)
            {
                if (rcOptions[BOXARM] && f.OK_TO_ARM)
                {
                    if (feature(FEATURE_FAILSAFE))                   // Crashpilot1000
                    {
                        if (GoodRCcnt > (5 * cfg.failsafe_delay)) f.ARMED = 1;
                        else  f.ARMED = 0;
                    }
                    else f.ARMED = 1;
                    headFreeModeHold = heading;
                }
                else if (f.ARMED) f.ARMED = 0;
                rcDelayCommand = 0;
            }
            else if ((rcData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && rcData[ROLL] < cfg.mincheck)) && f.ARMED)
            {
                if (rcDelayCommand == 20)
                    f.ARMED = 0;                                     // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            }
            else if ((rcData[YAW] > cfg.maxcheck || (rcData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1)) && rcData[PITCH] < cfg.maxcheck && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED)
            {
                if (rcDelayCommand == 20)
                {
                    if (feature(FEATURE_FAILSAFE))                   // Crashpilot1000
                    {
                        if (GoodRCcnt > (5 * cfg.failsafe_delay)) f.ARMED = 1;
                        else  f.ARMED = 0;
                    }
                    else f.ARMED = 1;
                    headFreeModeHold = heading;
                }
            }
            else
                rcDelayCommand = 0;
        }
        else if (rcData[THROTTLE] > cfg.maxcheck && !f.ARMED)
        {
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck) // throttle=max, yaw=left, pitch=min
            {
                if (rcDelayCommand == 20)
                    calibratingA = AccCalibSamples;
                rcDelayCommand++;
            }
            else if (rcData[YAW] > cfg.maxcheck && rcData[PITCH] < cfg.mincheck) // throttle=max, yaw=right, pitch=min
            {
                if (rcDelayCommand == 20)
                    f.CALIBRATE_MAG = 1;                             // MAG calibration request
                rcDelayCommand++;
            }
            else if (rcData[PITCH] > cfg.maxcheck)
            {
                cfg.angleTrim[PITCH] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[PITCH] < cfg.mincheck)
            {
                cfg.angleTrim[PITCH] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[ROLL] > cfg.maxcheck)
            {
                cfg.angleTrim[ROLL] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[ROLL] < cfg.mincheck)
            {
                cfg.angleTrim[ROLL] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else rcDelayCommand = 0;
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL))
        {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > cfg.mincheck && !rcOptions[BOXARM])     // Copter is airborne and you are turning it off via boxarm : start measurement
            {
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXPASSTHRU])                              // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
            {
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
                    InflightcalibratingA = 50;
            }
            else if (AccInflightCalibrationMeasurementDone && !f.ARMED)
            {
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }
        
        for (i = 0; i < cfg.auxChannels; i++)                        // for (i = 0; i < 4; i++)
        {
            auxStateTMP = rcData[AUX1 + i];
            auxState   |= (auxStateTMP < 1300) << (3 * i) |
                          (1300 < auxStateTMP && auxStateTMP < 1700) << (3 * i + 1) |
                          (auxStateTMP > 1700) << (3 * i + 2);
        }
        for (i = 0; i < CHECKBOXITEMS; i++) rcOptions[i] = (auxState & cfg.activate[i]) > 0;

        if (feature(FEATURE_LCD) && !f.ARMED)
        {
            if (rcData[THROTTLE] < cfg.mincheck && rcData[YAW] > cfg.maxcheck && rcData[PITCH] > cfg.maxcheck)
                serialOSD();
            else
                if (OLED_Type > 0) OLED_Status();
        }

        if ((rcOptions[BOXARM]) == 0) f.OK_TO_ARM = 1;               // Moved it here

        if (sensors(SENSOR_BARO) && !GroundAltInitialized) DisArmCopter(); // Keep Copter disarmed until baro init is done
          
/////// GPS INS TESTCODE
//				int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
//				cfg.gps_ins_vel = (500 + (float)knob * 0.5f)/1000.0f;
//				debug[0] = cfg.gps_ins_vel*1000;
/////// GPS INS TESTCODE
/////// GPS INS TESTCODE
//				int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
//				cfg.gps_ins_pos = (200 + (float)knob * 0.8f)/1000.0f;
//				debug[0] = cfg.gps_ins_pos*1000;
/////// GPS INS TESTCODE

        /////////////
        PHminSat = cfg.gps_ph_minsat;                                // Don't forget to set PH Minsats here!!
        /////////////
        // Failsafe routine Modified & Moved here by Crashpilot
        // 4 Cases:
        // 1: No Sensors -> Do the old stuff
        // 2: GPS -> Do RTL and do the old stuff
        // 3: Baro -> Do Baro Autoland
        // 4: BARO & GPS -> Do full feature RTL
        if (feature(FEATURE_FAILSAFE) && f.ARMED)                    // Only check Failsafe if copter is armed
        {
            // Logic Do FS if FScount is too high, or pilotdead (no stickinput for x seconds)
            // If FS is already running (fstimer !=0) but the goodcount is too low (maybe single good spike) keep FS running
            if ((failsafeCnt > (5 * cfg.failsafe_delay)) || DeadPilot() || (Failsafetimer !=0 && GoodRCcnt < (5 * cfg.failsafe_delay)))
            {
                if (Failsafetimer == 0) Failsafetimer = currentTimeMS + 100 * (uint32_t)(cfg.failsafe_delay + cfg.failsafe_off_delay);

                rcData[ROLL]           = cfg.midrc;                  // Center Sticks
                rcData[PITCH]          = cfg.midrc;                  // Center Sticks
                rcData[YAW]            = cfg.midrc;                  // Center Sticks
                rcCommand[ROLL]        = 0;                          // Center Sticks - seems obsolete, just to be sure
                rcCommand[PITCH]       = 0;                          // Center Sticks - seems obsolete, just to be sure
                rcCommand[YAW]         = 0;                          // Center Sticks - seems obsolete, just to be sure
                rcOptions[BOXHORIZON]  = 0;
                rcOptions[BOXANGLE]    = 1;
                rcOptions[BOXPASSTHRU] = 0;                          // Passthru off
                PHminSat               = 5;                          // Sloppy PH is sufficient
                rcOptions[BOXMAG]      = 1;                          // MAG ON
                rcOptions[BOXHEADFREE] = 0;                          // HeadFree off
                rcOptions[BOXBARO]     = 1;                          // Baro On
                if (cfg.failsafe_ignoreSNR == 1) cfg.snr_land = 0;   // This can disable the aided sonarlanding, that could cause problems on trees
                if (TiltValue < 0) DisArmCopter();                   // Copter is upside down in anglemode -> disarm the poor bustard

                if (!sensors(SENSOR_GPS) && !sensors(SENSOR_BARO))   // No Sensors just go down
                {
                    rcData[THROTTLE] = cfg.failsafe_throttle;
                    if (currentTimeMS > Failsafetimer) DisArmCopter();// Turn OFF motors Time in 0.1sec
                }

                if (sensors(SENSOR_GPS) && !sensors(SENSOR_BARO))    // Just GPS. We initiate RTL not caring if homepos is set, because he is going down anyway
                {
                    rcData[THROTTLE] = cfg.failsafe_throttle;
                    if (f.GPS_FIX_HOME && cfg.failsafe_justph == 0)
                    {
                        rcOptions[BOXGPSHOME]  = 1;                  // Do RTL
                        rcOptions[BOXGPSHOLD]  = 0;                  // No explicit PH
                    }
                    else
                    {
                        rcOptions[BOXGPSHOME]  = 0;                  // No RTL
                        rcOptions[BOXGPSHOLD]  = 1;                  // Just PH
                    }
                    if (currentTimeMS > Failsafetimer) DisArmCopter();// Turn OFF motors Time
                }

                if (!sensors(SENSOR_GPS) && sensors(SENSOR_BARO))    // Just Baro. Do Autoland and drift into next tree
                {
                    rcData[THROTTLE]       = cfg.mincheck - 10;      // Do Autoland turns off motors
                }

                if (sensors(SENSOR_GPS) && sensors(SENSOR_BARO))     // GPS & Baro. Do RTH & Autoland. If Homepos not set just Autoland & try PH
                {
                    if (f.GPS_FIX_HOME && cfg.failsafe_justph == 0)
                    {
                        rcOptions[BOXGPSHOME]  = 1;                  // Do RTL+Autoland+MotorOFF
                        rcOptions[BOXGPSHOLD]  = 0;                  // No explicit PH
                    }
                    else                                             // OMG we have no Homepos - just do Autoland, turn on PH just in case...
                    {
                        rcOptions[BOXGPSHOME]  = 0;
                        rcOptions[BOXGPSHOLD]  = 1;                  // Pos Hold ON
                        rcData[THROTTLE]       = cfg.mincheck-10;    // Do Autoland turns off motors
                    }
                }
            }
            else Failsafetimer = 0;
        }

        if (feature(FEATURE_FAILSAFE))
        {
            if (failsafeCnt == 0)
            {
                GoodRCcnt ++;
                if (GoodRCcnt > 250) GoodRCcnt = 250;
            }
            else GoodRCcnt = 0;
            failsafeCnt++;                                           // reset to 0 by pwm / spektrum driver on Signal
        }
        else failsafeCnt = 0;

//      SPECIAL RTL Crashpilot
//      Full RTL with Althold+Hightcheck+Autoland+Disarm
        if (rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD])          // Switch to Angle & MAG mode when GPS is on anyway
        {
            rcOptions[BOXHORIZON] = 0;
            rcOptions[BOXANGLE]   = 1;
            rcOptions[BOXMAG]     = 1;
        }

#define RTLsettleTime 2000                                        // 2 sec
#define RTLClimbRate  64
        if (sensors(SENSOR_GPS) && sensors(SENSOR_BARO) && f.GPS_FIX_HOME && rcOptions[BOXGPSHOME])
        {
            rcOptions[BOXBARO]     = 1;                              // Baro On
            rcOptions[BOXMAG]      = 1;                              // MAG ON Idea: Louis
            rcOptions[BOXPASSTHRU] = 0;                              // Passthru off
            rcOptions[BOXHEADFREE] = 0;                              // HeadFree off
            rcOptions[BOXGPSHOLD]  = 1;                              // GPS hold
            rcOptions[BOXGPSHOME]  = 0;                              // RTL OFF
            rcData[THROTTLE] = cfg.midrc;                            // Put throttlestick to middle: Althold
            PHminSat = 5;                                            // Sloppy PH is sufficient
            if (RTLstate == 0)  RTLstate = 1;                        // Start RTL Sequence if it isn't already running
            if (GPS_numSat < 5) RTLstate = 0;                        // Error!
            if (cfg.gps_rtl_mindist != 0 && RTLstate == 1 && GPS_distanceToHome < cfg.gps_rtl_mindist)
                RTLstate = 0;                                        // Dont Do RTL if too close and RTL not already running
            
            switch (RTLstate)
            {
            case 0:                                                  // Error!! Do landing
                rcData[THROTTLE] = cfg.mincheck-10;                  // Put throttlestick to lowest-10
                break;
            case 1:                                                  // prepare timer
                RTLGeneralTimer = currentTimeMS + RTLsettleTime;
                RTLstate++;
                break;
            case 2:                                                  // Hover certain time and wait for solid PH
                if (currentTimeMS > RTLGeneralTimer && ph_status == PH_STATUS_DONE) RTLstate++;
                break;
            case 3:                                                  // Check hight and climb if neccessary
                if (cfg.gps_rtl_minhight == 0) RTLstate++;           // For safety, skip if turned off
                else
                {
                    if (EstAlt < (cfg.gps_rtl_minhight * 100)) rcData[THROTTLE] = cfg.midrc + cfg.alt_hold_throttle_neutral + RTLClimbRate;
                    else RTLstate++;
                }
                break;
            case 4:                                                  // Wait for Tailstuff before RTL
                if (cfg.nav_controls_heading == 1)                   // Tail control
                {
                    if (cfg.nav_tail_first == 1) magHold = wrap_18000(((float)GPS_directionToHome * 100) - 18000) * 0.01f;
                    else magHold = GPS_directionToHome;
                    tmp0 = heading - magHold;                        // tmp0 contains headingdifference
                    if (tmp0 <= -180) tmp0 += 360;
                    if (tmp0 >= +180) tmp0 -= 360;
                    if (abs(tmp0) < 5) RTLstate++;                   // Turns true, when in range of +-5 degrees
                }
                else RTLstate++;
                break;
            case 5:                                                  // Prepare RTL
                DistanceToHomeMetersOnRTLstart = GPS_distanceToHome; // Set actual distance to Home in meters
                rcOptions[BOXGPSHOLD]  = 0;                          // GPS hold OFF
                rcOptions[BOXGPSHOME]  = 1;                          // Engage RTL
                RTLstate++;
                break;
            case 6:                                                  // OMG Do the f** RTL now
                rcOptions[BOXGPSHOLD]  = 0;                          // GPS hold OFF
                rcOptions[BOXGPSHOME]  = 1;                          // RTL
                tmp0 = (int16_t)GPS_distanceToHome - DistanceToHomeMetersOnRTLstart; // tmp0 contains flyawayvalue
                if ((cfg.gps_rtl_flyaway !=0 && tmp0 > (int16_t)cfg.gps_rtl_flyaway) ||
                   (wp_status == WP_STATUS_DONE && ph_status  == PH_STATUS_DONE)) RTLstate++;
                break;
            case 7:                                                  // Do Autoland
                rcData[THROTTLE] = cfg.mincheck-10;                  // Put throttlestick to lowest-10
                break;                                               // Repeat forever because Autoland will disarm the thing
            }
        }
        else RTLstate = 0;                                           // No BOXGPSHOME request? Reset Variable
//      SPECIAL RTL Crashpilot END

        if (rcOptions[BOXANGLE] && sensors(SENSOR_ACC))
        {
            if (!f.ANGLE_MODE)
            {
                errorAngleI[ROLL] = 0;                               // bumpless transfer
                errorAngleI[PITCH] = 0;
                f.ANGLE_MODE = 1;
            }
        }
        else f.ANGLE_MODE = 0;

        if (rcOptions[BOXHORIZON] && sensors(SENSOR_ACC))
        {
            if (!f.HORIZON_MODE)
            {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.HORIZON_MODE = 1;
            }
        }
        else f.HORIZON_MODE = 0;

        if ((GPS_numSat < 5 || !f.GPS_FIX) &&(f.ANGLE_MODE || f.HORIZON_MODE)) LD1_ON();
        else LD1_OFF();

        if (rcOptions[BOXGPSLOG])
        {
            if (!f.GPS_LOG_MODE) f.GPS_LOG_MODE = GPSFloppyInitWrite(); // Will turn false if not armed or anything else is fucked up
        } else f.GPS_LOG_MODE = 0;

#ifdef BARO
        if (!f.ARMED)                                                // Reset Baro stuff while not armed, but keep the other shit running so that poor user can see a green box
            f.BARO_MODE = 0;                                         // and not cry in the forums my baro is dead, and someone writes some code that also tries to read out the baro serial number.

        if (sensors(SENSOR_BARO))
        {
            if (rcOptions[BOXBARO] && GroundAltInitialized)
            {
                if (!f.BARO_MODE)                                    // Initialize Baromode here if it isn't already
                {
                    AltHold             = EstAlt;
                    AltRCTimer0         = 0;
                    ThrFstTimeCenter    = 0;
                    Althightchange      = 0;
                    AutolandState       = 0;
                    initialThrottleHold = rcCommand[THROTTLE];
                    LastAltThrottle     = rcCommand[THROTTLE];
                    f.BARO_MODE         = 1;                         // Finally set baromode to initialized
                }
                else
                {                                                    // Baromode initialized check for Autolanding
                    if (rcData[THROTTLE] < cfg.mincheck && AutolandState == 0) AutolandState = 1;     // Start Autoland
                    if (rcData[THROTTLE] > cfg.mincheck && AutolandState != 0)// Autolandus interruptus on Userinput reset some stuff
                    {
                        AutolandState       = 0;   
                        ThrFstTimeCenter    = 0;
                        Althightchange      = 0;
                        AltHold             = EstAlt;
                        initialThrottleHold = LastAltThrottle;
                    }
                }
            }
            else
            {
                f.BARO_MODE   = 0;
                AutolandState = 0;                                   // No Baroswitch, no Autoland
            }
        }
        else
        {
            AutolandState = 0;                                      // No Baro, no Autoland
            f.BARO_MODE   = 0;                                      // No Baromode
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_MAG) &&  cfg.mag_calibrated == 1)
        {
            if (rcOptions[BOXMAG])
            {
                if (!f.MAG_MODE)
                {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            }
            else f.MAG_MODE = 0;

            if (rcOptions[BOXHEADFREE])
            {
                if (!f.HEADFREE_MODE) f.HEADFREE_MODE = 1;
            }
            else f.HEADFREE_MODE = 0;

            if (rcOptions[BOXHEADADJ]) headFreeModeHold = heading;   // acquire new heading
        }
#endif

        if (sensors(SENSOR_GPS))
        {

            if (f.GPS_FIX && GPS_numSat >= 5)
            {

                if (rcOptions[BOXGPSHOME] && f.GPS_FIX_HOME)         // Crashpilot RTH is possible with 5 Sats for emergency and if homepos is set!
                {
                    if (!f.GPS_HOME_MODE)
                    {
                        f.GPS_HOME_MODE = 1;
                        nav_mode = NAV_MODE_RTL;                     // Set nav_mode before so GPS_set_next_wp can init it.
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                    }
                }
                else f.GPS_HOME_MODE = 0;

                if (rcOptions[BOXGPSHOLD] && GPS_numSat >= PHminSat) // Crashpilot Only do poshold with specified Satnr or more
                {
                    if (!f.GPS_HOLD_MODE)
                    {
                        f.GPS_HOLD_MODE = 1;
                        nav_mode = NAV_MODE_POSHOLD;
                        GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON]);
                    }
                }
                else f.GPS_HOLD_MODE = 0;

            }
            else
            {
                f.GPS_HOME_MODE = 0;
                f.GPS_HOLD_MODE = 0;
                nav_mode = NAV_MODE_NONE;
            }

        }                                                            // END of sensors SENSOR_GPS

        else

        {
            f.GPS_HOME_MODE = 0;
            f.GPS_HOLD_MODE = 0;
            nav_mode = NAV_MODE_NONE;
        }

        if (rcOptions[BOXPASSTHRU]) f.PASSTHRU_MODE = 1;
        else f.PASSTHRU_MODE = 0;

        if (cfg.mixerConfiguration == MULTITYPE_FLYING_WING || cfg.mixerConfiguration == MULTITYPE_AIRPLANE) f.HEADFREE_MODE = 0;

        if (DoingGPS() && cfg.gps_adddb != 0)                        // Do some additional deadband for GPS, if needed
        {
            rcCommand[PITCH] = RCDeadband(rcCommand[PITCH], cfg.gps_adddb);
            rcCommand[ROLL]  = RCDeadband(rcCommand[ROLL],  cfg.gps_adddb);
        }        

// AT THE VERY END DO SOME KILLSWITCHSTUFF, IF NEEDED
// If Copter is armed by box and a cfg.killswitchtime (in ms) is defined
// Don't enable Killswitch when Notarmed or Rc Signal disturbed, dont do killswitch if FEATURE_INFLIGHT_ACC_CAL is wanted
        if (cfg.activate[BOXARM] > 0 && rcOptions[BOXARM] == 0 && Killtimer == 0 && cfg.killswitchtime != 0) Killtimer = currentTimeMS + (uint32_t)cfg.killswitchtime;
        if (!f.ARMED || rcOptions[BOXARM] == 1 || cfg.killswitchtime == 0 || failsafeCnt > 1 || feature(FEATURE_INFLIGHT_ACC_CAL)) Killtimer = 0;
        if (Killtimer != 0 && currentTimeMS >= Killtimer) DisArmCopter(); // Kill Copter
// AT THE VERY END DO SOME KILLSWITCHSTUFF, IF NEEDED

// *********** END OF 50Hz RC LOOP ***********

    }

// SCHEDULE EEPROM WRITES HERE
// WHY?
// 1. Because all changes within that timeout are collected and then written at once without bothering the eeprom too much
// 2. While flying you can save so many Data as you want in cfg.x and when you land and disarm they are actually written
    
    if (ScheduleEEPROMwriteMS != 0 && currentTimeMS >= ScheduleEEPROMwriteMS && !f.ARMED)
    {
        ScheduleEEPROMwriteMS = 0;                                   // Reset Timer do this only once
        writeParams(0);                                              // Write, don't blink
    }
    
    
#ifdef SONAR
    if (sensors(SENSOR_SONAR))
    {
        Sonar_update();                                              // And update "SonarStatus"
        if (cfg.snr_debug == 1) debug[0] = sonarAlt;
    }
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) Mag_getADC();
#endif

    currentTimeMS = millis();                                        // Feed values if needed in outer timeloop
    currentTime   = micros();
    if (currentTime - LastLoopTime < cfg.looptime)                   // Waste away some time?
    {
        serialCom();                                                 // Yes! Do serial then
        LastSerialTimeMS = currentTimeMS;
        return;                                                      // Return to main "while" endless loop
    }
    LastLoopTime = currentTime;

    computeIMU();                                                    // looptime Timeloop starts here on predefined basis

    currentTime   = micros();		                                     // Crashpilot moved it here
    cycleTime     = currentTime - previousTime;
    previousTime  = currentTime;
    currentTimeMS = millis();

#ifdef BARO
    if (sensors(SENSOR_BARO))                                        // The normal stuff to keep it simple
    {
        Baro_update();
        getEstimatedAltitude();
        getAltitudePID();
    }

#define HoverTimeBeforeLand     2000                                 // Wait 2 sec in the air for VirtualThrottle to catch up
    if (sensors(SENSOR_BARO) && f.BARO_MODE && f.ARMED)              // GroundAltInitialized must not be checked but armed, in case of dumb user -> see above
    {
        switch (AutolandState)
        {
        case 0:                                                      // No Autoland Do nothing
            SnrLandThrlimiter     = 0;                               // Reset it here!
            BaroLandThrlimiter    = 0;
            AutolandGeneralTimer  = 0;
            break;
        
        case 1:                                                      // Start Althold
            rcData[THROTTLE]     = cfg.midrc;                        // Put throttlestick to middle
            AutolandGeneralTimer = currentTimeMS + HoverTimeBeforeLand;// prepare timer
            HoverThrcnt          = 1;                                // Initialize Hoverthrottlestuff here
            HoverThrottle        = LastAltThrottle;
            AutolandState++;
            break;
        
        case 2:                                                      // We hover here and gather the Hoverthrottle
            rcData[THROTTLE]     = cfg.midrc;                        // Put throttlestick to middle: Hover some time to gather Hoverthr
            HoverThrottle       += LastAltThrottle;
            HoverThrcnt ++;
            if (HoverThrcnt == 20)
            {
              HoverThrottle      = HoverThrottle / 20;               // Average of 20 Values
              HoverThrcnt        = 0;
            }
            if (HoverThrcnt == 0 && currentTimeMS > AutolandGeneralTimer) AutolandState++; // Wait for Hoverthrottle to finish before proceeding
            break;
            
        case 3:                                                      // Start descent initialize Variables
            if (cfg.al_debounce != 0)                                // Set BaroLandThrlimiter now, if wanted
            {
                tmp0 = (int16_t)HoverThrottle - cfg.minthrottle;     // tmp0 contains absolute absolute hoverthrottle
                if (tmp0 > 0)                                        // Check for crazy error here to be on the safe side
                    BaroLandThrlimiter = HoverThrottle + ((float)tmp0 * (float)cfg.al_debounce * 0.01f);
                else                                                 // Something is very wrong here don't set BaroLandThrlimiter
                    BaroLandThrlimiter = 0;
            }
            rcData[THROTTLE] = cfg.midrc - cfg.alt_hold_throttle_neutral - cfg.al_barolr;
            if (sensors(SENSOR_SONAR) && SonarStatus == 2)           // Set al_snrlr on steady sonar contact
                rcData[THROTTLE] = cfg.midrc - cfg.alt_hold_throttle_neutral - cfg.al_snrlr;
            AutolandGeneralTimer = 0;
            AutolandState++;
            break;
            
        case 4:                                                      // Keep descending and check for landing
            rcData[THROTTLE] = cfg.midrc - cfg.alt_hold_throttle_neutral - cfg.al_barolr;
            if (sensors(SENSOR_SONAR))                               // Adjust Landing
            {
                if (SonarStatus == 2)                                // SolidSonarContact use maybe different Landrate
                    rcData[THROTTLE] = cfg.midrc - cfg.alt_hold_throttle_neutral - cfg.al_snrlr;
                if (cfg.snr_land == 1 && SonarBreach == 1 && SnrLandThrlimiter == 0) // Sonarlanding if SonarBreach = 1 (Proximity breach) fix the upper throttlevalue
                    SnrLandThrlimiter = cfg.maxthrottle;                             // Set maximal thr as upper limit, will be adjusted below
            }
            if (LastAltThrottle <= LandDetectMinThr && AutolandGeneralTimer == 0)    // LandDetectMinThr is set upon Baro initialization in sensors/sensorsAutodetect
            {
                if (SnrLandThrlimiter == 0)
                    AutolandGeneralTimer = currentTimeMS + (uint32_t)cfg.al_tobaro;  // No Sonar limiter, do the normal timeout
                else
                    AutolandGeneralTimer = currentTimeMS + (uint32_t)cfg.al_tosnr;   // Aided Sonar landing is wanted and limiter set, do perhaps shorter timeout then
            }
            if (LastAltThrottle > LandDetectMinThr) AutolandGeneralTimer = 0;        // Reset Timer
            if (AutolandGeneralTimer != 0 && currentTimeMS > AutolandGeneralTimer) AutolandState++;
            if (TiltValue < 0)  AutolandState++;                     // Proceed to disarm if copter upside down
            break;
            
        case 5:                                                      // Shut down Copter forever....
            DisArmCopter();
            break;
        }
        
        thrdiff = rcData[THROTTLE] - cfg.midrc;
        tmp0    = abs(thrdiff);
        if (tmp0 < cfg.alt_hold_throttle_neutral && ThrFstTimeCenter == 0) ThrFstTimeCenter = 1;
        if (currentTimeMS >= AltRCTimer0)                                                                                       // X Hz Loop
        {
            AltRCTimer0 = currentTimeMS + 100;
            if (ThrFstTimeCenter == 1 && tmp0 > cfg.alt_hold_throttle_neutral)
            {
                initialThrottleHold = initialThrottleHold + (BaroP / 100);							                                      // Adjust Baselinethr by 1% of BaroP
                if (LastAltThrottle < cfg.maxthrottle && thrdiff >= 0)
                {
                    Althightchange = 1;
                    AltHold += (float)(thrdiff - cfg.alt_hold_throttle_neutral) * 0.125f;
                }
                if (LastAltThrottle > cfg.minthrottle && thrdiff < 0)
                {
                    Althightchange = -1;
                    if (AutolandState == 0) AltHold += ((float)thrdiff * cfg.barodownscale + (float)cfg.alt_hold_throttle_neutral) * 0.125f; // Descent with less rate
                    else AltHold += ((float)thrdiff * 0.6f + (float)cfg.alt_hold_throttle_neutral) * 0.125f;                 // Autoland with spec. rate
                }
            }
            else                                                                                                              // Stick is to center here
            {
                if (ThrFstTimeCenter == 1 && Althightchange != 0)                                                             // Are we coming from a hight change?
								{
								    AltHold = EstAlt + vario * cfg.baro_lag ;                                                             // We are coming from a hightchange project stoppingpoint
                    initialThrottleHold = LastAltThrottle;                                                                    // This is for starting in althold otherwise the initialthr would be idle throttle
								}
                Althightchange = 0;
            }
        }                                                                                                                     // End of X Hz Loop

        if (AutolandState != 0) BaroD = 0;                                                                                    // Don't do Throttle angle correction when autolanding
        rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroP + BaroD - BaroI, cfg.minthrottle, cfg.maxthrottle);

        if (AutolandState != 0)                                                                                               // We are Autolanding and 
        {
            if (SnrLandThrlimiter != 0)                                                                                       // Check sonarlimiter first
            {                                                                                                                 // Sonar has given proximity alert and sonar land support is wanted
                if (LastAltThrottle < SnrLandThrlimiter) SnrLandThrlimiter = LastAltThrottle;                                 // Adjust limiter here
                if (rcCommand[THROTTLE] > SnrLandThrlimiter) rcCommand[THROTTLE] = SnrLandThrlimiter;                         // We are autolanding and Limiter is set
            }
            else                                                                                                              // Only do Barolimiter on landing, if we have no Sonarlimiter
            {
                if (BaroLandThrlimiter != 0 && rcCommand[THROTTLE] > BaroLandThrlimiter)
                    rcCommand[THROTTLE] = BaroLandThrlimiter;
            }
        }
        LastAltThrottle = rcCommand[THROTTLE];
    }
    
// Baro STATS LOGGING
        if (sensors(SENSOR_BARO) && f.ARMED)
        {
            tmp0 = (int16_t)((int32_t)EstAlt / 100);
            if (tmp0 > cfg.MaxAltMeter) cfg.MaxAltMeter = tmp0;
            if (tmp0 < cfg.MinAltMeter) cfg.MinAltMeter = tmp0;
        }
// Baro STATS LOGGING

#endif

#ifdef MAG
    if (sensors(SENSOR_MAG))
    {
        float magP;
        if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE)
        {
            float dif = heading - magHold;
            if (dif <= -180.0f) dif += 360.0f;
            if (dif >= +180.0f) dif -= 360.0f;
            if (DoingGPS()) magP = (float)cfg.gps_yaw;               // Do slower mag P in gps mode
            else magP = (float)cfg.P8[PIDMAG];
            if (f.SMALL_ANGLES_25) rcCommand[YAW] -= dif * magP / 30;// 18 deg
        }
        else magHold = heading;
    }
#endif

    if (sensors(SENSOR_GPS) && sensors(SENSOR_MAG))                  // Only do GPS stuff if the Mag is available
    {
        GPS_alltime();                                               // Do INS GPS stuff here

// GPS/MAG/BARO LOGGING
        if (f.GPS_LOG_MODE && currentTimeMS >= GPSlogTimer)          // Do logging Lat/Lon/Alt/Hdg every 2 secs. Conditions are checked in WriteNextFloppyDataset()
        {
            GPSlogTimer = currentTimeMS + 2000;
            WriteNextFloppyDataset();                                // Will Log as much as possible and set writeschedule
        }
// GPS/MAG/BARO LOGGING
        
        if (!DoingGPS() || !f.GPS_FIX_HOME || cfg.mag_calibrated != 1)
        {
            GPS_reset_nav();
            nav_mode  = NAV_MODE_NONE;
            ph_status = PH_STATUS_NONE;
            wp_status = WP_STATUS_NONE;
        }
        else                                                         // proceed here if mag calibrated and the rest is fine as well
        {
            if (cfg.gps_phase == 0)                                  // Do Phaseshift, if wanted
            {
                CosYawxPhase = cos_yaw_x;
                SinYawyPhase = sin_yaw_y;
            }
            else
            {
                TmpPhase = heading + (float)cfg.gps_phase;           // add Phase
                if (TmpPhase > 180.0f) TmpPhase = TmpPhase - 360.0f; // Wrap to -180 0 +180 Degree
                else if (TmpPhase < -180.0f) TmpPhase = TmpPhase + 360.0f;
                TmpPhase     = TmpPhase * RADX;                      // Degree to RAD
                CosYawxPhase = cosf(TmpPhase);
                SinYawyPhase = sinf(TmpPhase);
            }

            if (cfg.nav_slew_rate != 0 )
            {
                nav_rated[LON]  += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                nav_rated[LAT]  += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
                GPS_angle[ROLL]  = (nav_rated[LON] * CosYawxPhase - nav_rated[LAT] * SinYawyPhase) * 0.1f;
                GPS_angle[PITCH] = (nav_rated[LON] * SinYawyPhase + nav_rated[LAT] * CosYawxPhase) * 0.1f;
            }
            else
            {
                GPS_angle[ROLL]  = (nav[LON] * CosYawxPhase - nav[LAT] * SinYawyPhase) * 0.1f;
                GPS_angle[PITCH] = (nav[LON] * SinYawyPhase + nav[LAT] * CosYawxPhase) * 0.1f;
            }
        }
    } else GPS_angle[0] = GPS_angle[1] = 0;

    tmp0flt = (float)cycleTime;
    MwiiTimescale = tmp0flt * 3.3333333e-4f;
    dT   = tmp0flt * 0.000001f;                                      // pt1 element http://www.multiwii.com/forum/viewtopic.php?f=23&t=2624
    prop = (float)max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL]));  // range [0;500] Crashpilot: prop is float now

    switch (cfg.mainpidctrl)
    {
    case 1:                                                          // 1 = OriginalMwiiPid pimped by me
        for (axis = 0; axis < 3; axis++)
        {
            if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2)        // MODE relying on ACC 50 degrees max inclination
            {
                errorAngle  = constrain(2.0f * (float)rcCommand[axis] + GPS_angle[axis], -500.0f, +500.0f) - angle[axis] + (float)cfg.angleTrim[axis]; //  Removed INFO BRM errorAngle = errorAngle * (float)cycleTime / BasePIDtime; // Crashpilot: Include Cylcletime take 3ms as basis. More deltaT more error
                PTermACC    = errorAngle * (float)cfg.P8[PIDLEVEL] * 0.01f;
                tmp0flt     = (float)cfg.D8[PIDLEVEL] * 5.0f;
                PTermACC    = constrain(PTermACC, -tmp0flt, +tmp0flt);
                errorAngle *= MwiiTimescale;
                errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000.0f, +10000.0f);
                ITermACC    = (errorAngleI[axis] * (float)cfg.I8[PIDLEVEL]) / 4096.0f;
            }

            if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2)        // MODE relying on GYRO or YAW axis
            {
                error  = (float)rcCommand[axis] * 80.0f / (float)cfg.P8[axis]; // Removed INFO BRM error  = error * (float)cycleTime / BasePIDtime;     // Crashpilot: Include Cylcletime take 3ms as basis. More deltaT more error
                error -= gyroData[axis];
                PTermGYRO = (float)rcCommand[axis];
                error *= MwiiTimescale;
                errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000.0f, +16000.0f);
                if (abs(gyroData[axis]) > 640.0f) errorGyroI[axis] = 0;
                ITermGYRO = errorGyroI[axis] * (float)cfg.I8[axis] * 0.000125f;
            }

            if (f.HORIZON_MODE && axis < 2)
            {
                PTerm = (PTermACC * (500.0f - prop) + PTermGYRO * prop) * 0.002f;
                ITerm = (ITermACC * (500.0f - prop) + ITermGYRO * prop) * 0.002f;
            }
            else
            {
                if (f.ANGLE_MODE && axis < 2)
                {
                    PTerm = PTermACC;
                    ITerm = ITermACC;
                }
                else
                {
                    PTerm = PTermGYRO;
                    ITerm = ITermGYRO;
                }
            }
            PTerm          -= (gyroData[axis] * (float)dynP8[axis] * 0.0125f);
            delta           = (float)(gyroData[axis] - lastGyro[axis]) / MwiiTimescale;
            lastGyro[axis]  = gyroData[axis];
            deltaSum        = delta1[axis] + delta2[axis] + delta;
            delta2[axis]    = delta1[axis];
            delta1[axis]    = delta;
            if (cfg.mainpt1cut != 0)						                     // pt1 element http://www.multiwii.com/forum/viewtopic.php?f=23&t=2624
            {
                deltaSum        = lastDTerm[axis] + (dT / (MainDpt1Cut + dT)) * (deltaSum - lastDTerm[axis]);
                lastDTerm[axis] = deltaSum;
            }
            DTerm           = deltaSum * (float)dynD8[axis] * 0.03125f;
            axisPID[axis]   = PTerm + ITerm - DTerm;
        }
        break;      

// Alternative Controller by alex.khoroshko http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=30#p37465
    case 2:                                                          // 2 = New mwii controller (float pimped + pt1element)
        for (axis = 0; axis < 3; axis++)                             // Get the desired angle rate depending on flight mode
        {
            if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2)        // MODE relying on ACC
                errorAngle = constrain(2.0f * (float)rcCommand[axis] + GPS_angle[axis], -500.0f, +500.0f) - angle[axis] + (float)cfg.angleTrim[axis];

            if (axis == 2)
                AngleRateTmp = (float)(cfg.yawRate + 27) * (float)rcCommand[2] * 0.03125f; // AngleRateTmp = (((int32_t)(cfg.yawRate + 27) * rcCommand[2]) >> 5);
            else
            {
                if (!f.ANGLE_MODE)                                   //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                {
                    AngleRateTmp = (float)(cfg.rollPitchRate + 27) * (float)rcCommand[axis] * 0.0625f; // AngleRateTmp = ((int32_t) (cfg.rollPitchRate + 27) * rcCommand[axis]) >> 4;
                    if (f.HORIZON_MODE)
                        AngleRateTmp += (float)cfg.I8[PIDLEVEL] * errorAngle * 0.0390625f; //increased by x10 //0.00390625f AngleRateTmp += (errorAngle * (float)cfg.I8[PIDLEVEL]) >> 8;
                }
                else                                                 // it's the ANGLE mode - control is angle based, so control loop is needed
                    AngleRateTmp = (float)cfg.P8[PIDLEVEL] * errorAngle * 0.0223214286f; // AngleRateTmp = (errorAngle * (float)cfg.P8[PIDLEVEL]) >> 4; * LevelPprescale;
            }
            RateError         = AngleRateTmp - gyroData[axis];
            PTerm             = (float)cfg.P8[axis] * RateError * 0.0078125f;
            errorGyroI[axis] += (float)cfg.I8[axis] * RateError * (float)cycleTime / 2048.0f;
            errorGyroI[axis]  = constrain(errorGyroI[axis], -newpidimax, newpidimax);
            ITerm             = errorGyroI[axis] / 8192.0f;
            delta             = RateError - lastError[axis];
            lastError[axis]   = RateError;
            delta             = delta * 16383.75f / (float)cycleTime;
            deltaSum          = delta1[axis] + delta2[axis] + delta;
            delta2[axis]      = delta1[axis];
            delta1[axis]      = delta;
            if (cfg.mainpt1cut != 0)						                     // pt1 element http://www.multiwii.com/forum/viewtopic.php?f=23&t=2624
            {
                deltaSum        = lastDTerm[axis] + (dT / (MainDpt1Cut + dT)) * (deltaSum - lastDTerm[axis]);
                lastDTerm[axis] = deltaSum;
            }
            DTerm             = (float)cfg.D8[axis] * deltaSum * 0.00390625f;
            axisPID[axis]     = PTerm + ITerm + DTerm;
        }
        break;
    }
    mixTable();
    writeServos();
    writeMotors();
    if (currentTimeMS - LastSerialTimeMS >= 10)                      // If Serial wasn't possible during timewaste (user set looptime too small), do it now and ensure 100Hz
    {
        serialCom();
        LastSerialTimeMS = currentTimeMS;
    }
}
