#include "board.h"
#include "mw.h"
#include <string.h>

config_t cfg;
const char rcChannelLetters[] = "AERT1234";

static uint8_t  EEPROM_CONF_VERSION = 34;
static uint32_t enabledSensors      = 0;
static void resetConf(void);

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s) cfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

static uint8_t validEEPROM(void)
{
    const config_t *temp = (const config_t *)FLASH_WRITE_ADDR;
    const uint8_t  *p;
    uint8_t        chk = 0;

    if (EEPROM_CONF_VERSION != temp->version) return 0;                                   // check version number
    if (temp->size != sizeof(config_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF) return 0; // check size and magic numbers
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(config_t)); p++) chk ^= *p;   // verify integrity of temporary copy
    if (chk != 0) return 0;                                                               // checksum failed
    return 1;                                                                             // looks good, let's roll!
}

void readEEPROM(void)
{
    uint8_t i;
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));                             // Read flash
    for (i = 0; i < 6; i++)
        lookupPitchRollRC[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 2500;

    for (i = 0; i < 11; i++)
    {
        int16_t tmp = 10 * i - cfg.thrMid8;
        uint8_t y = 1;
        if (tmp > 0) y = 100 - cfg.thrMid8;
        if (tmp < 0) y = cfg.thrMid8;
        lookupThrottleRC[i] = 10 * cfg.thrMid8 + tmp * (100 - cfg.thrExpo8 + (int32_t) cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        lookupThrottleRC[i] = cfg.minthrottle + (int32_t) (cfg.maxthrottle - cfg.minthrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }
    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max); //REAR
    GPS_set_pids();                                                                       // Set GPS PIDS in any case
    GPS_reset_nav();
}

void writeParams(uint8_t b)
{
    FLASH_Status status;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;

    cfg.version    = EEPROM_CONF_VERSION;
    cfg.size       = sizeof(config_t);
    cfg.magic_be   = 0xBE;
    cfg.magic_ef   = 0xEF;
    cfg.chk        = 0;
    for (p = (const uint8_t *)&cfg; p < ((const uint8_t *)&cfg + sizeof(config_t)); p++) chk ^= *p; // recalculate checksum before writing
    cfg.chk = chk;
    FLASH_Unlock();                                                                       // write it
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  
    for (i = 0; i < FLASH_PAGES_FORCONFIG; i++)                                           // Erase the pages here
    {
        while (FLASH_ErasePage(FLASH_WRITE_ADDR + (i * FLASH_PAGE_SIZE)) != FLASH_COMPLETE);
    }

    for (i = 0; i < sizeof(config_t); i += 4)                                             // Write that config now.
    {
        status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
        if (status != FLASH_COMPLETE) break;                                              // TODO: fail
    }
    FLASH_Lock();
    readEEPROM();
    if (b) blinkLED(15, 20, 1);
}

void checkFirstTime(bool reset)
{
    if (!validEEPROM() || reset) resetConf();                                             // check the EEPROM integrity
}

// Default settings
static void resetConf(void)
{
    uint8_t i;
    const int8_t default_align[3][3] = { /* GYRO */ { 0, 0, 0 }, /* ACC */ { 0, 0, 0 }, /* MAG */ { -2, -3, 1 } };
    memset(&cfg, 0, sizeof(config_t));

    cfg.version = EEPROM_CONF_VERSION;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    featureClearAll();
    featureSet(FEATURE_VBAT);
//    featureSet(FEATURE_PPM);
    featureSet(FEATURE_FAILSAFE);
//    featureSet(FEATURE_LCD);
    featureSet(FEATURE_GPS);
//    featureSet(FEATURE_PASS);                   // Just pass Throttlechannel
//    featureSet(FEATURE_SONAR);

    cfg.P8[ROLL]                  = 38;         // 40
    cfg.I8[ROLL]                  = 30;
    cfg.D8[ROLL]                  = 23;

    cfg.P8[PITCH]                 = 38;         // 40
    cfg.I8[PITCH]                 = 30;
    cfg.D8[PITCH]                 = 23;

    cfg.P8[YAW]                   = 60;         // 70
    cfg.I8[YAW]                   = 45;

    cfg.P8[PIDALT]                = 100;
    cfg.I8[PIDALT]                = 30;
    cfg.D8[PIDALT]                = 80;

    cfg.P8[PIDPOS]                = 12;         // FIND YOUR VALUE
    cfg.I8[PIDPOS]                =  0;         // NOT USED
    cfg.D8[PIDPOS]                =  0;         // NOT USED

    cfg.P8[PIDPOSR]               = 50;         // FIND YOUR VALUE                    // Controls the speed part with my PH logic
    cfg.I8[PIDPOSR]               =  0;         // 25;  // DANGER "I" may lead to circeling   // Controls the speed part with my PH logic
    cfg.D8[PIDPOSR]               = 50;         // FIND YOUR VALUE                    // Controls the speed part with my PH logic

    cfg.P8[PIDNAVR]               = 45;         // 14 More ?
    cfg.I8[PIDNAVR]               =  0;         // NAV_I * 100;                       // Scaling/Purpose unchanged
    cfg.D8[PIDNAVR]               =  0;         // NAV_D * 1000;                      // Scaling/Purpose unchanged

//    cfg.P8[PIDPOS]                = 11;         // APM PH Stock values
//    cfg.I8[PIDPOS]                = 0;
//    cfg.D8[PIDPOS]                = 0;

//    cfg.P8[PIDPOSR]               = 20;         // POSHOLD_RATE_P * 10;
//    cfg.I8[PIDPOSR]               = 8;          // POSHOLD_RATE_I * 100;
//    cfg.D8[PIDPOSR]               = 45;         // POSHOLD_RATE_D * 1000;

//    cfg.P8[PIDNAVR]               = 14;         // NAV_P * 10;
//    cfg.I8[PIDNAVR]               = 20;         // NAV_I * 100;
//    cfg.D8[PIDNAVR]               = 80;         // NAV_D * 1000;

    cfg.P8[PIDLEVEL]              = 75;         // 70
    cfg.I8[PIDLEVEL]              = 20;
    cfg.D8[PIDLEVEL]              = 50;

    cfg.P8[PIDMAG]                = 50;         // cfg.P8[PIDVEL] = 0;// cfg.I8[PIDVEL] = 0;// cfg.D8[PIDVEL] = 0;

    cfg.rcRate8                   = 45;
    cfg.rcExpo8                   = 50;         // cfg.rollPitchRate = 0;// cfg.yawRate = 0;// cfg.dynThrPID = 0;
    cfg.thrMid8                   = 55;
    cfg.thrExpo8                  = 50;//for (i = 0; i < CHECKBOXITEMS; i++)//cfg.activate[i] = 0;
    // cfg.angleTrim[0] = 0;// cfg.angleTrim[1] = 0;// cfg.accZero[0] = 0;// cfg.accZero[1] = 0;
    // cfg.accZero[2] = 0;// cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    memcpy(&cfg.align, default_align, sizeof(cfg.align));

//    cfg.mag_declination           = 0;          // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    cfg.mag_declination           = 110;        // Crashpilot //cfg.acc_hardware = ACC_DEFAULT;// default/autodetect
    cfg.mag_oldcalib              = 0;          // 1 = old hard iron calibration // 0 = extended calibration (better)
    cfg.mag_oldctime              = 1;          // 1 - 5 Time in MINUTES for old calibration. Use this together with mag_oldcalib = 1 if you have a monster of a copter
    cfg.acc_hardware              = 2;          // Crashpilot MPU6050
    cfg.acc_lpf_factor            = 100;	      // changed 27.11.2012
    cfg.acc_ins_lpf               = 10;         // General LPF for all INS stuff

    cfg.looptime                  = 3000;	      // changed 27.11.2012 //    cfg.acc_lpf_factor = 4;
    cfg.mainpidctrl               = 1;          // 1 = OriginalMwiiPid pimped by me, 2 = New mwii controller (experimental, float pimped + pt1)
    cfg.mainpt1cut                = 15;         // (0-50Hz) 0 Disables pt1element. Cuf Off Frequency for pt1 element D term in Hz of main Pid controller
    cfg.newpidimax                = 256;        // [10-65000) 256 Default. Imax of new Pidcontroller
    cfg.gpspt1cut                 = 10;         // (1-50Hz) Cuf Off Frequency for D term in Hz of GPS Pid controller 
    cfg.gyro_cmpf_factor          = 1000;       // (10-1000) 400 default. Now 1000. The higher, the more weight gets the gyro and the lower is the correction with Acc data.
    cfg.gyro_cmpfm_factor         = 1000;       // (10-2000) 200 default for 10Hz. Now 1000 for 70Hz seems ok. Gyro/Magnetometer Complement. Greater Value means more filter on mag/delay
    
    cfg.gyro_lpf                  = 42;         // Possible values 256 188 98 42 20 10 (HZ)

    // Baro
    cfg.accz_vel_cf               = 0.985f;     // Crashpilot: Value for complementary filter accz and barovelocity
    cfg.accz_alt_cf               = 0.940f;     // Crashpilot: Value for complementary filter accz and altitude
    cfg.baro_lag                  = 0.4f;       // Lag of Baro/Althold stuff in general, makes stop in hightchange snappier
    cfg.barodownscale             = 0.7f;       // Scale downmovement down (because copter drops faster than rising)
    cfg.al_suptime                = 2000;       // 0 = disable // althold is engage autonomously after for ex: 2000 ms when throttle stick is not moving deadband Throttle = 60 
    cfg.al_deadsup                = 60;         // Throttle deadband when Althold support is use up to 200 possible but not recommanded 
    cfg.al_maxthrsup              = 1790;       // Over this throttle value Althold support is disangage
  
    // Autoland
    cfg.al_barolr                 = 110;         // Temporary value "64" increase to increase Landingspeed
    cfg.al_snrlr                  = 75;         // You can specify different landingfactor here on sonar contact, because sonar land maybe too fast when snr_cf is high
    cfg.al_lndthr                 = 0;          // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.minthrottle is taken.
    cfg.al_debounce               = 5;          // (0-20%) 0 Disables. Defines a Throttlelimiter on Autoland. Percentage defines the maximum deviation of assumed hoverthrottle during Autoland
    cfg.al_tobaro                 = 2000;       // Timeout in ms (100 - 5000) before shutoff on autoland. "al_lndthr" must be undershot for that timeperiod
    cfg.al_tosnr                  = 1000;       // Timeout in ms (100 - 5000) If sonar aided land is wanted (snr_land = 1) you can choose a different timeout here

    cfg.baro_debug                = 0;          // Crashpilot: 1 = Debug Barovalues //cfg.baro_noise_lpf = 0.6f;// Crashpilot: Not used anymore//cfg.baro_cf = 0.985f;// Crashpilot: Not used anymore
    cfg.moron_threshold           = 32;
    cfg.gyro_smoothing_factor     = 0x00141403; // default factors of 20, 20, 3 for R/P/Y
    cfg.vbatscale                 = 110;
    cfg.vbatmaxcellvoltage        = 43;
    cfg.vbatmincellvoltage        = 33;
    cfg.power_adc_channel         = 0;

    // Radio
    parseRcChannels("AETR1234");
    cfg.deadband                  = 20;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.yawdeadband               = 20;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.alt_hold_throttle_neutral = 60;         // Crashpilot: A little deadband will not harm our crappy RC
    cfg.gps_adddb                 = 10;          // Additional Deadband for all GPS functions;

    // cfg.spektrum_hires = 0;
    cfg.midrc                     = 1500;
    cfg.mincheck                  = 1100;
    cfg.maxcheck                  = 1900;
    cfg.retarded_arm              = 0;          // disable arm/disarm on roll left/right
    cfg.auxChannels               = 4;          // [4 - 10] cGiesen: Default = 4, then like the standard!
    cfg.killswitchtime            = 0;          // Time in ms when your arm switch becomes a Killswitch, 0 disables the Killswitch, can not be used together with FEATURE_INFLIGHT_ACC_CAL
    cfg.rc_motor                  = 1;          // [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 

    // Motor/ESC/Servo
    cfg.minthrottle               = 1150;       // ORIG
//    cfg.minthrottle               = 1100;
    cfg.maxthrottle               = 1950;
    cfg.esc_nfly                  = 1300;       // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.rc_minchk + 5% is taken. Also baselinethr for Autostart, also plausibility check for initial Failsafethrottle
//  cfg.esc_nfly                  = 0;
    cfg.passmotor                 = 0;          // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor
    cfg.mincommand                = 1000;
    cfg.motor_pwm_rate            = 400;
    cfg.servo_pwm_rate            = 50;

    // servos
    cfg.yaw_direction             = 1;
    cfg.tri_yaw_middle            = 1500;
    cfg.tri_yaw_min               = 1020;
    cfg.tri_yaw_max               = 2000;

    // flying wing
    cfg.wing_left_min             = 1020;
    cfg.wing_left_mid             = 1500;
    cfg.wing_left_max             = 2000;
    cfg.wing_right_min            = 1020;
    cfg.wing_right_mid            = 1500;
    cfg.wing_right_max            = 2000;
    cfg.pitch_direction_l         = 1;
    cfg.pitch_direction_r         = -1;
    cfg.roll_direction_l          = 1;
    cfg.roll_direction_r          = 1;

    // gimbal
    cfg.gimbal_pitch_gain         = 10;
    cfg.gimbal_roll_gain          = 10;
    cfg.gimbal_flags              = GIMBAL_NORMAL;
    cfg.gimbal_pitch_min          = 1020;
    cfg.gimbal_pitch_max          = 2000;
    cfg.gimbal_pitch_mid          = 1500;
    cfg.gimbal_roll_min           = 1020;
    cfg.gimbal_roll_max           = 2000;
    cfg.gimbal_roll_mid           = 1500;

    // gps/nav
    cfg.gps_type                  = 1;          // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
    cfg.gps_baudrate              = 115200;     //38400; // Changed 8/6/13 to 115200;
//    cfg.gps_baudrate              = 38400;      //38400; // Changed 8/6/13 to 115200;
//    cfg.gps_ins_vel               = 0.72f;      // Crashpilot GPS INS The LOWER the value the closer to gps speed // Dont go to high here
    cfg.gps_ins_vel               = 0.6f;       // Crashpilot GPS INS The LOWER the value the closer to gps speed // Dont go to high here
    cfg.gps_ins_mdl               = 1;          // NOTE: KEEP THIS TO "1" FOR NOW because other models work like shit currently. GPS ins model. 1 = Based on lat/lon, 2 = based on Groundcourse & speed,(3 = based on ublx velned deleted)
    cfg.gps_lag                   = 2000;       // GPS Lag in ms
    cfg.gps_phase                 = 0;          // +- 30 Degree Make a phaseshift of GPS output for whatever reason you might want that (frametype etc)
    cfg.gps_ph_minsat             = 6;          // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
    cfg.gps_ph_settlespeed        = 10;         // 1 - 200 cm/s PH settlespeed in cm/s
    cfg.gps_ph_brakemaxangle      = 10;         // 1 - 45 Degree Maximal 5 Degree Overspeedbrake
    cfg.gps_ph_minbrakepercent    = 50;         // 1 - 99% minimal percent of "brakemaxangle" left over for braking. Example brakemaxangle = 6 so 50 Percent is 3..
    cfg.gps_ph_brkacc             = 100;         // [1 - 500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though. It will be a timeout. The lower the Value the longe the Timeout
    cfg.gps_ph_abstub             = 100;        // 0 - 1000cm (300 Dfault, 0 disables) Defines the "bath tub" around current absolute PH Position, where PosP is diminished, reaction gets harder on tubs edge and then goes on linear
    cfg.gps_maxangle              = 25;         // 10 - 45 Degree Maximal over all GPS bank angle
    cfg.gps_wp_radius             = 150;
//    cfg.gps_rtl_minhight          = 20;         // (0 - 200) Minimal RTL hight in m, 0 disables feature
	  cfg.gps_rtl_minhight          = 0;          // (0 - 200) Minimal RTL hight in m, 0 disables feature
    cfg.gps_rtl_mindist           = 0;          // 0 Disables. Minimal distance for RTL in m, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    cfg.gps_rtl_flyaway           = 0;          // 0 Disables. If during RTL the distance increases beyond this value (in meters relative to RTL activation point), something is wrong, autoland
    cfg.gps_yaw                   = 40;         // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    cfg.nav_rtl_lastturn          = 1;          // 1 = when copter gets to home position it rotates it's head to takeoff direction independend of nav_controls_heading
//    cfg.nav_slew_rate             = 30;         // was 30 and 50 before
    cfg.nav_slew_rate             = 20;         // was 30 and 50 before
    cfg.nav_tail_first            = 0;          // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1)
    cfg.nav_controls_heading      = 1;          // 1 = Nav controls YAW during WP ONLY
//    cfg.nav_controls_heading      = 1;          // 1 = Nav controls YAW during WP ONLY
    cfg.nav_speed_min             = 150;        // 10 - 200 cm/s don't set higher than nav_speed_max! That dumbness is not covered.
    cfg.nav_speed_max             = 350;        // 50 - 2000 cm/s don't set lower than nav_speed_min! That dumbness is not covered.
    cfg.nav_approachdiv           = 3;          // 2 - 10 This is the divisor for approach speed for wp_distance. Example: 400cm / 3 = 133cm/s if below nav_speed_min it will be adjusted
    cfg.nav_tiltcomp              = 50;         // 0 - 100 (20 TestDefault) Only arducopter really knows. Dfault was 54. This is some kind of a hack of them to reach actual nav_speed_max. 54 was Dfault, 0 disables
    cfg.nav_ctrkgain              = 1;       // 0 - 10.0 (0.5 TestDefault) (Floatvariable) That is the "Crosstrackgain" APM Dfault is "1". "0" disables

    // Failsafe Variables
    cfg.failsafe_delay            = 10;         // in 0.1s (10 = 1sec)
    cfg.failsafe_off_delay        = 200;        // in 0.1s (200 = 20sec)
    cfg.failsafe_throttle         = 1200;       // decent Dfault which should always be below hover throttle for people.
    cfg.failsafe_deadpilot        = 0;		      // DONT USE, EXPERIMENTAL Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH, 0 disables max 250
    cfg.failsafe_justph           = 1;          // Does just PH&Autoland an not RTL, use this in difficult areas with many obstacles to avoid RTL crash into something
    cfg.failsafe_ignoreSNR        = 1;          // When snr_land is set to 1, it is possible to ignore that on Failsafe, because FS over a tree could turn off copter

    // serial (USART1) baudrate
    cfg.serial_baudrate           = 115200;
    cfg.tele_prot                 = 0;          // Protocol ONLY used when Armed including Baudchange if necessary. 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)

    // LED Stuff
    cfg.LED_invert                = 0;          // Crashpilot: Inversion of LED 0&1 Partly implemented because Bootup is not affected
    cfg.LED_Type                  = 1;		      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
    cfg.LED_Pinout                = 1;		      // rc6
    cfg.LED_ControlChannel        = 8;		      // AUX4 (Channel 8)
    cfg.LED_Armed                 = 0;		      // 0 = Show LED only if armed, 1 = always show LED
    cfg.LED_Pattern1			        = 1300; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Pattern2			        = 1800; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Pattern3			        = 1900; 		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    cfg.LED_Toggle_Delay1         = 0x08;       // slow down LED_Pattern
    cfg.LED_Toggle_Delay2         = 0x08;       // slow down LED_Pattern
    cfg.LED_Toggle_Delay3         = 0x08;       // slow down LED_Pattern

    // SONAR

    // SOME INFO ON SONAR:
    // PWM56 are 5V resistant, RC78 only tolerate 3.3V(!!) so add a 1K Ohms resistor!!!
    // Note: You will never see the maximum possible sonar range in a copter, so go for the half of it (or less?)
    //
    // Connection possibilities depending on Receivertype:
    // PPSUM: RC78 possible, PWM56 possible (on max. quadcopters, see below)
    // Normal RX: Just Connection on Motorchannel 5&6 (PWM56) is possible.
    // The PWM56 sonar connection option is only available in setups with max motors 4, otherwise sonar is not initialized.
    //
    // HC-SR04:
    // Operation Voltage: 5V (!! Use PWM56 or 1K resistor !!)
    // Range: 2cm - 400cm
    // Angle: 15 Degrees (Test out for yourself: cfg.snr_tilt = X)
    //
    // Maxbotics in general
    // Operation Voltage: (some 2.5V)3.3V - 5V ((!! Use PWM56 or resistor with 5V !!)
    // Only wire the Maxbotics for PWM output (more precise anyway), not the analog etc. modes, just wire echopin (normally pin 2)
    // Range: 20cm(!) - 765cm (some >1000cm), MaxTiltAngle is not specified, depending on Model
    // Tested on MB1200 XL-MaxSonar-EZ0
    //
    // GENERAL WARNING: DON'T SET snr_min TOO LOW, OTHERWISE THE WRONG SONARVALUE WILL BE TAKEN AS REAL MEASUREMENT!!
    // I implemented some checks to prevent that user error, but still keep that in mind.
    // Min/Max are checked and changed if they are too stupid for your sonar. So if you suddenly see other values, thats not an eeprom error or so.
    // MAXBOTICS: SET snr_min to at least 25! I check this in sensors and change the value, if needed.
    // NOTE: I limited Maxbotics to 7 meters in the code, knowing that some types will do >10m, if you have one of them 7m is still the limit for you.
    // HC-SR04:   SET snr_min to at least 5 ! I check this in sensors and change the value, if needed.
    // DaddyWalross Sonar: I DON'T KNOW! But it uses HC-SR04 so i apply the same limits (5cm-400cm) to its output
    // NOTE: Sonar is def. not a must - have. But nice to have.
    cfg.snr_type                  = 3;          // 0 = PWM56 HC-SR04, 1 = RC78 HC-SR04, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
    cfg.snr_min                   = 25;         // Valid Sonar minimal range in cm (5-200) see warning above
    cfg.snr_max                   = 200;        // Valid Sonar maximal range in cm (50-700)
    cfg.snr_debug                 = 0;          // 1 Sends Sonardata (within defined range and tilt) to debug[0] and tiltvalue to debug[1], debug[0] will be -1 if out of range/tilt. debug[2] contains raw sonaralt, like before
    cfg.snr_tilt                  = 18;         // Somehow copter tiltrange in degrees (Not exactly but good enough. Value * 0.9 = realtilt) in wich Sonar is possible
    cfg.snr_cf                    = 0.7f;       // The bigger, the more Sonarinfluence, makes switch between Baro/Sonar smoother and defines baroinfluence when sonarcontact. 1.0f just takes Sonar, if contact (otherwise baro)
    cfg.snr_diff                  = 0;          // 0 disables that check. Range (0-200) Maximal allowed difference in cm between sonar readouts (100ms rate and snr_diff = 50 means max 5m/s)
    cfg.snr_land                  = 1;          // Aided Sonar - landing, by setting upper throttle limit to current throttle. - Beware of Trees!! Can be disabled for Failsafe with failsafe_ignoreSNR = 1

    // LOGGING
    cfg.floppy_mode               = FD_MODE_GPSLOGGER; // Usagemode of free Space. 1 = GPS Logger
    cfg.FDUsedDatasets            = 0;          // Default no Datasets stored
    cfg.stat_clear                = 1;          // This will clear the stats between flights, or you can set to 0 and treasue overallstats, but you have to write manually eeprom or have logging enabled
    cfg.sens_1G                   = 1;          // Just feed a dummy "1" to avoid div by zero
    ClearStats();
    // custom mixer. clear by Dfaults.
    for (i = 0; i < MAX_MOTORS; i++) cfg.customMixer[i].throttle = 0.0f;
    writeParams(0);
}

void ClearStats(void)
{
    cfg.GPS_MaxDistToHome = cfg.MAXGPSspeed = 0;
    cfg.MaxAltMeter = cfg.MinAltMeter = 0;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

bool feature(uint32_t mask)
{
    return cfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return cfg.enabledFeatures;
}
