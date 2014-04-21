#pragma once

/* for VBAT monitoring frequency */
#define VBATFREQ 6                          // to read battery voltage - nth number of loop iterations

#define  VERSION  211
#define  FIRMWARE  "Naze32 Harakiri10 Summer Games2.5" __DATE__ " / " __TIME__
#define  FIRMWAREFORLCD "Harakiri 10"

#define LAT  0
#define LON  1
#define GPS_Y 0
#define GPS_X 1

#define AccCalibSamples  1000
#define GyroCalibSamples 1000
#define acc_1G           512.0f
#define INVacc_1G        0.001953125f     // 1/acc_1G
#define acc_25deg        216.576f         // acc_25deg = (int16_t)(acc_1G * 0.423f);
#define SQacc_1G         262144

// Serial GPS only variables
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_RTL,
    NAV_MODE_WP,
    NAV_MODE_CIRCLE
} NavigationMode;

typedef enum WPstatus
{
    WP_STATUS_NONE = 0,
    WP_STATUS_NAVIGATING,
    WP_STATUS_DONE
} WPstatus;

typedef enum PHstatus
{
    PH_STATUS_NONE = 0,
    PH_STATUS_BRAKING,
    PH_STATUS_SETTLING,
    PH_STATUS_DONE
} PHstatus;

typedef enum Protocol
{
    PROTOCOL_AUTOSENSE = 0,
    PROTOCOL_MWII21,
    PROTOCOL_MAVLINK
} Protocol;

typedef enum FloppyDiskType
{
    FD_MODE_NONE = 0,
    FD_MODE_GPSLOGGER,
    FD_MODE_WPLIST,
    FD_MODE_IMULOGGER
} FloppyDiskType;

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,                  // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,               // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,               // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,                // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,                  // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;

typedef enum GimbalFlags
{
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX = 1 << 3,
    GIMBAL_MIXTILT = 1 << 4,
} GimbalFlags;

/*********** RC alias *****************/
enum
{
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

enum
{
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

enum   // This is limited to 32 Boxes!!
{
    BOXANGLE = 0,
    BOXHORIZON,
    BOXBARO,
    BOXMAG,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXARM,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXGPSLOG,
    BOXPASSTHRU,
    BOXHEADFREE,
    BOXBEEPERON,
    BOXHEADADJ,
    BOXOSD,
    BOXFAILSAFE,
    CHECKBOXITEMS
};

static const char boxnames[] =
    "ANGLE;"
    "HORIZON;"
    "BARO;"
    "MAG;"
    "CAMSTAB;"
    "CAMTRIG;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "GPS LOG;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "HEADADJ;"
    "OSD SW;"
    "FAILSAFE;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct motorMixer_t
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixer_t
{
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

enum
{
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

typedef struct config_t
{
    uint8_t  version;
    uint16_t size;
    uint8_t  magic_be;                      // magic number, should be 0xBE
    uint8_t  mixerConfiguration;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
    uint8_t  mainpidctrl;                   // 1 = OriginalMwiiPid pimped by me, 2 = New mwii controller (experimental, float pimped + pt1)
    uint8_t  mainpt1cut;                    // (0-50Hz) 0 Disables pt1element. Cuf Off Frequency for pt1 element D term in Hz of main Pid controller
    uint16_t newpidimax;                    // [10-65000) 256 Default. Imax for Gyropart (incl. Yaw) of new Pidcontroller
    uint8_t  gpspt1cut;                     // (1-50Hz) Cuf Off Frequency for D term in Hz of GPS Pid controller 
    uint8_t  P8[PIDITEMS];
    uint8_t  I8[PIDITEMS];
    uint8_t  D8[PIDITEMS];
    uint8_t  rcRate8;
    uint8_t  rcExpo8;
    uint8_t  thrMid8;
    uint8_t  thrExpo8;
    uint8_t  rollPitchRate;
    uint8_t  yawRate;
    uint8_t  dynThrPID;
    float    accZero[3];
    float    sens_1G;
    float    magZero[3];
    float    sphere_radius;
    uint8_t  mag_calibrated;                // Just to supress crazymag in gui display
    uint8_t  mag_motorcompusable;           // Prepare for future Motorcompensation
    int16_t  mag_declination;               // Get your magnetic decliniation from here : http://magnetic-declination.com/
    uint8_t  mag_oldcalib;                  // 1 = old hard iron calibration // 0 = extended calibration (better)
    uint8_t  mag_oldctime;                  // 1 - 5 Time in MINUTES for old calibration. Use this together with mag_oldcalib = 1 if you have a monster of a copter
    int16_t  angleTrim[2];                  // accelerometer trim
    // sensor-related stuff
    int8_t   align[3][3];                   // acc, gyro, mag alignment (ex: with sensor output of X, Y, Z, align of 1 -3 2 would return X, -Z, Y)
    uint8_t  acc_hardware;                  // Which acc hardware to use on boards with more than one device
    uint8_t  acc_lpf_factor;                // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint16_t gyro_lpf;                      // mpuX050 LPF setting (TODO make it work on L3GD as well)
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gyro_cmpfm_factor;
    uint32_t gyro_smoothing_factor;         // How much to smoothen with per axis (32bit value with Roll, Pitch, Yaw in bits 24, 16, 8 respectively
    float    accz_vel_cf;                   // Crashpilot: Value for complementary filter accz and barovelocity
    float    accz_alt_cf;                   // Crashpilot: Value for complementary filter accz and altitude
    float    baro_lag;                      // Lag of Baro
    float    barodownscale;                 // Scale downmovement down
    uint16_t al_suptime;                    // 0 = disable // althold is engage autonomously after for ex: 2000 ms when throttle stick is not moving deadband fix = 30 Throttle
    uint8_t  al_deadsup;                    // Throttle deadband when Althold support is use up to 200 possible but not recommanded 
    uint8_t  baro_debug;                    // Crashpilot: 1 = Debug Barovalues
    uint8_t  moron_threshold;               // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.

    uint32_t activate[CHECKBOXITEMS];       // activate switches
    uint8_t  vbatscale;                     // adjust this to match battery voltage to reported value
    uint8_t  vbatmaxcellvoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t  vbatmincellvoltage;            // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
    uint8_t  power_adc_channel;             // which channel is used for current sensor. Right now, only 2 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9)

    // Radio/ESC-related configuration
    uint8_t  rcmap[MAX_RC_CHANNELS];        // uint8_t rcmap[8]; // mapping of radio channels to internal RPYTA+ order
    uint8_t  auxChannels;                   // cGiesen: the number of supported aux channels. default = 4
    uint8_t  deadband;                      // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t  yawdeadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t  alt_hold_throttle_neutral;     // defines the neutral zone of throttle stick during altitude hold, default setting is +/-20
    uint8_t  gps_adddb;                     // Additional Deadband for all GPS functions;
    uint8_t  spektrum_hires;                // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t  retarded_arm;                  // allow disarsm/arm on throttle down + roll left/right
    uint16_t killswitchtime;                // Time in ms when your arm switch becomes a Killswitch, 0 disables
    uint8_t  rc_motor;                      // [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 
    
    // Failsafe related configuration
    uint8_t  failsafe_delay;                // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t  failsafe_off_delay;            // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint8_t  failsafe_deadpilot;		        // Time in sec when FS is engaged after idle on THR/YAW/ROLL/PITCH
    uint8_t  failsafe_justph;               // Does just PH&Autoland an not RTL,
    uint8_t  failsafe_ignoreSNR;            // When snr_land is set to 1, it is possible to ignore that on Failsafe, because FS over a tree could turn off copter

    // motor/esc/servo related stuff
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t esc_nfly;                      // This is the absolute throttle that kicks off the "has landed timer" if it is too low cfg.rc_minchk is taken.
    uint8_t  passmotor;                     // Crashpilot: Only used with feature pass. If 0 = all Motors, otherwise specific Motor
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)
    int16_t  servotrim[8];                  // Adjust Servo MID Offset & Swash angles
    int8_t   servoreverse[8];               // Invert servos by setting -1

    // mixer-related configuration
    int8_t   yaw_direction;
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // flying wing related configuration
    uint16_t wing_left_min;                 // min/mid/max servo travel
    uint16_t wing_left_mid;
    uint16_t wing_left_max;
    uint16_t wing_right_min;
    uint16_t wing_right_mid;
    uint16_t wing_right_max;
    int8_t   pitch_direction_l;             // left servo - pitch orientation
    int8_t   pitch_direction_r;             // right servo - pitch orientation (opposite sign to pitch_direction_l if servos are mounted mirrored)
    int8_t   roll_direction_l;              // left servo - roll orientation
    int8_t   roll_direction_r;              // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)

    // gimbal-related configuration
    int8_t   gimbal_pitch_gain;             // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t   gimbal_roll_gain;              // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t  gimbal_flags;                  // in servotilt mode, various things that affect stuff
    uint16_t gimbal_pitch_min;              // gimbal pitch servo min travel
    uint16_t gimbal_pitch_max;              // gimbal pitch servo max travel
    uint16_t gimbal_pitch_mid;              // gimbal pitch servo neutral value
    uint16_t gimbal_roll_min;               // gimbal roll servo min travel
    uint16_t gimbal_roll_max;               // gimbal roll servo max travel
    uint16_t gimbal_roll_mid;               // gimbal roll servo neutral value

    // Autoland
    uint8_t  al_barolr;                     // Temporary value "64" increase to increase Landingspeed
    uint8_t  al_snrlr;                      // You can specify different landingfactor here on sonar contact, because sonar land maybe too fast...
    uint16_t al_lndthr;                     // This is the absolute throttle that kicks off the "has landed timer" if it is "0" or too low cfg.minthrottle is taken.
    uint8_t  al_debounce;                   // (0-20%) 0 Disables. Defines a Throttlelimiter on Autoland. This percentage defines the maximum deviation of assumed hoverthrottle during Autoland
    uint16_t al_tobaro;                     // Timeout in ms (100 - 5000) before shutoff on autoland. "al_lndthr" must be undershot for that timeperiod
    uint16_t al_tosnr;                      // Timeout in ms (100 - 5000) If sonar aided land is wanted (snr_land = 1) you can choose a different timeout here

    // gps-related stuff
    uint8_t  gps_type;                      // Type of GPS hardware. 0: NMEA 1: UBX 2+ ??
    float    gps_ins_vel;                   // Crashpilot: Value for complementary filter INS and GPS Velocity
    uint8_t  gps_ins_mdl;                   // GPS ins model. 1 = Based on lat/lon, 2 = based on Groundcourse & speed, 3 = based on ublx velned

    uint16_t gps_lag;                       // GPS Lag in ms
    int8_t   gps_phase;                     // Make a phaseshift +-30 Deg max of GPS output
    uint8_t  acc_ins_lpf;                   // ACC lowpass for Acc GPS INS
    uint8_t  gps_ph_minsat;                 // Minimal Satcount for PH, PH on RTL is still done with 5Sats or more
    uint8_t  gps_ph_settlespeed;            // PH settlespeed in cm/s
    uint8_t  gps_maxangle;                  // maximal over all GPS bank angle
    uint8_t  gps_ph_brakemaxangle;          // Maximal 5 Degree Overspeedbrake
    uint8_t  gps_ph_minbrakepercent;        // 1-99% minimal percent of "brakemaxangle" left over for braking. Example brakemaxangle = 6 so 50 Percent is 3..
    uint16_t gps_ph_brkacc;                 // [50-500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though
    uint16_t gps_ph_abstub;                 // 0 - 1000cm (300 default) Defines the "bath tub" around current absolute PH Position, where PosP is diminished, reaction gets harder on tubs edge and then goes on linear
    uint32_t gps_baudrate;                  // GPS baudrate
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t  gps_rtl_mindist;               // Minimal distance for RTL, otherwise it will just autoland, prevent Failsafe jump in your face, when arming copter and turning off TX
    uint8_t  gps_rtl_flyaway;               // If during RTL the distance increases beyond this valus in meters, something is wrong, autoland
    uint8_t  gps_yaw;                       // Thats the MAG P during GPS functions, substitute for "cfg.P8[PIDMAG]"
    uint8_t  nav_slew_rate;                 // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t  nav_tail_first;                // 1 = Copter comes back with ass first (only works with nav_controls_heading = 1)
    uint8_t  nav_controls_heading;          // copter faces toward the navigation point, maghold must be enabled for it
    uint8_t  nav_rtl_lastturn;              // Something like NAV_SET_TAKEOFF_HEADING on mwii
    uint8_t  nav_speed_min;                 // 10 - 200 cm/s don't set higher than nav_speed_max! That dumbness is not covered.
    uint16_t nav_speed_max;                 // 50 - 2000 cm/s don't set lower than nav_speed_min! That dumbness is not covered.
    uint8_t  nav_approachdiv;               // 2-10 This is the divisor for approach speed for wp_distance. Example: 400cm / 3 = 133cm/s if below nav_speed_min it will be adjusted
    uint8_t  nav_tiltcomp;                  // 0-100 Only arducopter really knows. This is some kind of a hack of them to reach actual nav_speed_max. 54 was default, 0 disables
    float    nav_ctrkgain;                  // 0 - 10.0 (Floatvariable) That is the "Crosstrackgain" APM default is "1". "0" disables
    uint16_t gps_rtl_minhight;              // Minimal RTL hight in m, 0 disable  // Crashpilot

    uint32_t serial_baudrate;               // serial(uart1) baudrate
    uint8_t  tele_prot;                     // Protocol ONLY used when Armed including Baudchange if necessary. 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)

    // LED Stuff
    uint8_t  LED_invert;                    // Crashpilot invert LED 0&1
    uint8_t  LED_Type;                      // 1=MWCRGB / 2=MONO_LED / 3=LEDRing
    uint8_t  LED_Pinout;                    // choose LED pinout (MONO_LED: 0=LED rc5, 1=LED rc6 / MWCRGB: coming soon)
    uint8_t  LED_ControlChannel;            // RC Channel to control the LED Pattern
    uint8_t  LED_Armed;          		        // 0 = Show LED only if armed, 1 = always show LED
    uint8_t  LED_Toggle_Delay1;             // 16bit bit pattern to slow down led patterns
    uint8_t  LED_Toggle_Delay2;             // 16bit bit pattern to slow down led patterns
    uint8_t  LED_Toggle_Delay3;             // 16bit bit pattern to slow down led patterns
    uint32_t LED_Pattern1;            	  	// 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern2;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000
    uint32_t LED_Pattern3;            		  // 32bit bit pattern to have flickering led patterns / the pattern for MWCRGB 1000-2000

    // Sonar Stuff
    uint8_t  snr_type;                      // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4=MBRC78
    uint8_t  snr_min;                       // Valid Sonar minimal range in cm (0-200)
    uint16_t snr_max;                       // Valid Sonar maximal range in cm (0-700)
    uint8_t  snr_debug;                     // 1 Sets Sonardata within snr_min/max in debug[0]
    uint8_t  snr_tilt;                      // Somehow copter tiltrange in degrees (not exactly) in wich Sonar is possible
    float    snr_cf;                        // The bigger, the more Sonarinfluence
    uint8_t  snr_diff;                      // Maximal allowed difference in cm between sonar readouts (100ms rate and maxdiff = 50 means max 5m/s)
    uint8_t  snr_land;                      // This helps Sonar when landing, by setting upper throttle limit to current throttle. - Beware of Trees!!   
    uint8_t  floppy_mode;                   // Usagemode of free Space. 1 = GPS Logger
    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable

    // LOGGING
    uint8_t  stat_clear;                    // This will clear the stats between flights, or you can set to 0 and treasue overallstats
    uint16_t GPS_MaxDistToHome;             // Treasure Maximal distance from home for later Statistic chart
    uint16_t MAXGPSspeed;                   // Maxspeed in cm/s
    int16_t  MaxAltMeter;
    int16_t  MinAltMeter;

    int32_t  WP_BASE[2];                    // Base LAT/LON Coordinates for WP stuff
    int16_t  WP_BASE_HIGHT;                 // Starthight, normally "0"
    uint16_t FDUsedDatasets;                // Number of valid datasets of current type
    int8_t  FloppyDisk[FDByteSize];         // Reserve 2200 general purpose SIGNED Bytes
    uint8_t  magic_ef;                      // magic number, should be 0xEF
    uint8_t  chk;                           // XOR checksum
} config_t;

typedef struct flags_t
{
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t GPS_LOG_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLES_25;
    uint8_t CALIBRATE_MAG;
    uint8_t FAILSAFE;
} flags_t;

extern float    gyroData[3];
extern float    angle[2];
extern int16_t  axisPID[3];
extern float    newpidimax;

extern int16_t  rcCommand[4];
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern uint16_t failsafeCnt;
extern float    TiltValue;

extern int16_t  debug[4];
extern float    accSmooth[3];
extern float    accADC[3], gyroADC[3], magADCfloat[3];
extern uint32_t currentTime;
extern uint32_t currentTimeMS;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingG;

extern float    BaroAlt;
extern int16_t  sonarAlt;
extern float    EstAlt;
extern float    AltHold;
extern float    vario;

extern float    BaroDeltaTime;
extern int16_t  BaroP;
extern int16_t  BaroI;
extern int16_t  BaroD;
extern bool     newbaroalt;
extern bool     GroundAltInitialized;

extern int16_t  motor[MAX_MOTORS];
extern int16_t  servo[8];
extern int16_t  rcData[MAX_RC_CHANNELS];    // extern int16_t rcData[8];
extern int16_t  rcDataSAVE[MAX_RC_CHANNELS];
extern uint8_t  rssi;                       // 0 - 255 = 0%-100%


extern uint8_t  vbat;
extern int16_t  telemTemperature1;          // gyro sensor temperature
extern int16_t  lookupPitchRollRC[6];       // lookup table for expo & RC rate PITCH+ROLL
extern int16_t  lookupThrottleRC[11];       // lookup table for expo & mid THROTTLE
extern uint8_t  toggleBeep;

// IMU
extern float    ACC_speed[2];
extern float    ACCDeltaTimeINS;

// Sensors
extern float    MainDpt1Cut;
extern float    GPSDpt1freqCut;
extern bool     MpuSpecial;

// MAG
extern float    headFreeModeHold;
extern float    heading;
extern float    magHold;
extern float    magneticDeclination;

// SONAR /BARO / AUTOLAND
extern uint8_t  SonarStatus;                // 0 = no contact, 1 = made contact, 2 = steady contact
extern uint8_t  SonarBreach;                // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
extern uint16_t LandDetectMinThr;           // Is set upon Baro initialization
extern float    pressure;
extern int16_t  ESCnoFlyThrottle;

// GPS stuff
extern int32_t  GPS_coord[2];
extern int32_t  Real_GPS_coord[2];          // RAW GPS Coords
extern int32_t  GPS_home[2];
extern int32_t  GPS_WP[2];                  // Currently used WP
extern uint8_t  GPS_numSat;
extern uint16_t GPS_distanceToHome;         // distance to home
extern int16_t  GPS_directionToHome;        // direction to home
extern uint16_t GPS_altitude, GPS_speed;    // altitude in m and speed in cm/s
extern uint8_t  GPS_update;                 // it's a binary toogle to distinct a GPS position update
extern float    GPS_angle[2];               // it's the angles that must be applied for GPS correction
extern uint16_t GPS_ground_course;          // degrees*10
extern uint8_t  GPS_Present;                // Checksum from Gps serial
extern uint8_t  GPS_Enable;
extern uint32_t GPS_time;                    //UTC time of coord calc - haydent // Shikra osd time
extern float    nav[2];
extern int8_t   nav_mode;                   // Navigation mode
extern int8_t   wp_status;
extern int8_t   ph_status;
extern float    nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
extern int32_t  WP_Target_Alt;
extern int16_t  WP_Desired_Climbrate;
extern bool     WP_Fastcorner;              // Dont decrease Speed at Target
extern float    sin_yaw_y;
extern float    cos_yaw_x;
extern uint32_t TimestampNewGPSdata;        // Crashpilot in micros
extern float    dTnav;                      // Delta Time in milliseconds for navigation computations, updated with every good GPS read
extern int32_t  target_bearing;             // target_bearing is where we should be heading
extern uint32_t wp_distance;
extern float    waypoint_speed_gov;         // used for slow speed wind up when start navigation;
extern int32_t  nav_bearing;                // This is the angle from the copter to the "next_WP" location  with the addition of Crosstrack error in degrees * 100 // Crosstrack eliminated left here on purpose
extern int16_t  nav_takeoff_heading;        // saves the heading at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
extern int32_t  original_target_bearing;    // deg*100, The original angle to the next_WP when the next_WP was set Also used to check when we pass a WP
extern float    LocError[2];                // updated after GPS read - 5-10hz Error in cm from target

// General
extern config_t cfg;
extern flags_t  f;
extern sensor_t acc;
extern sensor_t gyro;
extern baro_t   baro;
extern uint8_t  Currentprotocol;
extern uint32_t ScheduleEEPROMwriteMS;

// Serial
extern bool BlockProtocolChange;

// MWCRGB
extern uint32_t LED_Value;
extern uint32_t LED_Value;
extern uint8_t  LED_Value_Delay;

// Main
void     loop(void);
void     pass(void);
void     LD0_OFF(void);                         // Crashpilot LED Inverter stuff
void     LD1_OFF(void);
void     LD0_ON(void);
void     LD1_ON(void);

// IMU
void     imuInit(void);
void     annexCode(void);
void     computeIMU(void);
void     blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void     getEstimatedAltitude(void);
void     getAltitudePID(void);

// Sensors
void     sensorsAutodetect(void);
void     batteryInit(void);
uint16_t batteryAdcToVoltage(uint16_t src);
void     ACC_getADC(void);
void     alignSensors(uint8_t type, int16_t *data);
void     Baro_update(void);
void     Gyro_getADC(void);
void     Mag_init(void);
int      Mag_getADC(void);
void     Sonar_init(void);
void     Sonar_update(void);
void     MPU6050ReadAllShit(int16_t *accData, int16_t *tempData, int16_t *gyroData);
void     GETMPU6050(void);

// Output
uint8_t  mixerInit(void);

//void mixerInit(void);
void     mixerLoadMix(int index);
void     writeServos(void);
void     writeMotors(void);
void     writeAllMotors(int16_t mc);
void     mixTable(void);

// Serial Mwii & CLI & Mavlink
void     serialInit(uint32_t baudrate);
void     serialCom(void);
void     serialOSD(void);
void     cliSave(char *cmdline);
void     baseflight_mavlink_init(void);

// Config
void     parseRcChannels(const char *input);
void     readEEPROM(void);
void     writeParams(uint8_t b);
void     checkFirstTime(bool reset);
bool     sensors(uint32_t mask);
void     sensorsSet(uint32_t mask);
void     sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool     feature(uint32_t mask);
void     featureSet(uint32_t mask);
void     featureClear(uint32_t mask);
void     featureClearAll(void);
uint32_t featureMask(void);
void     ClearStats(void);

// General RC stuff
void     computeRC(void);
void     GetActualRCdataOutRCDataSave(void);

// RC spektrum
void     spektrumInit(void);
bool     spektrumFrameComplete(void);

// buzzer
void     buzzer(uint8_t warn_vbat);

// cli
void     cliProcess(void);

// gps
void     GPS_set_pids(void);
void     GPS_reset_home_position(void);
void     GPS_reset_nav(void);
void     GPS_alltime(void);
void     GPS_calc_longitude_scaling(void);
void     GPS_set_next_wp(int32_t *lat, int32_t *lon);
void     GPS_calc_velocity(void);
void     GPS_calc_posholdCrashpilot(bool overspeed);
void     GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);
void     GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);
uint16_t GPS_calc_desired_speed(void);
void     GPS_calc_nav_rate(uint16_t max_speed);
bool     check_missed_wp(void);
bool     DoingGPS(void);
float    wrap_18000(float value);

// floppy
bool     GPSFloppyInitWrite(void);
bool     WriteNextFloppyDataset(void);
bool     GPSFloppyInitRead(void);
bool     ReadNextFloppyDataset(uint16_t *DataSetNr, int32_t *DataLAT, int32_t *DataLON, int32_t *DataALT, int16_t *DataHDG);

// telemetry
void     initFRSKYTelemetry(bool State);
void     initMinimOSDTelemetry(bool State);
void     sendFRSKYTelemetry(void);

// Init the led gpio port when enabled
void     ledToggleInit(void);

// Update the leds, enabled signals that the leds are enabled
void     ledToggleUpdate(bool activated);

// Mathematic helper functions here
int16_t  _atan2f(float y, float x);
