#include "board.h"
#include "mw.h"
float        accSmooth[3];
float        accADC[3], gyroADC[3], magADCfloat[3];
float        BaroAlt;
int16_t      sonarAlt;
float        EstAlt;           // in cm
float        AltHold;
float        vario;            // variometer in cm/s + is up
int16_t      BaroP;
int16_t      BaroI;
int16_t      BaroD;
bool         newbaroalt;
bool         GroundAltInitialized = false;
float        ACC_speed[2], BaroDeltaTime;
float        ACCDeltaTimeINS = 0;
static float INV_GYR_CMPF_FACTOR, INV_GYR_CMPFM_FACTOR, INV_ACC_INS_LPF, INV_ACC_LPF;

// **************
// gyro+acc IMU
// **************
/*
 * Sensor data rate:
 * Baro  - 25 Hz  - 40 ms  | 50 ms
 * Accel - 400 Hz - 2.5 ms | 10 ms
 * Mag   - 30 Hz  - 4.5 ms | 40 ms
 * Gyro  - 760 Hz - 1.3 ms | 10 ms
 */

float   gyroData[3] = { 0, 0, 0 };
float   angle[2]    = { 0, 0 };                                            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static  uint8_t Smoothing[3]  = { 0, 0, 0 };
static  int16_t gyroSmooth[3] = { 0, 0, 0 };

static void getEstimatedAttitude(void);

void imuInit(void)                                                         // Initialize & precalculate some values here
{
    INV_GYR_CMPF_FACTOR  = (1.0f / ((float)cfg.gyro_cmpf_factor  + 1.0f)); // Default 400
    INV_GYR_CMPFM_FACTOR = (1.0f / ((float)cfg.gyro_cmpfm_factor + 1.0f)); // Default 200
    if (cfg.acc_ins_lpf == 0)    cfg.acc_ins_lpf    = 1;                   // Just to be safe
    if (cfg.acc_lpf_factor == 0) cfg.acc_lpf_factor = 1;                   // Just to be safe
    INV_ACC_INS_LPF = 1.0f / (float)cfg.acc_ins_lpf;
    INV_ACC_LPF     = 1.0f / (float)cfg.acc_lpf_factor;
    accADC[0] = accADC[1] = accADC[2] = 0;

    if (feature(FEATURE_GYRO_SMOOTHING))                                   // initialize if needed
    {
        Smoothing[ROLL]  = (cfg.gyro_smoothing_factor >> 16) & 0xff;
        Smoothing[PITCH] = (cfg.gyro_smoothing_factor >> 8) & 0xff;
        Smoothing[YAW]   = (cfg.gyro_smoothing_factor) & 0xff;
    }

#ifdef MAG
    if (sensors(SENSOR_MAG)) Mag_init();
#endif
}

void computeIMU(void)
{
    static int16_t  gyroYawSmooth  = 0;
    uint8_t axis;

    if(MpuSpecial)
    {
        GETMPU6050();
        getEstimatedAttitude();
    }
    else
    {
        if (sensors(SENSOR_ACC))
        {
            ACC_getADC();
            getEstimatedAttitude();
        }
        Gyro_getADC();
    }

    for (axis = 0; axis < 3; axis++) gyroData[axis] = gyroADC[axis];

    if (feature(FEATURE_GYRO_SMOOTHING))
    {
        for (axis = 0; axis < 3; axis++)
        {
            gyroData[axis]   = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    }
    else if (cfg.mixerConfiguration == MULTITYPE_TRI)
    {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
        gyroYawSmooth = gyroData[YAW];
    }

    annexCode();    // Left that junk here
}

#define GYRO_SCALE (((32767.0f / 16.4f) * M_PI) / ((32767.0f / 4.0f) * 180.0f * 1000000.0f))
//#define GYRO_SCALE (((32767.0f / 16.4f) * M_PI) / (32767.0f * 180.0f * 1000000.0f))

typedef struct fp_vector
{
    float X, Y, Z;
} t_fp_vector_def;

typedef union
{
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct    fp_vector v_tmp = *v;
    float     mat[3][3];                                                  // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float     cosx, sinx, cosy, siny, cosz, sinz;
    float     coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;
    cosx      = cosf(-delta[PITCH]);
    sinx      = sinf(-delta[PITCH]);
    cosy      = cosf(delta[ROLL]);
    siny      = sinf(delta[ROLL]);
    cosz      = cosf(delta[YAW]);
    sinz      = sinf(delta[YAW]);
    coszcosx  = cosz * cosx;
    coszcosy  = cosz * cosy;
    sinzcosx  = sinz * cosx;
    coszsinx  = sinx * cosz;
    sinzsinx  = sinx * sinz;
    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;
    v->X      = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y      = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z      = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static void getEstimatedAttitude(void)
{
    static t_fp_vector EstM;
    static float    accLPFINS[3];
    static uint32_t previousT;
    float           scale, deltaGyroAngle[3];
    float           rollRAD, pitchRAD, cr, sr, cp, sp, Xh, Yh;
    float           cy, sy, spcy, spsy, acc_south, acc_west, acc_up;
    float           tmp0, tmp1, tmp2, tmp3, AccMag = 0;
    uint8_t         axis;
    uint32_t        currentT = micros();
  
    tmp0            = (float)(currentT - previousT);
    scale           = tmp0 * GYRO_SCALE;               // Gyroscale adjusted for div 4 shit from mwii
    ACCDeltaTimeINS = tmp0 * 0.000001f;
    previousT       = currentT;

    tmp1 = 1.0f - INV_ACC_INS_LPF;
    tmp3 = 1.0f - INV_ACC_LPF;
    for (axis = 0; axis < 3; axis++)
    {
        deltaGyroAngle[axis] = gyroADC[axis]   * scale;
        accLPFINS[axis]      = accLPFINS[axis] * tmp1 + accADC[axis] * INV_ACC_INS_LPF;
        accSmooth[axis]      = accSmooth[axis] * tmp3 + accADC[axis] * INV_ACC_LPF;
        AccMag              += accSmooth[axis] * accSmooth[axis];
    }
    AccMag = AccMag * 100 / SQacc_1G;
    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG)) rotateV(&EstM.V, deltaGyroAngle);
    if (abs(accSmooth[ROLL])  < acc_25deg &&
        abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0) f.SMALL_ANGLES_25 = 1;
    else f.SMALL_ANGLES_25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < AccMag && AccMag < 133)
    {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)cfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    if (sensors(SENSOR_MAG))
    {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)cfg.gyro_cmpfm_factor + magADCfloat[axis]) * INV_GYR_CMPFM_FACTOR; // EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + magADCfloat[axis]) * INV_GYR_CMPFM_FACTOR;
    }    

    rollRAD      = atan2f(EstG.V.X, EstG.V.Z);
    pitchRAD     = asinf(EstG.V.Y / -sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    angle[ROLL]  = constrain( rollRAD  * RADtoDEG10, -1800, 1800);
    angle[PITCH] = constrain(-pitchRAD * RADtoDEG10, -1800, 1800);
    cr           = cosf(rollRAD);
    sr           = sinf(rollRAD);
    cp           = cosf(pitchRAD);
    sp           = sinf(pitchRAD);
    TiltValue    = EstG.V.Z * INVacc_1G;                               // / acc_1G;

#ifdef MAG
    if (sensors(SENSOR_MAG) && cfg.mag_calibrated == 1)
    {
        tmp0 = EstM.A[0];                                              // CORRECT MAG TILT COMPENSATION
        tmp1 = EstM.A[1];
        tmp2 = EstM.A[2];
        Xh   = tmp1 * cp + tmp0 * sr * sp + tmp2 * cr * sp;            // Xh = EstM.A[1] * cp + EstM.A[0] * sr * sp + EstM.A[2] * cr * sp;
        Yh   = tmp0 * cr - tmp2 * sr;                                  // Yh = EstM.A[0] * cr - EstM.A[2] * sr;
        heading = constrain(atan2f(-Yh,Xh) * RADtoDEG, -180.0f, 180.0f) + magneticDeclination; // Get rad to Degree and add declination (without *10 shit)
        if (heading > 180.0f)       heading = heading - 360.0f;        // Wrap to -180 0 +180 Degree
        else if (heading < -180.0f) heading = heading + 360.0f;
    }
    else heading = 0;                                                  // if no mag or not calibrated do bodyframe below
    tmp0      = heading * RADX;                                        // Do GPS INS rotate ACC X/Y to earthframe no centrifugal comp. yet
    cy        = cosf(tmp0);
    sy        = sinf(tmp0);
    cos_yaw_x = cy;                                                    // Store for general use
    sin_yaw_y = sy;                                                    // Store for general use
    spcy      = sp * cy;
    spsy      = sp * sy;
    tmp0      = accLPFINS[0] * INVacc_1G;                              // / acc_1G Reference to Gravity before rotation
    tmp1      = accLPFINS[1] * INVacc_1G;
    tmp2      = accLPFINS[2] * INVacc_1G;
    acc_up    = ((-sp) * tmp1 + (sr * cp) * tmp0 + cp * cr * tmp2) - 1;// -1G That works good for althold
    tmp0      = accLPFINS[0];
    tmp1      = accLPFINS[1];
    tmp2      = accLPFINS[2];
    tmp3      = sqrtf(EstG.V.X * tmp0 + EstG.V.Y  * tmp1 + EstG.V.Z * tmp2); // Normalize ACCvector so the gps ins works
    if (tmp3 == 0.0f) tmp3 = 1;                                        // In that case all tmp must be zero so div by 1 is ok
    tmp0      = tmp0 / tmp3;
    tmp1      = tmp1 / tmp3;
    tmp2      = tmp2 / tmp3;
    acc_south = (cp * cy) * tmp1 + (sr * spcy - cr * sy) * tmp0 + ( sr * sy + cr * spcy) * tmp2;
    acc_west  = (cp * sy) * tmp1 + (cr * cy + sr * spsy) * tmp0 + (-sr * cy + cr * spsy) * tmp2;
    tmp3      = 980.665f  * ACCDeltaTimeINS;                           // vel factor for normalized output tmp3      = (9.80665f * (float)ACCDeltaTime) / 10000.0f;
    tmp2      = constrain(TiltValue, 0.5f, 1.0f) * tmp3;               // Empirical reduction of hightdrop in forward flight
    vario     = vario + acc_up * tmp2;                                 // Positive when moving Up
    ACC_speed[LAT] = ACC_speed[LAT] - acc_south * tmp3;                // Positive when moving North cm/sec when no MAG this is speed to the front
    ACC_speed[LON] = ACC_speed[LON] - acc_west  * tmp3;                // Positive when moving East cm/sec when no MAG this is speed to the right
#endif
}

#ifdef BARO
///////////////////////////////////////////////
//Crashpilot1000 Mod getEstimatedAltitude ACC//
///////////////////////////////////////////////

#define VarioTabsize 8
#define BaroTabsize 5
void getEstimatedAltitude(void)
{
    static uint8_t  Vidx, Bidx, Gathercnt;
    static float    BaroTab[BaroTabsize], VarioTab[VarioTabsize], LastEstAltBaro, GroundAlt, Sonarcorrector;
    static uint32_t IniTimer = 0;
    uint8_t         i;
    float           BaroClimbRate, fltmp, EstAltBaro;

    if (!GroundAltInitialized && newbaroalt)                     // Do init here, Sonar is blocked during that.
    {
      if (IniTimer == 0) IniTimer = currentTimeMS + 4000;        // 4 Secs of warmup
       else
       {
           if (currentTimeMS < IniTimer)
           {
               GroundAlt = BaroAlt;                              // Take measurement after warmup time, actually it adjusts all the time till timeout
               Gathercnt = 1;
           }
           else
           {
               GroundAlt += BaroAlt;                             // Gather 64 values now takes around 1,9 sec
               Gathercnt++;
               if (Gathercnt == 64)
               {
                   GroundAlt = GroundAlt / 64.0f;                // Get Groundalt (/64) and purge buffers below
                   for (i = 0; i < VarioTabsize; i++) VarioTab[i] = 0;
                   for (i = 0; i < BaroTabsize;  i++)  BaroTab[i] = 0;
                   EstAlt = vario  = Sonarcorrector  = 0;        // cfg.MaxAltMeter = cfg.MinAltMeter = 0;
                   GroundAltInitialized = true;                  // Initstuff done here
               }
           }
       }
    }

    if(GroundAltInitialized)
    {
        if (sensors(SENSOR_SONAR) && SonarStatus != 0)           // Only do sonar if available and everything is settled
        {
            if(SonarStatus == 1)                                 // First contact
            {
                Sonarcorrector = EstAlt + GroundAlt - (float)sonarAlt; // Calculate baro/sonar displacement on 1st contact
            }
            else                                                 // SonarStatus must be 2 here "steady contact"
            {
                if (newbaroalt)                                  // We have steady sonar contact, but we need new barovals to CF them
                    BaroAlt = (Sonarcorrector + (float)sonarAlt) * cfg.snr_cf + BaroAlt * (1 - cfg.snr_cf); // Set weight / make transition smoother
            }
        } else Sonarcorrector = 0;                               // Obsolete, but i like my variables set to 0 if state unknown

        EstAlt += vario * ACCDeltaTimeINS;
        if (newbaroalt)                                          // MS Baro Timecheck 27ms // BMP085 Timecheck 26ms debug[0] = BaroDeltaTime/1000;
        {
            BaroTab[Bidx] = BaroAlt - GroundAlt;                 // BaroAlt - GroundAlt Get EstAltBaro
            Bidx++;
            if (Bidx == BaroTabsize) Bidx = 0;
            fltmp = 0;
            for (i = 0; i < BaroTabsize; i++) fltmp += BaroTab[i];
            EstAltBaro     = fltmp / BaroTabsize;
            fltmp          = 1000000 / BaroDeltaTime;            // BaroDeltaTime in us
            VarioTab[Vidx] = (float)constrain(EstAltBaro - LastEstAltBaro, -127, 127) * fltmp;
            Vidx++;                                              // Baro Climbrate
            if (Vidx == VarioTabsize) Vidx = 0;
            LastEstAltBaro = EstAltBaro;
            fltmp = 0;
            for (i = 0; i < VarioTabsize; i++) fltmp += VarioTab[i];
            BaroClimbRate = fltmp / (float)VarioTabsize;         // BaroClimbRate in cm/sec // + is up // 27ms * 37 = 999ms
            vario  = vario  * cfg.accz_vel_cf + BaroClimbRate * (1.0f - cfg.accz_vel_cf);
            EstAlt = EstAlt * cfg.accz_alt_cf + EstAltBaro    * (1.0f - cfg.accz_alt_cf);
            if (cfg.baro_debug == 1)
            {
                debug[0] = EstAltBaro * 10;
                debug[1] = EstAlt * 10;
                debug[2] = BaroClimbRate;
                debug[3] = vario;
            }
        }
    } else EstAlt = vario = Sonarcorrector = 0;                  // Set to 0 to prevent garbage display during Init, otherwise user suspects an error
}
#endif

void getAltitudePID(void)                                        // I put this out of getEstimatedAltitude seems logical
{
    float ThrAngle;
    ThrAngle = constrain(TiltValue * 100.0f, 0, 100.0f);
    BaroP  = BaroI = BaroD = 0;                                  // Reset the Pid, create something new, or not....
    if (ThrAngle < 40 || TiltValue < 0) return;                  // Don't do BaroPID if copter too tilted
    BaroP  = (int16_t)((float)cfg.P8[PIDALT] * (AltHold - EstAlt) * 0.005f);
    BaroI  = (int16_t)((float)cfg.I8[PIDALT] * vario * 0.02f);   // That is actually a "D"
    BaroD  = (int16_t)((float)cfg.D8[PIDALT] * (100.0f - ThrAngle) * 0.04f); // That is actually the Tiltcompensation
}

/*
    tmp0      = accLPFINS[0];
    tmp1      = accLPFINS[1];
    tmp2      = accLPFINS[2];
    tmp3      = sqrtf(tmp0 * tmp0 + tmp1 * tmp1 + tmp2 * tmp2);        // Normalize ACCvector so the gps ins works
    if (tmp3 == 0.0f) tmp3 = 1;                                        // In that case all tmp must be zero so div by 1 is ok
    tmp0      = tmp0 / tmp3;
    tmp1      = tmp1 / tmp3;
    tmp2      = tmp2 / tmp3;

float fsq(float x)
{
    return x * x;
}

float InvSqrt (float x)
{
    union
    {
        int32_t i;  
        float   f; 
    } conv;

    conv.f = x; 
    conv.i = 0x5f3759df - (conv.i >> 1); 
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

    tmp3      = InvSqrt(fsq(tmp0) + fsq(tmp1) + fsq(tmp2));            // Normalize acc vector
    tmp0      = tmp0 * tmp3;
    tmp1      = tmp1 * tmp3;
    tmp2      = tmp2 * tmp3;

float applyDeadbandFloat(float value, float deadband)
{
    if (abs(value) < deadband) value = 0;
     else if (value > 0) value = value - deadband;
           else if (value < 0) value = value + deadband;
    return value;
}

/////// GPS INS TESTCODE
//  Testcode
    static uint32_t previous5HzT = 0;
		flthead = 0;                                                // if no mag do bodyframe below
//  Testcode
    int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
		float  knobbi= (float)knob * 0.001f;
		debug[0] = knobbi * 1000;
	  if (currentT > previous5HzT + 200000){
        previous5HzT = currentT;
		    VelNorth     = VelNorth * knobbi;
        VelEast      = VelEast  * knobbi;
		}
		debug[1] = VelNorth;
		debug[2] = VelEast;
/////// GPS INS TESTCODE
*/
