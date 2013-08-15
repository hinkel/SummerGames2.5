Naze32 Harakiri10 Summer Games2.5
=================================
- Experimental IMU Changes. Better MPU Gyro usage. All acc scaled to "512" for 1G
- Precision improved ACC/Gyro calibration.
- MPU6050 "single shot" readout like suggested by Sebbi
- Changed MPU Acc setting from 8G to 4G, increasing resolution without observing saturation effect (right now...).
- Experimental IMU Change for GPS
- gyro_cmpf_factor changed to 1000 (from 400)
- Added vario data transmission within MSP_ALTITUDE


Naze32 Harakiri10 Summer Games2.4 WITH EXPERIMENTAL LOGGING
===========================================================
- LLIGHTS & LEDMAX Deleted (Had no use in BF anyway)

// Logging don't expect anything of it yet
// - Introduced some "GPS LOG Box" will record when armed, only one log possible. So restarting logging will delete old log.
// - Introduced gpslog in cli to show dataset(s)
// - GPS Logging for 19,5 Minutes for LAT/LON/ALT/HEADING at 0.5 HZ (every 2 sec new dataset)
// - Logging precision is slightly decreased for compression reasons, one Dataset needs just 4 Bytes, so 2,3KB can store 20Min flight
// - Flightstats couldn't display negative alt, basic stats can be saved now. When doing a general save on eeprom (like save in cli or write in gui, or if logging anyway)
// - Flightstats will be cleared at power up with the default stat_clear = 1. With 0 the last saved stats will be taken into account concerning max speed/hight etc..

Core changes:
- Reintroduced old, (t)rusty MTK parser.
- Removed the fix acc_1G value from multiwii and calculate real acc_1G during calibration. So you will not see that fix "512" for mpu any more. On the first run altitude will show a flatline
  until ACC calibrated (wait a little for save) and FC repowered
- Moved Gyro calibration, angle calculation to float point calculations.
- INS FACTORS WILL HAVE TO BE RETUNED - OMG. gps_ins_vel reduced to 0.6. nav_slew_rate set to 20 now (reducing, increases strength). accz_vel_cf untouched seems to fit.
- gps_tbe_speed deleted was of limited use
- PH/RTL Bug fixed (actual position could be ignored esp. on RTH)
- PosrI put to work (and scaled further down by /100) but just with the velocity error that is calculated from position error (posP). So will hopefully have effect now without circeling. Default 0.Untested.
- Stay away from the feature inflightcalibration it will probably kill INS/ACC functions because it isn't affecting the trims but the real acc calibration. - Outdated mwii code.
- Reworked MsBaroDriver a little (probably slight resolution increase)

Naze32 Harakiri10 Summer Games2.3
=================================
- Little update just concerning ublox parser. Some people reportet uBLox 6M didn't show Lat/Lon but satcount.
- So ublox parser is redone here and it is a mixture of current BF/Mwii and Arducopter driver
- I hope it resolves that issue. Works the same on my rig than the old one
- Reduced the defaultPIDs, because the correct controller is more aggressive

Naze32 Harakiri10 Summer Games2.2
=================================
- Just updated main PID controller (mainpidctrl = 1 (default)) according to BRM's suggestions here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=150#p38927
- Relaxed timings on ublox startup configuration
- Minor mavlink changes

Naze32 Harakiri10 Summer Games2.1
=================================
- Just updated the Arducopter "plain earth" bearing calculation to a little more STM like correct, spherical Bearing.
Forumla:http://www.movable-type.co.uk/scripts/latlong.html under "BEARING"
JavaScript: 	
var y = Math.sin(dLon) * Math.cos(lat2);
var x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
var brng = Math.atan2(y, x).toDeg();
I don't understand that but RTL works to the point now :)

Harakiri Summergames2
=====================

- Feature telemetry is gone and replaced by "tele_prot = X"
- Some basic Mavlinksupport. It's beta but there are a few things you can do already.
- Autosensing Mavlink/Multiwii protocols (can change every 4 seconds)
- Due to Autosensing you will see the Mavlink "1Hz Heartbeat - garbage" in cli.
- Entering the CLI requires three times "#" now! ("###"). Alternatively 3 times <RETURN>. CLI entering is only possible when disarmed.
- Flashing requires three times "R" now! ("RRR")
- Reworked PH PosrD Controller. PosrD is scaled down now (factor 30)
- Re-enabled crosstrack support. Crosstrack gain can be adjusted or turned off.


Telemetry / Mavlink
===================
Since "feature telemetry" was just for FRsky it was misleading. Some people might do multiwii/bt telemetry and are mislead by that.
Now you can "set tele_prot = X".
That X will decide, what is done on the usb/telem. port ONLY WHEN ARMED. When disarmed the mavlink/multiwii autodetection will kick in.
So here is what that X stands for:
0 (Dfault) = Keep Multiwii @CurrentUSB Baud (like always)
1          = Frsky @9600Baud Turn to Frsky protocol at 9600 Baud when armed.
2          = Mavlink @CurrentUSB Baud. Some Mavlink 1.0 Packages are sent at low rate (ca <2Kb/s)
3          = Mavlink @57KBaud (like stock minimOSD wants it) Sends the same stuff like in mode "2" but only at 57K

NOTE: If you choose to work just with missionplaner and also want to feed minimosd you might want to set serial_baudrate = 57600 and set
set tele_prot to 2 or 3. So you will not loose MP connection on arming. Or recompile MinimOSD FW with 115KBaud setting.

Current Datastream over mavlink:
Heartbeat      @ 1  Hz: Armed/Disarmed, Copter Type, Stabilized/Acro.
Sys_status     @ 2  Hz: Sensors present and healthstatus. Voltage.
Attitude       @30  Hz: TimeStamp(ms), roll/pitch/yaw(compass heading - RAD)
RC_Channels    @ 2  Hz: RAW, unscaled 8 RC Channels and RSSI (currently unknown to naze)
VFR_HUD        @10  Hz: Speed measured by GPS, scaled Throttle (0-100%), Baro Altitude, Variometer, compass heading (again, this time in Degree)
Scaled_pressure@ 0.5Hz: Gyro Temperature, Airpressure in hPa (mBar)

Note: The Datarates are wishful thinking, because only one Datapacket is send per (100Hz-)cycle to keep the serial rate low. So "Attitude"
is actually send at 25.xHz rate. The real rates are "as high" like the APM sends them. I checked it with GCS. Attitude is 10Hz at Arducopter, so that's faster now.
These Data are send without request, once Mavlink is established. MinimOSD is not tested but should be able to read and display something from the datastream.
Datastreamrequests are currently not handled (with one exception) so for testing you can connect minimOSD only with its' RX pin (same like in arducopter telem mode).
Let me know if minimOSD works with that (set tele_prot = 3).
The only request that is handled is, when Missionplaner requests Parameterlist (won't work with the windows GCS soft I tested).
Missionplaner will load the Parameterlist on start. It will be accessible as advantaged parameter list. You can change and write and compare whole parametersets.
Note: New MP will cry out for missing Arducopterparameters. The new MP doesn't show altitude and some other data on mainscreen - they changed that.
My old and rusty Mission Planner 1.2.38 shows everything and is not so picky. So go for older MP.
Note: Naze parameters will be crippled to 16 chars if necessary. No problem.
WP etc is not supported right now. The main reason for that is that i am having trouble with Naze EEPROM writing more than 1KB data.


Changes in PH
=============
Parameters used:
PosrP (Strength of velocity influence, tries to keep velocity, normally "0" in PH, but might change with PosP - see below)
PosrI (keep it 0, might lead to circeling)
PosrD is rescaled now.

PosP Works in conjunction with PosrP. And defines how much a POSITION error is translated to an VELOCITY error.

Parameters NOT USED: PosI and PosD

gps_ph_brkacc = 40 (Dfault)  // [1 - 500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though. It will be a timeout. The lower the Value the longe the Timeout.

gps_ph_abstub = 150 (Dfault) // 0 - 1000cm (150 Dfault, 0 disables) Defines the "bath tub" around current absolute PH Position, where PosP is diminished, reaction gets harder on tubs edge and then goes on linear.
I changed the form of the bathtub -> see attached picture. The bathtub is for absolute position (influence set by PosP).

gps_tbe_speed = 0(Dfault)    // 0 - 1000 (0 disables) Speed in cm/s for TBE detection MUST be greater than gps_ph_settlespeed or it will be disabled
When at the end of the PH chain a movement with that speed (150cm/s) is detected for 2 seconds a toilet bowl is assumed and the PH cascade is redone, keeping the target PH Position in mind.
Thanks "HINKEL" for the idea!
WARNING: gps_tbe_speed Also can disable PH in wind, because if it is carried away with 150cm/s a TBE will be assumed! Set it to 0 to disable it.


Changes in NAVIGATION
=====================
The final stage of a RTH is the PH. If you move the sticks and oversteer, the PH cascade is reset and a new GPS targetpoint is set. It will not land at the once assumed GPS pos anymore thats on purpose.

nav_tiltcomp is reduced to 20  // 0 - 100 (20 TestDefault) Only arducopter really knows. Dfault was 54. This is some kind of a hack of them to reach actual nav_speed_max. 54 was Dfault, 0 disables

nav_ctrkgain = 0.5  // 0 - 10.0 (0.5 TestDefault) That is the "Crosstrackgain" APM Dfault is "1". "0" disables
Re - Introduced Crosstrack. I think it is not neccessary for copters. You can disable it with "0".
See for details: http://diydrones.com/profiles/blogs/705844:BlogPost:43438





