# ARINC Characteristic 705 Attitude and Heading Reference System Specification

## 1\. Scope & System Definition

ARINC Characteristic 705 specifies the form, fit, functional capabilities, and digital bus interfaces of the airborne Attitude and Heading Reference System (AHRS). Installed across regional transport aircraft (ATR 42/72, Dash 8, CRJ, ERJ), business jets (Citation, Gulfstream, Bombardier Challenger/Learjet), and commercial transport helicopters, the AHRS provides reliable three-axis attitude (pitch and roll), angular rates, linear accelerations, and magnetic/true heading to the Electronic Flight Instrument System (EFIS), Autopilot (ARINC 760), Weather Radar (ARINC 708), and Flight Management System (ARINC 702A).

## 2\. Sensor Architecture & Slaving Kinematics

- **Strapdown Inertial Sensors**:  
  - Triaxial MEMS or Fiber Optic Rate Gyros measuring body pitch (\$q\$), roll (\$p\$), and yaw (\$r\$) rates up to \$\\pm 128^\\circ/\\text{sec}\$.  
  - Triaxial linear quartz accelerometers measuring longitudinal (\$a\_x\$), lateral (\$a\_y\$), and normal (\$a\_z\$) accelerations.  
- **Remote Flux Valve Magnetometer Slaving**:  
  - Remotely mounted wingtip or tailcone triaxial flux valve detecting the Earth's horizontal and vertical magnetic field components.  
  - Closed-loop slaving loop with an approximate 20-second time constant (\$K\_{slave} \\approx 0.05\\text{ rad/s}\$) during straight-and-level flight.  
- **Centrifugal Acceleration Turn Compensation**:  
  - Uses True Airspeed (TAS) from the Air Data Computer and yaw rate (\$r\$) to compute and subtract centrifugal lateral acceleration: \$\$a\_{y,centrifugal} \= \\frac{V\_{TAS} \\times r}{g}\$\$  
  - Dynamic slaving cutoff: When bank angle \$\|\\phi\| \> 6.0^\\circ\$ or yaw rate \$\|r\| \> 3.0^\\circ/\\text{sec}\$, magnetic slaving is attenuated/inhibited to prevent northerly/southerly turning errors and dip distortion.  
- **Operational Modes**:  
  - `ALIGNMENT`: Rapid ground gyro leveling and leveling in under 45 seconds from cold start.  
  - `SLAVED`: Normal flight mode with flux valve heading slaving.  
  - `FREE_DG`: Directional gyro mode with low unslaved drift (\$\\le 1.5^\\circ/\\text{hour}\$) for polar operations.

## 3\. Digital ARINC 429 Bus Interface

- **Label 324**: Pitch Angle (BNR, resolution \$0.011^\\circ\$, range \$\\pm 90^\\circ\$).  
- **Label 325**: Roll Angle (BNR, resolution \$0.011^\\circ\$, range \$\\pm 180^\\circ\$).  
- **Label 320**: Magnetic Heading (BNR, resolution \$0.011^\\circ\$, range \$0^\\circ\\text{ to }360^\\circ\$).  
- **Label 314**: True Heading (BNR, resolution \$0.011^\\circ\$, range \$0^\\circ\\text{ to }360^\\circ\$).  
- **Label 270**: AHRS Discrete Mode & Status Word (Align, Slaved, Free DG, Fast Erect, Attitude Valid).  
- **Label 350**: AHRS BITE Fault Word (Gyro failure, accel failure, flux valve disconnect, power low).  
- **Parity**: Strict Bit 32 odd parity enforced on all transmitted words.