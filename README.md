# Sensor Zoo

Open-source, pure JavaScript implementations of real-time sensor fusion algorithms for mobile and embedded devices. Currently three categories are supported: orientation (AHRS) and related quantities such as gravity vector and user acceleration; step counting; and compass heading.

## Why

Most mobile operating systems provide built-in sensor fusion (gravity, orientation, step counting), but these are black boxes: the algorithms are undocumented, vary across vendors and OS versions, and may use private APIs or hardware unavailable to third-party code. This makes results non-reproducible and impossible to compare across devices.

Sensor Zoo provides a transparent, cross-platform alternative. The goal is not to beat proprietary implementations, but to offer open algorithms whose behaviour is fully inspectable, consistent across iOS and Android, and reproducible by anyone. These sensor fusion algorithms are optionally available in [Sensor Logger](https://sensorlogger.app), and they work hand-in-hand with Sensor Logger's [standardisation feature](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/CROSSPLATFORM.md): iOS and Android use opposite sign conventions for sensor axes (inertial vs accelerating force) and different units, so Sensor Logger can normalise both to a common right-handed coordinate system, ensuring the same filter code produces consistent results on any device.

## Orientation Filters

All filters assume sensor data sampled at 100 Hz with [standardisation](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/CROSSPLATFORM.md) turned on.

All orientation filters share the same output: quaternion (`qw, qx, qy, qz`), Euler angles (`roll, pitch, yaw`), gravity vector, and user acceleration (gravity removed). Each filter lazy-initialises from the first accelerometer reading and includes NaN guards that return the last known state if any input is invalid.

### Complementary Filter

A lightweight filter that blends gyroscope integration (prediction) with accelerometer-derived tilt correction. A single `alpha` parameter (0–1) controls the gyro/accel trust balance: each update rotates the estimated gravity direction a fraction `(1 - alpha)` of the way toward the measured one, correcting roll and pitch while leaving yaw purely gyro-integrated. Does not support magnetometer input, so yaw is relative to initialisation and drifts with gyro bias.

- `alpha`: gyro/accel trust balance (higher = trust gyro more)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt);
```

### Madgwick Filter

A gradient-descent AHRS algorithm that minimises an objective function measuring the error between the estimated and measured direction of gravity (and optionally the magnetic field). Magnetometer readings outside 10–200 µT are rejected as outliers.

- `beta`: gradient-descent step size, equivalent to estimated mean gyro error in rad/s
- `betaMag`: magnetometer gradient step size (set to 0 to disable)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

### Mahony Filter

A PI-controller (proportional-integral) AHRS algorithm that computes the cross-product error between measured and estimated gravity, then feeds it through a PI loop to correct the gyroscope in real time. The integral term continuously estimates and removes gyroscope bias. Optional magnetometer support with the same 10–200 µT outlier bounds; magnetometer error is projected onto the gravity axis to isolate yaw correction and prevent heading errors from destabilising the tilt estimate, and is excluded from the integral term so that magnetic disturbance cannot corrupt the gyroscope bias estimate.

- `kP`: proportional gain (how aggressively the filter corrects toward the accelerometer)
- `kI`: integral gain (how quickly the filter learns and removes gyroscope bias)
- `kPMag`: magnetometer gain (defaults to kP if not specified)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

### Extended Kalman Filter (EKF)

An error-state EKF with a 6-state vector (3-axis attitude error + 3-axis gyroscope bias). Predicts via bias-corrected gyro integration, then corrects using accelerometer and (optionally) magnetometer measurements. Features adaptive accelerometer noise scaling during motion, magnetometer outlier rejection (10–200 µT), and Joseph-form covariance updates for numerical stability.

- `processNoise`: gyroscope process noise standard deviation
- `accelNoise`: accelerometer measurement noise standard deviation
- `magNoise`: magnetometer measurement noise standard deviation

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

## Step Counter

### Adaptive Step Counter

Detects footsteps from accelerometer data using an adaptive threshold that tracks the min/max acceleration magnitude over a sliding window. The threshold is recomputed every frame as the midpoint of the window's range, automatically adapting to different phone placements and walking speeds. A cooldown period prevents double-counting.

- `windowSize`: sliding window duration in seconds
- `cooldown`: minimum time between steps in seconds
- `minAmplitude`: minimum peak-to-trough swing (m/s²) to count as a step

```js
update(accelX, accelY, accelZ, dt); // returns { steps }
```

### Windowed Peak Detection

Detects footsteps by finding local maxima (peaks) in low-pass-filtered acceleration magnitude within a sliding window. The algorithm maintains a ring buffer of smoothed acceleration values and checks whether the center sample exceeds all other samples in the window. Peaks are validated by minimum prominence (peak value minus window minimum must exceed a threshold) and a cooldown timer to prevent double-counting. Based on the peak-detection approach described in Brajdic & Harle (UbiComp 2013).

- `windowSize`: peak detection window duration in seconds (how many samples around the center to check)
- `cooldown`: minimum time between steps in seconds
- `minProminence`: minimum peak-minus-trough swing (m/s²) to count as a step

```js
update(accelX, accelY, accelZ, dt); // returns { steps }
```

## Compass

### Tilt-Compensated Compass

Computes magnetic bearing from raw magnetometer data, with optional tilt compensation using a gravity vector (typically from an AHRS filter). Gravity and the magnetic field define an east/north basis, and the heading is read off whichever device axis is closest to horizontal: the camera direction (−z) when the phone is upright, in portrait *or* landscape, and the top of the phone (+y) as the camera axis approaches vertical (phone lying flat). The two are weighted by `1 - uz²` rather than averaged, since they are 90° apart in azimuth whenever both are horizontal. Falls back to a raw XY heading only when no usable gravity vector is supplied or the magnetic field is nearly parallel to gravity. Applies EMA smoothing with circular wrap-around handling.

- `smoothing`: EMA coefficient (0–1). Lower values give smoother but slower-responding headings; `1.0` disables smoothing entirely

```js
update(magX, magY, magZ, gravityX?, gravityY?, gravityZ?, dt) // returns { magneticBearing }
```

## Quaternion Conventions

- Quaternions are stored as `[w, x, y, z]` arrays.
- The gravity vector follows Apple CoreMotion conventions: `calculateGravityAppleConvention(q)` returns gravity in the device frame by rotating `[0, 0, g]` through the conjugate of `q`.
- Euler angles follow the intrinsic ZYX convention: `{ roll, pitch, yaw }`.

## Usage

### Via Sensor Logger

Since [Sensor Logger](https://sensorlogger.app) 1.55, Sensor Zoo is built in.

### Via Code

```js
import {
  MadgwickFilter,
  AdaptiveStepCounter,
  TiltCompensatedCompass,
} from "sensor-zoo";

// Orientation
const ahrs = new MadgwickFilter(0.05, 0.3);
const result = ahrs.update(ax, ay, az, gx, gy, gz, dt, mx, my, mz);
console.log(result.roll, result.pitch, result.yaw);
console.log(result.gravityX, result.gravityY, result.gravityZ);

// Step counting (adaptive threshold)
const stepper = new AdaptiveStepCounter(2.0, 0.35, 2.0);
const { steps } = stepper.update(ax, ay, az, dt);

// Step counting (windowed peak detection)
import { WindowedPeakStepCounter } from "sensor-zoo";
const peakStepper = new WindowedPeakStepCounter(0.6, 0.3, 1.5);
const { steps: peakSteps } = peakStepper.update(ax, ay, az, dt);

// Compass (using gravity from AHRS)
const compass = new TiltCompensatedCompass(0.15);
const { magneticBearing } = compass.update(
  mx,
  my,
  mz,
  result.gravityX,
  result.gravityY,
  result.gravityZ,
  dt,
);
```

## Running on Your Own Data

The included `run.js` script processes a [Sensor Logger](https://sensorlogger.app) recording exported with the Zipped CSV option through any filter and outputs CSV. Requires only Node.js, no dependencies.

### 1. Record

Install [Sensor Logger](https://sensorlogger.app) on your phone. Enable **Standardise Units & Frames** and **Uncalibrated Values**, then enable the sensors you need at 100 Hz:

- **Orientation filters** need Accelerometer + Gyroscope (+ optionally Magnetometer)
- **Step counter** needs Accelerometer
- **Compass** needs Magnetometer (+ optionally Gravity for tilt compensation)

Standardisation ensures consistent axis conventions across iOS and Android. Uncalibrated gives raw accelerometer data without OS filtering.

### 2. Export

In Sensor Logger, export your recording in Zipped CSV format. The app produces a `.zip` file. Unzip it and you'll get a directory containing one CSV per sensor:

```
my-recording/
  Accelerometer.csv
  Gyroscope.csv
  Magnetometer.csv
  Gravity.csv
  ...
```

### 3. Run

Point the script at the unzipped directory. Output goes to stdout, so pipe to a file with `>`.

```bash
node run.js my-recording/                                    # default: madgwick
node run.js my-recording/ --filter ekf > orientation.csv     # save to file
node run.js my-recording/ --filter ekf --no-mag              # disable magnetometer
node run.js my-recording/ --filter adaptiveThreshold         # step counting
node run.js my-recording/ --filter tiltCompensated           # compass bearing
```

Available filters: `madgwick`, `mahony`, `ekf`, `complementary`, `adaptiveThreshold`, `windowedPeak`, `tiltCompensated`.

### Output columns

- **Orientation** (`madgwick`, `mahony`, `ekf`, `complementary`): `time, qw, qx, qy, qz, roll, pitch, yaw, gravityX, gravityY, gravityZ, userAccelX, userAccelY, userAccelZ`. Euler angles in radians (aerospace intrinsic ZYX), gravity and user acceleration in m/s²
- **Step counter** (`adaptiveThreshold`, `windowedPeak`): `time, steps`, the cumulative step count
- **Compass** (`tiltCompensated`): `time, magneticBearing`, heading in degrees (0–360)

The script prefers `AccelerometerUncalibrated.csv` over `Accelerometer.csv` when available (uncalibrated data hasn't been filtered by the OS). A summary line with sample count and duration is printed to stderr.

## Evaluation Results

All algorithms are benchmarked against public research datasets (BROAD, Tyrex, Clemson, OxWalk, IPIN indoor localization competitions) and real device recordings, covering orientation accuracy against optical motion capture, magnetic disturbance robustness, step counting, compass heading, and end-to-end pedestrian dead reckoning.

**Explore the results interactively at [sensorlogger.app/zoo](https://sensorlogger.app/zoo)**

## Citation

If you use Sensor Zoo in your research, please cite:

```bibtex
@software{choi2025sensorzoo,
  author    = {Kelvin, Tsz Hei Choi},
  title     = {Sensor Zoo: Open-Source Sensor Fusion Algorithms},
  year      = {2025},
  url       = {https://github.com/tszheichoi/sensor-zoo},
  note      = {ORCID: 0000-0002-5796-5263}
}
```
