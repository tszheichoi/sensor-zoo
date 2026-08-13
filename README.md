# Sensor Zoo

Open-source, pure JavaScript implementations of real-time sensor fusion algorithms for mobile and embedded devices. Currently three categories are supported: orientation (AHRS) and related quantities such as gravity vector and user acceleration; step counting; and compass heading.

## Why

Most mobile operating systems provide built-in sensor fusion (gravity, orientation, step counting), but these are black boxes — the algorithms are undocumented, vary across vendors and OS versions, and may use private APIs or hardware unavailable to third-party code. This makes results non-reproducible and impossible to compare across devices.

Sensor Zoo provides a transparent, cross-platform alternative. The goal is not to beat proprietary implementations, but to offer open algorithms whose behaviour is fully inspectable, consistent across iOS and Android, and reproducible by anyone. These sensor fusion algorithms are optionally available in [Sensor Logger](https://sensorlogger.app), and they work hand-in-hand with Sensor Logger's [standardisation feature](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/CROSSPLATFORM.md) — iOS and Android use opposite sign conventions for sensor axes (inertial vs accelerating force) and different units, so Sensor Logger can normalise both to a common right-handed coordinate system, ensuring the same filter code produces consistent results on any device.

## Orientation Filters

All filters assume sensor data sampled at 100 Hz with [standardisation](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/CROSSPLATFORM.md) turned on.

All orientation filters share the same output: quaternion (`qw, qx, qy, qz`), Euler angles (`roll, pitch, yaw`), gravity vector, and user acceleration (gravity removed). Each filter lazy-initialises from the first accelerometer reading and includes NaN guards that return the last known state if any input is invalid.

### Complementary Filter

A lightweight filter that blends gyroscope integration (prediction) with accelerometer-derived tilt correction. A single `alpha` parameter (0–1) controls the gyro/accel trust balance: each update rotates the estimated gravity direction a fraction `(1 - alpha)` of the way toward the measured one, correcting roll and pitch while leaving yaw purely gyro-integrated. Does not support magnetometer input, so yaw is relative to initialisation and drifts with gyro bias.

- `alpha` — gyro/accel trust balance (higher = trust gyro more)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt);
```

### Madgwick Filter

A gradient-descent AHRS algorithm that minimises an objective function measuring the error between the estimated and measured direction of gravity (and optionally the magnetic field). Magnetometer readings outside 10–200 µT are rejected as outliers.

- `beta` — gradient-descent step size, equivalent to estimated mean gyro error in rad/s
- `betaMag` — magnetometer gradient step size (set to 0 to disable)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

### Mahony Filter

A PI-controller (proportional-integral) AHRS algorithm that computes the cross-product error between measured and estimated gravity, then feeds it through a PI loop to correct the gyroscope in real time. The integral term continuously estimates and removes gyroscope bias. Optional magnetometer support with the same 10–200 µT outlier bounds; magnetometer error is projected onto the gravity axis to isolate yaw correction and prevent heading errors from destabilising the tilt estimate, and is excluded from the integral term so that magnetic disturbance cannot corrupt the gyroscope bias estimate.

- `kP` — proportional gain (how aggressively the filter corrects toward the accelerometer)
- `kI` — integral gain (how quickly the filter learns and removes gyroscope bias)
- `kPMag` — magnetometer gain (defaults to kP if not specified)

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

### Extended Kalman Filter (EKF)

An error-state EKF with a 6-state vector (3-axis attitude error + 3-axis gyroscope bias). Predicts via bias-corrected gyro integration, then corrects using accelerometer and (optionally) magnetometer measurements. Features adaptive accelerometer noise scaling during motion, magnetometer outlier rejection (10–200 µT), and Joseph-form covariance updates for numerical stability.

- `processNoise` — gyroscope process noise standard deviation
- `accelNoise` — accelerometer measurement noise standard deviation
- `magNoise` — magnetometer measurement noise standard deviation

```js
update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX?, magY?, magZ?)
```

## Step Counter

### Adaptive Step Counter

Detects footsteps from accelerometer data using an adaptive threshold that tracks the min/max acceleration magnitude over a sliding window. The threshold is recomputed every frame as the midpoint of the window's range, automatically adapting to different phone placements and walking speeds. A cooldown period prevents double-counting.

- `windowSize` — sliding window duration in seconds
- `cooldown` — minimum time between steps in seconds
- `minAmplitude` — minimum peak-to-trough swing (m/s²) to count as a step

```js
update(accelX, accelY, accelZ, dt); // returns { steps }
```

### Windowed Peak Detection

Detects footsteps by finding local maxima (peaks) in low-pass-filtered acceleration magnitude within a sliding window. The algorithm maintains a ring buffer of smoothed acceleration values and checks whether the center sample exceeds all other samples in the window. Peaks are validated by minimum prominence (peak value minus window minimum must exceed a threshold) and a cooldown timer to prevent double-counting. Based on the peak-detection approach described in Brajdic & Harle (UbiComp 2013).

- `windowSize` — peak detection window duration in seconds (how many samples around the center to check)
- `cooldown` — minimum time between steps in seconds
- `minProminence` — minimum peak-minus-trough swing (m/s²) to count as a step

```js
update(accelX, accelY, accelZ, dt); // returns { steps }
```

## Compass

### Tilt-Compensated Compass

Computes magnetic bearing from raw magnetometer data, with optional tilt compensation using a gravity vector (typically from an AHRS filter). Gravity and the magnetic field define an east/north basis, and the heading is read off whichever device axis is closest to horizontal: the camera direction (−z) when the phone is upright, in portrait *or* landscape, and the top of the phone (+y) as the camera axis approaches vertical (phone lying flat). The two are weighted by `1 - uz²` rather than averaged, since they are 90° apart in azimuth whenever both are horizontal. Falls back to a raw XY heading only when no usable gravity vector is supplied or the magnetic field is nearly parallel to gravity. Applies EMA smoothing with circular wrap-around handling.

- `smoothing` — EMA coefficient (0–1). Lower values give smoother but slower-responding headings; `1.0` disables smoothing entirely

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

The included `run.js` script processes a [Sensor Logger](https://sensorlogger.app) recording exported with the Zipped CSV option through any filter and outputs CSV. Requires only Node.js — no dependencies.

### 1. Record

Install [Sensor Logger](https://sensorlogger.app) on your phone. Enable **Standardise Units & Frames** and **Uncalibrated Values**, then enable the sensors you need at 100 Hz:

- **Orientation filters** need Accelerometer + Gyroscope (+ optionally Magnetometer)
- **Step counter** needs Accelerometer
- **Compass** needs Magnetometer (+ optionally Gravity for tilt compensation)

Standardisation ensures consistent axis conventions across iOS and Android. Uncalibrated gives raw accelerometer data without OS filtering.

### 2. Export

In Sensor Logger, export your recording in Zipped CSV format. The app produces a `.zip` file. Unzip it — you'll get a directory containing one CSV per sensor:

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

- **Orientation** (`madgwick`, `mahony`, `ekf`, `complementary`): `time, qw, qx, qy, qz, roll, pitch, yaw, gravityX, gravityY, gravityZ, userAccelX, userAccelY, userAccelZ` — Euler angles in radians (aerospace intrinsic ZYX), gravity and user acceleration in m/s²
- **Step counter** (`adaptiveThreshold`, `windowedPeak`): `time, steps` — cumulative step count
- **Compass** (`tiltCompensated`): `time, magneticBearing` — heading in degrees (0–360)

The script prefers `AccelerometerUncalibrated.csv` over `Accelerometer.csv` when available (uncalibrated data hasn't been filtered by the OS). A summary line with sample count and duration is printed to stderr.

## Evaluation Results

All filters are validated against public research datasets and real device recordings. Summary results are also available in [`eval.json`](./eval.json).

The Complementary filter is not included in the orientation benchmarks: it takes no magnetometer input, so its yaw is relative to initialisation and cannot be scored against the absolute heading in the ground truth.

### Orientation — BROAD Dataset

Tested on the [BROAD](https://github.com/dlaidig/broad) dataset (39 trials, 286 Hz downsampled to 100 Hz, 9-axis IMU with optical motion capture ground truth). Metrics are median angular RMSE in degrees during movement phases.

| Filter                                | With Magnetometer | Without Magnetometer |
| ------------------------------------- | :---------------: | :------------------: |
| **Madgwick** (beta=0.05, betaMag=0.3) |       4.4°        |        16.9°         |
| **Mahony** (kP=0.7, kI=0.01)          |       12.2°       |        22.4°         |
| **EKF** (q=0.01, a=0.3, m=1.0)        |     **3.9°**      |        82.5°         |

Under magnetic disturbances (nearby magnets, office environments):

| Filter       | With Magnetometer | Without Magnetometer |
| ------------ | :---------------: | :------------------: |
| **Madgwick** |     **6.3°**      |      **10.8°**       |
| **Mahony**   |       8.0°        |        23.3°         |
| **EKF**      |       9.2°        |        99.3°         |

The EKF's `magNoise` default of 1.0 was validated against alternatives: raising it to 3.0 improves roll/pitch similarity to the OS gravity sensor by 1–2° on real devices, but costs the EKF ~20° of ground-truth attitude accuracy on BROAD — including on the magnetically disturbed trials — because the weaker magnetometer correction lets yaw drift toward the no-magnetometer regime.

### Orientation — Real Devices

Gravity vector RMSE (degrees) vs system sensor, measured during 98-minute city walks on 5 devices. First 5 seconds excluded for filter convergence.

| Device        | Madgwick Roll/Pitch | Mahony Roll/Pitch | EKF Roll/Pitch |
| ------------- | :-----------------: | :---------------: | :------------: |
| iPhone SE     |     2.9° / 2.6°     |    2.5° / 1.9°    |  3.2° / 1.8°   |
| iPhone 17 Pro |    12.0° / 2.4°     |    7.4° / 1.4°    |  8.7° / 1.3°   |
| Samsung S21   |    14.0° / 2.7°     |   10.8° / 1.2°    |  11.1° / 1.6°  |
| Pixel 8       |     3.9° / 3.1°     |    2.3° / 3.2°    |  2.3° / 2.4°   |
| Samsung S25   |    17.0° / 2.4°     |    5.7° / 1.3°    |  8.7° / 1.7°   |

### Compass — BROAD Dataset

Tilt-compensated compass (smoothing=0.15) heading RMSE, using Madgwick-derived gravity for tilt compensation:

| Trial Type                        | Median Heading RMSE |
| --------------------------------- | :-----------------: |
| Translation (slow + fast)         |      **5.1°**       |
| Rotation                          |        76.2°        |
| Combined (rotation + translation) |        87.5°        |
| Magnetically disturbed            |        72.5°        |

Compass heading is most accurate during translation-only motion. Pure rotation and magnetic disturbances cause large heading errors, which is expected — the compass does not track gyroscope-integrated yaw.

### Step Counter — Clemson Dataset

Both step counters tested on the [Clemson Pedometer](http://cecas.clemson.edu/~ahoover/pedometer/) dataset (30 subjects, 15 Hz). Mean absolute error percentage:

**Adaptive Threshold** (windowSize=2.0, cooldown=0.35, minAmplitude=2.0):

| Gait Type    | Wrist |   Hip    | Ankle |
| ------------ | :---: | :------: | :---: |
| Regular      | 29.7% | **5.7%** | 22.8% |
| Semi-Regular | 49.7% |  48.0%   | 34.0% |
| Irregular    | 46.8% |  56.2%   | 51.4% |

**Windowed Peak Detection** (windowSize=0.6, cooldown=0.3, minProminence=1.5):

| Gait Type    | Wrist |   Hip    |  Ankle   |
| ------------ | :---: | :------: | :------: |
| Regular      | 31.7% | **4.9%** | **3.0%** |
| Semi-Regular | 50.3% |  45.8%   |  17.6%   |
| Irregular    | 44.9% |  53.7%   |  29.3%   |

Windowed peak detection reduces overall mean absolute error from 38.3% to 31.2%, with the largest gains at the ankle (36.1% → 16.6% across all gaits). Both counters perform best at the hip during regular walking. The Clemson dataset is sampled at 15 Hz (vs the 100 Hz the counters are designed for), which partly explains the higher error rates. Semi-regular and irregular gaits are challenging across all placements.

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
