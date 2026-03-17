import { ComplementaryFilter } from "./filters/ComplementaryFilter.js";
import { MadgwickFilter } from "./filters/MadgwickFilter.js";
import { MahonyFilter } from "./filters/MahonyFilter.js";
import { EKFFilter } from "./filters/EKFFilter.js";
import { AdaptiveStepCounter } from "./steps/AdaptiveStepCounter.js";
import { WindowedPeakStepCounter } from "./steps/WindowedPeakStepCounter.js";
import { TiltCompensatedCompass } from "./compass/TiltCompensatedCompass.js";
import {
  formatSpeed,
  getStandardise,
  getUncalibrated,
  getSensorEnabled,
  getSensorSpeed,
} from "./utils/defaults.js";

export const FUSION_CATEGORIES = {
  orientation: {
    key: "zooOrientation",
    previewScreen: "Orientation Preview",
    label: "Orientation (AHRS)",
    description:
      "Attitude and heading reference using accelerometer, gyroscope, and magnetometer. Controls gravity, orientation, and user acceleration.",

    outputs: [
      {
        replaces: "Gravity",
        title: "Estimated Gravity",
        detail:
          "Gravity vector derived from the filter's orientation estimate.",
        resultMapping: { x: "gravityX", y: "gravityY", z: "gravityZ" },
      },
      {
        replaces: "Orientation",
        title: "Orientation",
        detail:
          "Quaternion and Euler angles (roll, pitch, yaw) from the filter.",
        resultMapping: {
          qw: "qw",
          qx: "qx",
          qy: "qy",
          qz: "qz",
          roll: "roll",
          pitch: "pitch",
          yaw: "yaw",
        },
        // Filters output aerospace Euler angles (roll=X, pitch=Y).
        // Standardised convention swaps axes (roll=Y, pitch=X) and negates pitch and yaw.
        transformResult: (values) => {
          const aeroRoll = values.roll;
          values.roll = values.pitch;
          values.pitch = -aeroRoll;
          values.yaw = -values.yaw;
        },
      },
      {
        replaces: "Accelerometer",
        title: "User Acceleration",
        detail: "Linear acceleration with estimated gravity removed.",
        resultMapping: { x: "userAccelX", y: "userAccelY", z: "userAccelZ" },
      },
    ],

    requirements: [
      {
        key: "standardise",
        label: "Standardisation On",
        check: (state) => getStandardise(state),
        required: "On",
      },
      {
        key: "uncalibrated",
        label: "Uncalibrated Data On",
        check: (state) => getUncalibrated(state),
        required: "On",
      },
      {
        key: "accelerometer",
        sensor: "Accelerometer",
        label: "Accelerometer Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Accelerometer"),
        required: "On",
      },
      {
        key: "gyroscope",
        sensor: "Gyroscope",
        label: "Gyroscope Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Gyroscope"),
        required: "On",
      },
      {
        key: "magnetometer",
        sensor: "Magnetometer",
        label: "Magnetometer Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Magnetometer"),
        required: "On",
      },
      {
        key: "inertialSpeed",
        label: "Inertial Sensor Sampling Rate at 100 Hz",
        check: (state) =>
          getSensorSpeed(state, "Accelerometer") === 10,
        required: "100 Hz",
        current: (state) =>
          formatSpeed(getSensorSpeed(state, "Accelerometer")),
      },
    ],

    enableRequirements: (setState) => {
      setState((state) => {
        let sensorState = { ...state.sensorState };
        ["Accelerometer", "Gyroscope", "Magnetometer"].forEach((s) => {
          sensorState[s] = { ...sensorState[s], enabled: true };
        });
        [
          "Accelerometer",
          "Gravity",
          "Gyroscope",
          "Orientation",
          "Magnetometer",
        ].forEach((s) => {
          sensorState[s] = { ...sensorState[s], speed: 10 };
        });
        return { standardise: true, uncalibrated: true, sensorState };
      });
    },

    systemDefault: {
      key: "system",
      label: "System Default",
      description:
        "Uses the device's built-in sensor fusion provided by the operating system.",
    },

    createProcessor: (filter) => {
      let lastAccel = null;
      let lastMag = null;
      let lastTime = null;
      let initialized = false;
      return {
        onData(data) {
          if (data.name === "accelerometeruncalibrated") {
            lastAccel = {
              x: data.values.x,
              y: data.values.y,
              z: data.values.z,
            };
          } else if (data.name === "accelerometer" && !lastAccel) {
            lastAccel = {
              x: data.values.x,
              y: data.values.y,
              z: data.values.z,
            };
          }
          if (data.name === "magnetometer") {
            lastMag = { x: data.values.x, y: data.values.y, z: data.values.z };
          }
          if (data.name === "gyroscope" && lastAccel) {
            if (!initialized) {
              filter.init(lastAccel.x, lastAccel.y, lastAccel.z);
              initialized = true;
              lastTime = data.time;
              return null;
            }
            const dt = lastTime != null ? (data.time - lastTime) / 1e9 : 0.01;
            lastTime = data.time;
            if (dt > 0 && dt < 1) {
              return filter.update(
                lastAccel.x,
                lastAccel.y,
                lastAccel.z,
                data.values.x,
                data.values.y,
                data.values.z,
                dt,
                lastMag?.x,
                lastMag?.y,
                lastMag?.z,
              );
            }
          }
          return null;
        },
      };
    },

    filters: {
      complementary: {
        label: "Complementary Filter",
        description:
          "Blends gyroscope integration with accelerometer correction.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
            {
              sensor: "Gyroscope",
              role: "gyroscope",
              title: "Rotation Rate",
              detail: "Device angular velocity in radians per second.",
            },
          ],
          optional: [],
          driverSensor: "Gyroscope",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) => new ComplementaryFilter(params.alpha),
        warnings: {
          orientation:
            "The complementary filter does not use the magnetometer, so yaw is relative to zero at initialisation and may not match the system's absolute yaw estimate. Roll and pitch are unaffected.",
        },
        params: [
          {
            key: "alpha",
            stateKey: "Alpha",
            label: "Alpha",
            description:
              "Controls the balance between gyroscope (higher) and accelerometer (lower) trust. Higher values produce smoother orientation but slower correction.",
            min: 0.8,
            max: 0.999,
            step: 0.001,
            decimals: 3,
            defaultValue: 0.997,
          },
        ],
      },
      madgwick: {
        label: "Madgwick Filter",
        description: "Gradient-descent with magnetometer correction.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
            {
              sensor: "Gyroscope",
              role: "gyroscope",
              title: "Rotation Rate",
              detail: "Device angular velocity in radians per second.",
            },
          ],
          optional: [
            {
              sensor: "Magnetometer",
              role: "magnetometer",
              title: "Magnetic Field",
              detail: "Ambient magnetic field strength in microteslas.",
            },
          ],
          driverSensor: "Gyroscope",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) =>
          new MadgwickFilter(params.beta, params.betaMag),
        params: [
          {
            key: "beta",
            stateKey: "Beta",
            label: "Beta",
            description:
              "Estimated mean gyro error (rad/s). Higher values correct faster but are noisier. Lower values are smoother but drift more.",
            min: 0.001,
            max: 0.2,
            step: 0.001,
            decimals: 3,
            defaultValue: 0.05,
          },
          {
            key: "betaMag",
            stateKey: "BetaMag",
            label: "Magnetometer Beta",
            description:
              "Magnetometer influence weight. Higher values use magnetometer more for yaw correction. Set to 0 to disable magnetometer.",
            min: 0.0,
            max: 5.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 0.3,
          },
        ],
      },
      mahony: {
        label: "Mahony Filter",
        description: "PI-controller that estimates gyroscope bias.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
            {
              sensor: "Gyroscope",
              role: "gyroscope",
              title: "Rotation Rate",
              detail: "Device angular velocity in radians per second.",
            },
          ],
          optional: [
            {
              sensor: "Magnetometer",
              role: "magnetometer",
              title: "Magnetic Field",
              detail: "Ambient magnetic field strength in microteslas.",
            },
          ],
          driverSensor: "Gyroscope",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) => new MahonyFilter(params.kP, params.kI, params.kPMag),
        params: [
          {
            key: "kP",
            stateKey: "KP",
            label: "Proportional Gain (kP)",
            description:
              "Proportional correction strength. Higher values correct faster but add noise.",
            min: 0.0,
            max: 5.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 0.7,
          },
          {
            key: "kI",
            stateKey: "KI",
            label: "Integral Gain (kI)",
            description:
              "Gyroscope bias estimation rate. Higher values remove drift faster but may cause yaw wander during dynamic motion.",
            min: 0.0,
            max: 0.1,
            step: 0.005,
            decimals: 3,
            defaultValue: 0.01,
          },
          {
            key: "kPMag",
            stateKey: "KPMag",
            label: "Magnetometer Gain (kPMag)",
            description:
              "Magnetometer correction strength for yaw. Higher values correct heading faster but may add noise. Defaults to kP if not set.",
            min: 0.0,
            max: 5.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 0.7,
          },
        ],
      },
      ekf: {
        label: "Extended Kalman Filter",
        description:
          "Error-state EKF with adaptive gains, gyro bias estimation, and magnetometer fusion.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
            {
              sensor: "Gyroscope",
              role: "gyroscope",
              title: "Rotation Rate",
              detail: "Device angular velocity in radians per second.",
            },
          ],
          optional: [
            {
              sensor: "Magnetometer",
              role: "magnetometer",
              title: "Magnetic Field",
              detail: "Ambient magnetic field strength in microteslas.",
            },
          ],
          driverSensor: "Gyroscope",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) =>
          new EKFFilter(
            params.processNoise,
            params.accelNoise,
            params.magNoise,
          ),
        params: [
          {
            key: "processNoise",
            stateKey: "ProcessNoise",
            label: "Process Noise",
            description:
              "Gyroscope noise level. Higher = trust gyro less, correct faster.",
            min: 0.001,
            max: 1.0,
            step: 0.001,
            decimals: 3,
            defaultValue: 0.01,
          },
          {
            key: "accelNoise",
            stateKey: "AccelNoise",
            label: "Accel Noise",
            description:
              "Accelerometer measurement noise. Lower = trust accel more for tilt.",
            min: 0.01,
            max: 10.0,
            step: 0.01,
            decimals: 2,
            defaultValue: 0.3,
          },
          {
            key: "magNoise",
            stateKey: "MagNoise",
            label: "Mag Noise",
            description:
              "Magnetometer measurement noise. Lower = trust mag more for heading.",
            min: 0.1,
            max: 50.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 1.0,
          },
        ],
      },
    },
  },

  stepCounter: {
    key: "zooStepCounter",
    label: "Step Counter",
    description: "Detects steps from accelerometer data.",

    outputs: [
      {
        replaces: "Pedometer",
        title: "Step Count",
        detail: "Cumulative step count detected by the filter.",
        resultMapping: { steps: "steps" },
      },
    ],

    requirements: [
      {
        key: "pedometer",
        sensor: "Pedometer",
        label: "Pedometer Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Pedometer"),
        required: "On",
      },
      {
        key: "accelerometer",
        sensor: "Accelerometer",
        label: "Accelerometer Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Accelerometer"),
        required: "On",
      },
      {
        key: "accelSpeed",
        label: "Accelerometer Sampling Rate at 100 Hz",
        check: (state) =>
          getSensorSpeed(state, "Accelerometer") === 10,
        required: "100 Hz",
        current: (state) =>
          formatSpeed(getSensorSpeed(state, "Accelerometer")),
      },
    ],

    enableRequirements: (setState) => {
      setState((state) => {
        let sensorState = { ...state.sensorState };
        sensorState["Pedometer"] = {
          ...sensorState["Pedometer"],
          enabled: true,
        };
        sensorState["Accelerometer"] = {
          ...sensorState["Accelerometer"],
          enabled: true,
          speed: 10,
        };
        return { sensorState };
      });
    },

    systemDefault: {
      key: "system",
      label: "System Pedometer",
      description: "Uses the device's built-in step counter.",
    },

    createProcessor: (filter) => {
      let lastTime = null;
      let initialized = false;
      let hasUncalibrated = false;
      return {
        onData(data) {
          if (data.name === "accelerometeruncalibrated") {
            hasUncalibrated = true;
          } else if (data.name === "accelerometer") {
            if (hasUncalibrated) return null;
          } else {
            return null;
          }
          if (!initialized) {
            filter.init();
            initialized = true;
            lastTime = data.time;
            return null;
          }
          const dt = lastTime != null ? (data.time - lastTime) / 1e9 : 0.01;
          lastTime = data.time;
          if (dt > 0 && dt < 1) {
            return filter.update(
              data.values.x,
              data.values.y,
              data.values.z,
              dt,
            );
          }
          return null;
        },
      };
    },

    filters: {
      adaptiveThreshold: {
        label: "Adaptive Threshold",
        description:
          "Tracks running min/max over a sliding window and sets the threshold at the midpoint. Adapts to different phone placements automatically.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
          ],
          optional: [],
          driverSensor: "AccelerometerUncalibrated",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) =>
          new AdaptiveStepCounter(
            params.windowSize,
            params.cooldown,
            params.minAmplitude,
          ),
        params: [
          {
            key: "windowSize",
            stateKey: "WindowSize",
            label: "Window Size",
            description:
              "Duration in seconds of the sliding window used to track min/max amplitude. Longer windows are more stable but slower to adapt.",
            min: 1.0,
            max: 5.0,
            step: 0.5,
            decimals: 1,
            defaultValue: 2.0,
          },
          {
            key: "cooldown",
            stateKey: "AdaptiveCooldown",
            label: "Cooldown",
            description:
              "Minimum seconds between detected steps to prevent double-counting.",
            min: 0.15,
            max: 1.0,
            step: 0.05,
            decimals: 2,
            defaultValue: 0.35,
          },
          {
            key: "minAmplitude",
            stateKey: "MinAmplitude",
            label: "Min Amplitude",
            description:
              "Minimum peak-to-valley swing required to register steps. Rejects low-motion periods like standing still.",
            min: 0.2,
            max: 3.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 2.0,
          },
        ],
      },
      windowedPeak: {
        label: "Windowed Peak Detection",
        description:
          "Detects peaks in smoothed acceleration magnitude within a sliding window. More tolerant of varying gait patterns.",
        inputs: {
          required: [
            {
              sensor: "AccelerometerUncalibrated",
              role: "accelerometer",
              title: "Total Acceleration",
              detail:
                "Raw accelerometer including gravity, without iOS bias correction.",
            },
          ],
          optional: [],
          driverSensor: "AccelerometerUncalibrated",
          fallbacks: { AccelerometerUncalibrated: "Accelerometer" },
        },
        createFilter: (params) =>
          new WindowedPeakStepCounter(
            params.windowSize,
            params.cooldown,
            params.minProminence,
          ),
        params: [
          {
            key: "windowSize",
            stateKey: "WindowSize",
            label: "Window Size",
            description:
              "Duration in seconds of the sliding window used to detect peaks. Controls how many samples around the center to check.",
            min: 0.2,
            max: 2.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 0.6,
          },
          {
            key: "cooldown",
            stateKey: "PeakCooldown",
            label: "Cooldown",
            description:
              "Minimum seconds between detected steps to prevent double-counting.",
            min: 0.15,
            max: 1.0,
            step: 0.05,
            decimals: 2,
            defaultValue: 0.3,
          },
          {
            key: "minProminence",
            stateKey: "MinProminence",
            label: "Min Prominence",
            description:
              "Minimum peak-minus-trough swing (m/s²) required to register a step. Rejects noise and low-motion periods.",
            min: 0.5,
            max: 5.0,
            step: 0.1,
            decimals: 1,
            defaultValue: 1.5,
          },
        ],
      },
    },
  },

  compass: {
    key: "zooCompass",
    previewScreen: "Compass Preview",
    label: "Compass",
    description:
      "Computes magnetic bearing, optionally with tilt compensation.",

    outputs: [
      {
        replaces: "Compass",
        title: "Bearing",
        detail: "Magnetic bearing in degrees.",
        resultMapping: { magneticBearing: "magneticBearing" },
      },
    ],

    requirements: [
      {
        key: "compass",
        sensor: "Compass",
        label: "Compass Sensor On",
        check: (state) => getSensorEnabled(state, "Compass"),
        required: "On",
      },
      {
        key: "magnetometer",
        sensor: "Magnetometer",
        label: "Magnetometer Sensor On",
        check: (state) =>
          getSensorEnabled(state, "Magnetometer"),
        required: "On",
      },
      {
        key: "gravity",
        sensor: "Gravity",
        label: "Gravity Sensor On",
        check: (state) => getSensorEnabled(state, "Gravity"),
        required: "On",
      },
    ],

    enableRequirements: (setState) => {
      setState((state) => {
        let sensorState = { ...state.sensorState };
        sensorState["Compass"] = { ...sensorState["Compass"], enabled: true };
        sensorState["Magnetometer"] = {
          ...sensorState["Magnetometer"],
          enabled: true,
        };
        sensorState["Gravity"] = { ...sensorState["Gravity"], enabled: true };
        return { sensorState };
      });
    },

    systemDefault: {
      key: "system",
      label: "Basic Bearing",
      description:
        "Computes heading from the raw magnetometer XY plane, assuming a flat device.",
    },

    createProcessor: (filter) => {
      let lastGravity = null;
      let lastTime = null;
      let initialized = false;
      return {
        onData(data) {
          if (data.name === "gravity") {
            lastGravity = {
              x: data.values.x,
              y: data.values.y,
              z: data.values.z,
            };
          }
          if (data.name === "magnetometer") {
            if (!initialized) {
              filter.init();
              initialized = true;
              lastTime = data.time;
            }
            const dt = lastTime != null ? (data.time - lastTime) / 1e9 : 0.01;
            lastTime = data.time;
            if (dt > 0 && dt < 1) {
              return filter.update(
                data.values.x,
                data.values.y,
                data.values.z,
                lastGravity?.x,
                lastGravity?.y,
                lastGravity?.z,
                dt,
              );
            }
          }
          return null;
        },
      };
    },

    filters: {
      tiltCompensated: {
        label: "Tilt-Compensated Compass",
        description:
          "Computes magnetic bearing with tilt compensation using accelerometer data.",
        inputs: {
          required: [
            {
              sensor: "Magnetometer",
              role: "magnetometer",
              title: "Magnetic Field",
              detail: "Ambient magnetic field strength in microteslas.",
            },
          ],
          optional: [
            {
              sensor: "Gravity",
              role: "gravity",
              title: "Gravity",
              detail: "Gravity vector estimated by the system sensor fusion.",
            },
          ],
          driverSensor: "Magnetometer",
          fallbacks: {},
        },
        createFilter: (params) => new TiltCompensatedCompass(params.smoothing),
        params: [
          {
            key: "smoothing",
            stateKey: "Smoothing",
            label: "Smoothing",
            description:
              "EMA alpha for heading stability. Higher values produce smoother but slower-responding headings.",
            min: 0.01,
            max: 1.0,
            step: 0.01,
            decimals: 2,
            defaultValue: 0.15,
          },
        ],
      },
    },
  },
};

// Flat filter map — all filters across all categories, with key and categoryKey added
export const FUSION_FILTERS = {};
for (const [categoryKey, category] of Object.entries(FUSION_CATEGORIES)) {
  for (const [filterKey, filterDef] of Object.entries(category.filters)) {
    FUSION_FILTERS[filterKey] = { ...filterDef, key: filterKey, categoryKey };
  }
}
