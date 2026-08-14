export { quaternionMultiply } from "./utils/quaternion.js";

export { ComplementaryFilter } from "./filters/ComplementaryFilter.js";
export { MadgwickFilter } from "./filters/MadgwickFilter.js";
export { MahonyFilter } from "./filters/MahonyFilter.js";
export { EKFFilter } from "./filters/EKFFilter.js";
export { VQFFilter } from "./filters/VQFFilter.js";
export { AdaptiveStepCounter } from "./steps/AdaptiveStepCounter.js";
export { WindowedPeakStepCounter } from "./steps/WindowedPeakStepCounter.js";
export { TiltCompensatedCompass } from "./compass/TiltCompensatedCompass.js";
export {
  FUSION_CATEGORIES,
  FUSION_FILTERS,
} from "./registry.js";
