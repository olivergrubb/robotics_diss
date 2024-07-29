from recovery_pipeline.error_detectors.wheels_off_ground import WheelsOffGroundErrorDetector
from recovery_pipeline.error_detectors.doorway_boundary import DoorwayBoundaryDetector
from recovery_pipeline.error_detectors.cable import CableDetector

ALL_ERROR_DETECTORS = [WheelsOffGroundErrorDetector,
                       DoorwayBoundaryDetector,
                       CableDetector]