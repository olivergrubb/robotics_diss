from recovery_pipeline.error_responders.place_robot_upright import PlaceRobotUprightErrorResponder
from recovery_pipeline.error_responders.perpendicular_alignment_to_doorway_and_speed import PerpendicularAlignmentToDoorwayAndSpeedErrorResponder
from recovery_pipeline.error_responders.move_cable import MoveCableErrorResponder

ALL_ERROR_RESPONDERS = [PlaceRobotUprightErrorResponder,
                        PerpendicularAlignmentToDoorwayAndSpeedErrorResponder,
                        MoveCableErrorResponder]