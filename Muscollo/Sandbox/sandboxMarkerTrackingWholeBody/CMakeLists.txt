MuscolloAddSandboxExecutable(NAME sandboxMarkerTrackingWholeBody
	    LIB_DEPENDS osimMuscollo tropter
		RESOURCES
			subject01.osim
			../walk_marker_trajectories.trc
			../walk_gait1018_state_reference.mot
        SOURCE_FILES
            MuscleLikeCoordinateActuator.cpp
            MuscleLikeCoordinateActuator.h
		)