set(EXAMPLE_MOCO_TRACKING_MUSCLE_ENDPOINTS_FILES
      gait_2D_contact_muscle_bounds_noPelvisRes.osim
      IK_reference_radian.sto)

MocoAddExampleCXX(NAME exampleTrackingMuscleEndpoints
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_MUSCLE_ENDPOINTS_FILES}")
