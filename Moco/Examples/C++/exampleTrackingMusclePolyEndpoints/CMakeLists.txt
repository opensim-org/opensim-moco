set(EXAMPLE_MOCO_TRACKING_MUSCLE_POLY_ENDPOINTS_FILES
      gait_2D_contact_muscle_poly_bounds_noPelvisRes.osim
      IK_reference_radian.sto)

MocoAddExampleCXX(NAME exampleTrackingMusclePolyEndpoints
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_MUSCLE_POLY_ENDPOINTS_FILES}")
