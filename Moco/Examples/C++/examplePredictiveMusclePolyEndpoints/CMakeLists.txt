set(EXAMPLE_MOCO_PREDICTIVE_MUSCLE_POLY_ENDPOINTS_FILES
      gait_2D_contact_muscle_poly_bounds_noPelvisRes.osim
      gait2D_TrackingMusclePolyEndPoints_SymPositions10_TrackingPosition10_solution.sto)

MocoAddExampleCXX(NAME examplePredictiveMusclePolyEndpoints
                  RESOURCES "${EXAMPLE_MOCO_PREDICTIVE_MUSCLE_POLY_ENDPOINTS_FILES}")
