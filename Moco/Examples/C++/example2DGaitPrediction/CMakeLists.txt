set(EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES
      gait10dof18musc_MusclePolynomials_withPassiveForces.osim
      gait10dof18musc_MusclePolynomials_withoutPassiveForces.osim
      gait10dof18musc_MuscleGeometryPath.osim
      gait10dof18musc_CoordinateActuators.osim
      coordinateTracking_MusclePolynomials_solution.sto)

MocoAddExampleCXX(NAME example2DGaitPrediction
                  RESOURCES "${EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES}")
