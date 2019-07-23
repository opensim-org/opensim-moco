set(EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES
      gait10dof18musc_MusclePolynomials_withPassiveForces.osim
      gait10dof18musc_MusclePolynomials_withoutPassiveForces.osim
      gait10dof18musc_MuscleGeometryPath_withPassiveForces.osim
      coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto
      coordinateTracking_MusclePolynomials_withoutPassiveForces_solution.sto
      coordinateTracking_solution.sto
      gait10dof18musc.osim)

MocoAddExampleCXX(NAME example2DGaitPrediction
                  RESOURCES "${EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES}")
