set(EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES
      coordinateTracking_solution.sto
      gait10dof18musc.osim)

MocoAddExampleCXX(NAME example2DGaitPrediction
                  RESOURCES "${EXAMPLE_MOCO_2D_GAIT_PREDICTION_FILES}")
