set(EXAMPLE_MOCO_TRACKING_MUSCLE_FILES
      gait_2D_muscle.osim
      IK_reference.sto)

MocoAddExampleCXX(NAME exampleTrackingMuscle
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_MUSCLE_FILES}")
