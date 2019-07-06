set(EXAMPLE_MOCO_TRACKING_TORQUE_FILES
      gait_2D_torque.osim
      IK_reference.sto
      IK_reference_guess.sto)

MocoAddExampleCXX(NAME exampleTrackingTorque
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_TORQUE_FILES}")
