set(EXAMPLE_MOCO_TRACKING_TORQUE_FILES
      gait_2D_contact_torque_bounds.osim
      gait_2D_contact_torque_bounds_noPelvisRes.osim
      IK_reference_radian.sto)

MocoAddExampleCXX(NAME exampleTrackingTorque
                  RESOURCES "${EXAMPLE_MOCO_TRACKING_TORQUE_FILES}")
