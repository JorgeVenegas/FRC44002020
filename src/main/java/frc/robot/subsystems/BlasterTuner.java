/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.BlasterTunerConstants;

public class BlasterTuner extends ComplexMotorSubsystem {
  private static BlasterTuner mInstance;

  public synchronized static BlasterTuner getInstance() {
    if (mInstance == null) {
      mInstance = new BlasterTuner(BlasterTunerConstants.mBlasterTunerConstants);
    }
    return mInstance;
  }

  public BlasterTuner(ComplexSubsystemConstants constants) {
    super(constants);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
