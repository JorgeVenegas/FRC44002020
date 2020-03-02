/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.FeederConstants;

public class Feeder extends ComplexMotorSubsystem {

  private static Feeder mInstance;

  public synchronized static Feeder getInstance() {
    if (mInstance == null) {
      mInstance = new Feeder(FeederConstants.mFeederConstants);
    }
    return mInstance;
  }

  public Feeder(ComplexSubsystemConstants constants) {
    super(constants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
