/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.BlasterConstants;

public class Blaster extends ComplexMotorSubsystem {
  
  private static Blaster mInstance;

  public synchronized static Blaster getInstance() {
    if (mInstance == null) {
      mInstance = new Blaster(BlasterConstants.mBlasterConstants);
    }
    return mInstance;
  }

  public Blaster(ComplexSubsystemConstants constants) {
    super(constants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
