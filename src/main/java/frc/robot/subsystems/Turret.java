/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.TurretConstants;

public class Turret extends ComplexMotorSubsystem {
  private static Turret mInstance;

  public synchronized static Turret getInstance() {
    if (mInstance == null) {
      mInstance = new Turret(TurretConstants.mTurretConstants);
    }
    return mInstance;
  }

  public Turret(ComplexSubsystemConstants constants) {
    super(constants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
