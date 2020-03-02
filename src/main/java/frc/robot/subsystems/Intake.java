/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

public class Intake extends ComplexMotorSubsystem {

  private static Intake mInstance;

  public synchronized static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake(IntakeConstants.mIntakeConstants);
    }
    return mInstance;
  }

  public Intake(ComplexSubsystemConstants constants) {
    super(constants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
