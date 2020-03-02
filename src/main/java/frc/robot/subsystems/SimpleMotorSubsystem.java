/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotorSubsystem extends SubsystemBase {

  public static class SparkMaxConstants {
    public int id = -1;
    public boolean invert_motor = false;
  }

  public static class SimpleSubsystemConstants {
    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public double kCurrentLimit = 80; // AMPS
    public double kMaxVoltage = 12.0;
  }

  protected final SimpleSubsystemConstants mConstants;
  protected final CANSparkMax mMaster;
  protected final CANSparkMax[] mSlaves;

  protected SimpleMotorSubsystem(final SimpleSubsystemConstants constants) {
    mConstants = constants;
    mMaster = new CANSparkMax(mConstants.kMasterConstants.id, MotorType.kBrushless);
    mSlaves = new CANSparkMax[mConstants.kSlaveConstants.length];

    mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
    mMaster.setIdleMode(IdleMode.kBrake);

    for (int i = 0; i < mSlaves.length; ++i) {
      mSlaves[i] = new CANSparkMax(mConstants.kSlaveConstants[i].id, MotorType.kBrushless);
      mSlaves[i].follow(mMaster, mConstants.kSlaveConstants[i].invert_motor);
      mSlaves[i].setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOpenLoop(double inPower) {
    mMaster.set(inPower);
  }

  public void stop() {
    mMaster.set(0);
  }
}
