/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ComplexMotorSubsystem extends SubsystemBase {

  public static class SparkMaxConstants {
    public int id = -1;
    public boolean invert_motor = false;
  }

  public static class CANEncoderConstants {
    public boolean isAlternateEncoder = false;
    public int kCPR = 4096;
  }

  public static class ComplexSubsystemConstants {
    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public CANEncoderConstants kEncoderConstants = new CANEncoderConstants();

    public double kCurrentLimit = 80; // AMPS
    public double kMaxVoltage = 12.0;
  }

  protected double mLastSet = Double.NaN;
  protected ControlType mLastControlType = null;

  protected final ComplexSubsystemConstants mConstants;
  protected final CANSparkMax mMaster;

  protected final CANEncoder mEncoder;
  protected final CANPIDController mPIDController;
  protected final CANSparkMax[] mSlaves;

  public ComplexMotorSubsystem(final ComplexSubsystemConstants constants) {
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

    mEncoder = constants.kEncoderConstants.isAlternateEncoder
        ? mMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, constants.kEncoderConstants.kCPR)
        : mMaster.getEncoder();

    mPIDController = mMaster.getPIDController();

    mPIDController.setFeedbackDevice(mEncoder);
    mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
  }

  public void setOpenLoop(double demand) {
    mMaster.set(demand);
  }

  public void setClosedLoop(ControlType type, double setpoint) {
    if (setpoint != mLastSet || type != mLastControlType) {
      mLastSet = setpoint;
      mLastControlType = type;
      mMaster.getPIDController().setReference(setpoint, type);
    }
  }

  public void stop() {
    mMaster.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
