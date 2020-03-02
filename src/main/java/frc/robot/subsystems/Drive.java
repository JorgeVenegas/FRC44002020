/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveSignal;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private MotorType selectedType = MotorType.kBrushless;
  private IdleMode selectedMode = IdleMode.kCoast;

  private CANSparkMax leftMaster = new CANSparkMax(DriveConstants.mFrontLeft, selectedType);
  private CANSparkMax rightMaster = new CANSparkMax(DriveConstants.mFrontRight, selectedType);
  private CANSparkMax leftSlave = new CANSparkMax(DriveConstants.mRearLeft, selectedType);
  private CANSparkMax rightSlave = new CANSparkMax(DriveConstants.mRearRight, selectedType);

  private CANEncoder mLeftEncoder = leftMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
  private CANEncoder mRightEncoder = rightMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);

  private ADIS16448_IMU mIMU = new ADIS16448_IMU();

  private Pose2d mPosition = new Pose2d(3.212, -6.98, Rotation2d.fromDegrees(getHeading()));

  private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DriveConstants.kTrackwidthInches));

  private DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
      mPosition);

  private SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  private PIDController mLeftPIDController = new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel,
      DriveConstants.kDDriveVel);

  private PIDController mRightPIDController = new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel,
      DriveConstants.kDDriveVel);

  private double lastTurnPower = 0, lastPower = 0;
  private static final double maxGain = 0.1;

  private FieldRelativeAngle desiredAngle;
  private DriveControlMode driveMode;

  private static Drive mInstance;

  public static enum DriveControlMode {
    CURVATURE, AUTO_STEER
  }

  public enum FieldRelativeAngle {
    FRONT, BACK
  }

  public synchronized static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  public Drive() {
    leftMaster.setInverted(false);
    rightMaster.setInverted(!leftMaster.getInverted());
    leftSlave.setInverted(leftMaster.getInverted());
    rightSlave.setInverted(!leftMaster.getInverted());

    leftMaster.setIdleMode(selectedMode);
    rightMaster.setIdleMode(selectedMode);
    leftSlave.setIdleMode(selectedMode);
    rightSlave.setIdleMode(selectedMode);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  @Override
  public void periodic() {
    mPosition = mOdometry.update(Rotation2d.fromDegrees(getHeading()), getEncoderDistanceMeters(mLeftEncoder),
        getEncoderDistanceMeters(mRightEncoder));
  }

  public DriveControlMode getDriveControlMode() {
    return driveMode;
  }

  public void setCheesyishDrive(XboxController joystick) {
    setCheesyishDrive(joystick.getRawAxis(1), -joystick.getRawAxis(4), joystick.getRawButton(6));
  }

  public void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
    if (epsilonEquals(throttle, 0.0, 0.04)) {
      throttle = 0.0;
    }

    if (epsilonEquals(wheel, 0.0, 0.035)) {
      wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
  }

  public void setOpenLoop(DriveSignal signal) {
    leftMaster.set(signal.getLeft());
    rightMaster.set(signal.getRight());
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  private static final double kEpsilon = 1E-9;

  public static DriveSignal inverseKinematics(Twist2d velocity) {
    if (Math.abs(velocity.dtheta) < kEpsilon) {
      return new DriveSignal(velocity.dx, velocity.dx);
    }
    double delta_v = DriveConstants.kTrackwidthInches * velocity.dtheta / (2 * DriveConstants.kTrackScrubFactor);
    return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
  }

  public void setAutoSteerDrive(XboxController joystick) {
    setAutoSteerDrive(-joystick.getRawAxis(1));
  }

  public double getError(double targetAngle) {
    double angleError = 0;

    angleError = (targetAngle - getHeading());
    angleError -= (360 * Math.floor(0.5 + ((angleError) / 360.0)));

    return angleError;
  }

  public void setAutoSteerDrive(double throttle) {
    // double adjustment = maxOutput - minAccel;
    double gain = 0.01;
    double turn = desiredAngle == FieldRelativeAngle.FRONT ? -getError(0) * gain
        : desiredAngle == FieldRelativeAngle.BACK ? -getError(180) * gain : -getError(0) * gain;
    if (Math.abs(throttle - lastPower) > maxGain)
      throttle = throttle >= lastPower ? lastPower + maxGain : lastPower - maxGain;
    if (Math.abs(turn - lastTurnPower) > maxGain)
      turn = turn >= lastTurnPower ? lastTurnPower + maxGain : lastTurnPower - maxGain;
    autoSteer(throttle, turn);
  }

  public void autoSteer(double movePower, double rotateValue) {
    setLeft(movePower + rotateValue);
    setRight(movePower - rotateValue);
    lastTurnPower = rotateValue;
    lastPower = movePower;
  }

  private void setLeft(double leftPower) {
    if (leftPower > 1)
      leftPower = 1;
    if (leftPower < -1)
      leftPower = -1;
    leftMaster.set(leftPower);
  }

  private void setRight(double rightPower) {
    if (rightPower > 1)
      rightPower = 1;
    if (rightPower < -1)
      rightPower = -1;
    rightMaster.set(rightPower);
  }

  public double getHeading() {
    return Math.IEEEremainder(mIMU.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //////////////////////////// PATH FOLLOWING \\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // izquierdoE.setVoltage(leftVolts);
    // derechoE.setVoltage(rightVolts);
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // return new DifferentialDriveWheelSpeeds(getEncoderRatePerSecond(izquierdoE),
    // getEncoderRatePerSecond(derechoE));
    return new DifferentialDriveWheelSpeeds();
  }

  public double getEncoderDistanceMeters(CANEncoder encoder) {
    return encoder.getPosition() * Math.PI * Units.inchesToMeters(6) / 4096;
  }

  public double getEncoderVelocityMetersPerSecond(CANEncoder encoder) {
    return encoder.getVelocity() / 4096 * Math.PI * Units.inchesToMeters(6) / 60;
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return mKinematics;
  }

  public SimpleMotorFeedforward getDriveFeedForward() {
    return mFeedForward;
  }

  public PIDController getDriveLeftPIDController() {
    return mLeftPIDController;
  }

  public PIDController getDriveRightPIDController() {
    return mRightPIDController;
  }
}
