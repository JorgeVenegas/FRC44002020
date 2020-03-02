/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry mTl = mLimelightTable.getEntry("tl");
  private NetworkTableEntry mTx = mLimelightTable.getEntry("tx");
  private NetworkTableEntry mTy = mLimelightTable.getEntry("ty");
  private NetworkTableEntry mTa = mLimelightTable.getEntry("ta");
  private NetworkTableEntry mTv = mLimelightTable.getEntry("tv");

  public static Limelight identity() {
    return new Limelight();
  }

  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTl() {
    return mTl.getDouble(0);
  }

  public double getTx() {
    return mTx.getDouble(0);
  }

  public double getTy() {
    return mTy.getDouble(0);
  }

  public double getTa() {
    return mTa.getDouble(0);
  }

  public double getTv() {
    return mTv.getDouble(0);
  }
}
