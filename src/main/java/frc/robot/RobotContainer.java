/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveDefaultReal;
import frc.robot.subsystems.Blaster;
import frc.robot.subsystems.BlasterTuner;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private Drive mDrive = Drive.getInstance();
  private Intake mIntake = Intake.getInstance();
  private Feeder mFeeder = Feeder.getInstance();
  private Turret mTurret = Turret.getInstance();
  private Blaster mBlaster = Blaster.getInstance();
  private BlasterTuner mBlasterTuner = BlasterTuner.getInstance();
  
  private Limelight mLimelight = Limelight.identity();
  private Limelight mLimelight2 = Limelight.identity();

  private XboxController mDriverController = new XboxController(Constants.mDriverControllerPort);
  private XboxController mSystemsController = new XboxController(Constants.mSystemsControllerPort);

  public RobotContainer() {
    mDrive.setDefaultCommand(new DriveDefaultReal(mDrive, mDriverController));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    mFeeder.setOpenLoop(0);
    mIntake.setOpenLoop(0);
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
