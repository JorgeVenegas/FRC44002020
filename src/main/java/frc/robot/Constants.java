/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.ComplexMotorSubsystem.ComplexSubsystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int mDriverControllerPort = 0;
    public static final int mSystemsControllerPort = 1;

    public static final class DriveConstants {
        public static final int mFrontLeft = 1;
        public static final int mFrontRight = 2;
        public static final int mRearLeft = 3;
        public static final int mRearRight = 4;

        public static final double kTrackwidthInches = 27.5;
        public static final double kTrackScrubFactor = 1.0469745223;

        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        public static final double kPDriveVel = 0;
        public static final double kDDriveVel = 0;
        public static final double kIDriveVel = 0;

        public static final boolean kGyroReversed = true;
    }

    public static final class IntakeConstants {
        public static final ComplexSubsystemConstants mIntakeConstants = new ComplexSubsystemConstants();
        static {
            mIntakeConstants.kMasterConstants.id = 5;
            mIntakeConstants.kMasterConstants.invert_motor = false;
        }
    }

    public static final class FeederConstants {
        public static final ComplexSubsystemConstants mFeederConstants = new ComplexSubsystemConstants();
        static {
            mFeederConstants.kMasterConstants.id = 5;
            mFeederConstants.kMasterConstants.invert_motor = false;

            mFeederConstants.kSlaveConstants[0].id = 6;
            mFeederConstants.kSlaveConstants[0].invert_motor = false;
        }
    }

    public static final class TurretConstants {
        public static final ComplexSubsystemConstants mTurretConstants = new ComplexSubsystemConstants();
        static {
            mTurretConstants.kMasterConstants.id = 5;
            mTurretConstants.kMasterConstants.invert_motor = false;
        }
    }

    public static final class BlasterTunerConstants {
        public static final ComplexSubsystemConstants mBlasterTunerConstants = new ComplexSubsystemConstants();
        static {
            mBlasterTunerConstants.kMasterConstants.id = 5;
            mBlasterTunerConstants.kMasterConstants.invert_motor = false;
        }
    }

    public static final class BlasterConstants {
        public static final ComplexSubsystemConstants mBlasterConstants = new ComplexSubsystemConstants();
        static {
            mBlasterConstants.kMasterConstants.id = 5;
            mBlasterConstants.kMasterConstants.invert_motor = false;

            mBlasterConstants.kSlaveConstants[0].id = 6;
            mBlasterConstants.kSlaveConstants[0].invert_motor = false;
        }
    }

}
