/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    // Motors
    public static int leftLeader = 1;
    public static int leftFollower1 = 2;
    public static int leftFollower2 = 3;
    public static int spool = 4;
    public static int rightLeader = 5;
    public static int rightFollower1 = 6;
    public static int rightFollower2 = 7;
    public static int intake = 8;
    public static int index = 9; // Burnt out
    public static int turret = 10; // blinking yellow
    public static int feeder = 11;
    public static int shooterLeft = 12;
    public static int shooterRight = 13;
    public static int hood = 14;
    public static int lift = 15;

    // Encoders
    public static int turretEncoder = 0;
    public static int hoodEncoder = 1;

    // Controllers
    public static int driveController = 0;
    public static int systemsController = 1;

    // Speeds
    public static double intakeSpeed = 0.5;
    public static double outtakeSlowlySpeed = 0.3;
    public static double indexSpeed = 0.8;
    public static double feederSpeed = -3000;
    public static double shooterSpeed = 4000;
    public static double hoodRotateSpeed = 0.3;
    public static double turretTurn = 0.7;
    public static double indexRunTime = 0.25;

    // Buttons
    public static int aButton = 1;
    public static int bButton = 2;
    public static int xButton = 3;
    public static int yButton = 4;
    public static int leftBumper = 5;
    public static int rightBumper = 6;
    public static int backButton = 7;
    public static int startButton = 8;
    public static int leftStickPressDown = 9;
    public static int rightStickPressDown = 10;

    public static class unjamBalls {
        public static double ind_power = 0.2;
        public static double s_power = 0.6;
        public static double f_power = 0.6;
    }

    public static class shooterPID {
        public static double kP = 0.001;
        public static double kI = 0.000001;
        public static double kD = 0;
        public static double kFF = 0.0002;
        public static double kMax = 0.8;
        public static double kMin = -0.8;
        public static double shooterSpeedMinimum = 3700;
    }

    public static class feederPID {
        public static double kP = 0.005;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0.00026;
        public static double kMax = 0.8;
        public static double kMin = -0.8;
    }

    public static class turretPID {
        public static double kP = 0.08;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 0.8;
        public static double kMin = -0.8;

        // find these values for turret soft limit
        public static double leftPositionLimit = 0;
        public static double rightPositionLimit = 0;
    }

    // Trajectory Mapping
    public static boolean trajectoryMapping = false;

    public static class drivePID {
        public static double kP = .05;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 1;
        public static double kMin = -1;
        public static double rotate_kP = 0.02;
        public static double autonMaxSpeed = 0.7;
        public static double tolerance = 0.05;

        public static double kB = 2.0;
        public static double kZeta = 0.7;

        // Measure values and insert
        public static final double kTrackwidthMeters = 0;
        public static final double positionConversionFactor = 0; // (pi*diameter) in meters

        // Pick something reasonable like '3'.
        public static final double kMaxSpeedMetersPerSecond = 0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0;

        // find values through Drive Characterization
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kPDriveVel = 0;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
    }

    public static class fieldBasedTurretPID {
        public static double kP = 0.05;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 0;
        public static double kMin = 0;
        public static double maxSpeed = 0.2;
    }

    //gather data and find line of best fit (limelightArea (distance from target) (x) vs. hood position (y))
    public static class shooterModel {
        public static double a = 0.03;
        public static double b = -0.57;
        public static double c = -0.95;
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMin = 0;
        public static double kMax = 0;
        public static double tolerance = 0.1; 
    } 
}
