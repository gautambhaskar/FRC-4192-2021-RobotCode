/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANPIDController;

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
    public static double indexSpeed = 0.3;
    public static double feederSpeed = -3000;
    public static double shooterSpeed = 4000;
    public static double hoodRotateSpeed = 0.3;
    public static double turretTurn = 0.7;
    public static double rotateDrivtrainAngle = .3;

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
        public static double s_power = 0.2;
        public static double f_power = 0.2;
    }

    public static class shooterPID {
        public static double kP = 0.0004;
        public static double kI = 0.000001;
        public static double kD = 0;
        public static double kFF = 0.0002;
        public static double kMax = 0.8;
        public static double kMin = -0.8;
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
        public static double kP = 0.03;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 0.8;
        public static double kMin = -0.8;
    }
    
    public static class drivePID{
        public static double kP = .05;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 1;
        public static double kMin = -1;
    }
    public static class rotateAnglePID{
        public static double kP = .0005;
        public static double kI = 0;
        public static double kD = 0;
        public static double kFF = 0;
        public static double kMax = 1;
        public static double kMin = -1;
    }

    public static void distributePID(double[] constants, CANPIDController controller) {
    controller.setP(constants[0]);
        controller.setI(constants[1]);
        controller.setD(constants[2]);
        controller.setFF(constants[3]);
        controller.setOutputRange(constants[4], constants[5]);
    }
}
