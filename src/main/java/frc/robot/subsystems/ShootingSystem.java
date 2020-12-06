/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.feederPID;
//import frc.robot.Constants.shooterPID;
import frc.robot.Constants.shooterPID;
import frc.robot.Constants.feederPID;

import java.util.Arrays;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

public class ShootingSystem extends SubsystemBase {
  /**
   * Creates a new ShootingSystem.
   */
  // Index 9
  // Turret 10
  // Feeder 11
  // Shooter Left 12
  // Shooter Right 13
  // Hood 14

  // Constants
  private NetworkTableEntry kP, kI, kD, kFF, kMax, kMin;
  private NetworkTableEntry f_kP, f_kI, f_kD, f_kFF, f_kMax, f_kMin;
  private ShuffleboardLayout shuffleFeederPID, shuffleShooterPID;

  // Motors
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.feeder, MotorType.kBrushed);
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.shooterLeft, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.shooterRight, MotorType.kBrushless);

  // PID Controller
  private CANPIDController shooterController = shooterLeftMotor.getPIDController();
  private CANPIDController feederController = feederMotor.getPIDController();

  // Shuffleboard Tabs
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

  private double[] s_pastPIDconstants;
  private double[] f_pastPIDconstants;
  private double[] newShooterPID;
  private double[] newFeederPID;

  private NetworkTableEntry shooterSpeed, feederSpeed, shooterSetpoint, feederSetpoint;

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);
    // feederController.setFeedbackDevice(feederMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,
    // 8192));
    shuffleFeederPID = tuningTab.getLayout("Feeder PID", BuiltInLayouts.kList).withSize(1, 5);
    shuffleShooterPID = tuningTab.getLayout("Shooter PID", BuiltInLayouts.kList).withSize(1, 5);

    kP = shuffleShooterPID.add("Shooter kP", shooterPID.kP).getEntry();
    kI = shuffleShooterPID.add("Shooter kI", shooterPID.kI).getEntry();
    kD = shuffleShooterPID.add("Shooter kD", shooterPID.kD).getEntry();
    kFF = shuffleShooterPID.add("Shooter kFF", shooterPID.kFF).getEntry();
    kMin = shuffleShooterPID.add("Shooter kMin", shooterPID.kMin).getEntry();
    kMax = shuffleShooterPID.add("Shooter kMax", shooterPID.kMax).getEntry();
    s_pastPIDconstants = new double[] { shooterPID.kP, shooterPID.kI, shooterPID.kD, shooterPID.kFF, shooterPID.kMin,
        shooterPID.kMax };

    f_kP = shuffleFeederPID.add("Feeder kP", feederPID.kP).getEntry();
    f_kI = shuffleFeederPID.add("Feeder kI", feederPID.kI).getEntry();
    f_kD = shuffleFeederPID.add("Feeder kD", feederPID.kD).getEntry();
    f_kFF = shuffleFeederPID.add("Feeder kFF", feederPID.kFF).getEntry();
    f_kMin = shuffleFeederPID.add("Feeder kMin", feederPID.kMin).getEntry();
    f_kMax = shuffleFeederPID.add("Feeder kMax", feederPID.kMax).getEntry();
    f_pastPIDconstants = new double[] { feederPID.kP, feederPID.kI, feederPID.kD, feederPID.kFF, feederPID.kMin,
        feederPID.kMax };

    newShooterPID = new double[6]; // If doesn't work, put outside constructor.
    newFeederPID = new double[6];

    shooterSpeed = tuningTab.add("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(5, 0).getEntry();
    shooterSetpoint = shuffleShooterPID.add("Shooter Setpoint", Constants.shooterSpeed).getEntry();
    feederSetpoint = shuffleFeederPID.add("Feeder Setpoint", Constants.feederSpeed).getEntry();
    feederSpeed = tuningTab.add("Feeder Speed", -feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterSpeed.setDouble(shooterLeftMotor.getEncoder().getVelocity());
    feederSpeed.setDouble(feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity());

    // Grab numbers from SmartDashboard and set to motors

    // TEST THIS!!! CHECK "SOURCES" AND PULL VALUE TO A WIDGET FOR THE PID
    // CONTROLLER. CHECK IF YOU CAN MANIPULATE VALUES DIRECTLY WHEN IN TEST MODE...

    newShooterPID[0] = kP.getDouble(0);
    newShooterPID[1] = kI.getDouble(0);
    newShooterPID[2] = kD.getDouble(0);
    newShooterPID[3] = kFF.getDouble(0);
    newShooterPID[4] = kMin.getDouble(0);
    newShooterPID[5] = kMax.getDouble(0);

    newFeederPID[0] = f_kP.getDouble(0);
    newFeederPID[1] = f_kI.getDouble(0);
    newFeederPID[2] = f_kD.getDouble(0);
    newFeederPID[3] = f_kFF.getDouble(0);
    newFeederPID[4] = f_kMin.getDouble(0);
    newFeederPID[5] = f_kMax.getDouble(0);

    // Puts new values into old array
    if (Arrays.equals(newShooterPID, s_pastPIDconstants) == false) {
      s_pastPIDconstants = newShooterPID.clone();// clone works?
      Constants.distributePID(newShooterPID, shooterController);
    }
    if (Arrays.equals(newFeederPID, f_pastPIDconstants) == false) {
      f_pastPIDconstants = newFeederPID.clone();
      Constants.distributePID(newFeederPID, feederController);
    }
  }

  public void startShooter() {
    shooterController.setReference(shooterSetpoint.getDouble(0), ControlType.kVelocity);
    // feederController.setReference(feederSpd, ControlType.kVelocity);
    feederController.setReference(feederSetpoint.getDouble(0), ControlType.kVelocity);
    // feederMotor.set(0.7);
  }

  public void setPower(double s_power, double f_power) {
    shooterController.setReference(s_power, ControlType.kVoltage);
    feederController.setReference(f_power, ControlType.kVoltage);
  }

  public void stopShooter() {
    shooterController.setReference(0, ControlType.kVoltage);
    feederController.setReference(0, ControlType.kVoltage);
  }
}
