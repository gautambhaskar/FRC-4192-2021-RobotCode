/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.feederPID;
import frc.robot.Constants.shooterPID;

import java.util.Arrays;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  
  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);
    feederController.setFeedbackDevice(feederMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192));
    
    tuningTab.add("Shooter kP", shooterController.getP()).getEntry();
    kI = tuningTab.add("Shooter kI", shooterController.getI()).getEntry();
    kD = tuningTab.add("Shooter kD", shooterController.getD()).getEntry();
    kFF = tuningTab.add("Shooter kFF", shooterController.getFF()).getEntry();
    kMin = tuningTab.add("Shooter kMin", shooterController.getOutputMin()).getEntry();
    kMax = tuningTab.add("Shooter kMax", shooterController.getOutputMax()).getEntry();
    s_pastPIDconstants = new double[] { shooterController.getP(), shooterController.getI(), shooterController.getD(),
        shooterController.getFF(), shooterController.getOutputMin(), shooterController.getOutputMax() };

    f_kP = tuningTab.add("Feeder kP", feederController.getP()).getEntry();
    f_kI = tuningTab.add("Feeder kI", feederController.getI()).getEntry();
    f_kD = tuningTab.add("Feeder kD", feederController.getD()).getEntry();
    f_kFF = tuningTab.add("Feeder kFF", feederController.getFF()).getEntry();
    f_kMin = tuningTab.add("Feeder kMin", feederController.getOutputMin()).getEntry();
    f_kMax = tuningTab.add("Feeder kMax", feederController.getOutputMax()).getEntry();
    f_pastPIDconstants = new double[] { feederController.getP(), feederController.getI(), feederController.getD(),
        feederController.getFF(), feederController.getOutputMin(), feederController.getOutputMax() };

    newShooterPID = new double[6]; // If doesn't work, put outside constructor.
    newFeederPID = new double[6];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tuningTab.addNumber("Shooter Speed", () -> shooterLeftMotor.getEncoder().getVelocity());
    tuningTab.addNumber("Feeder Speed",
        () -> feederMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getVelocity());

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
      s_pastPIDconstants = newShooterPID;
      Constants.distributePID(newShooterPID, shooterController);
    }
    if (Arrays.equals(newFeederPID, f_pastPIDconstants) == false) {
      f_pastPIDconstants = newFeederPID;
      Constants.distributePID(newFeederPID, feederController);
    }

    tuningTab.addNumber("Current kP of Shooter", () -> shooterController.getP());
    tuningTab.addNumber("Current kI of Shooter", () -> shooterController.getI());
    tuningTab.addNumber("Current kD of Shooter", () -> shooterController.getD());
  }
  public void startShooter(double shooterSpeed, double feederSpeed) {
    shooterController.setReference(shooterSpeed, ControlType.kVelocity);
    feederController.setReference(feederSpeed, ControlType.kVelocity);
    tuningTab.add("Shooter Setpoint", shooterSpeed);
    tuningTab.add("Feeder Setpoint", feederSpeed);
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
