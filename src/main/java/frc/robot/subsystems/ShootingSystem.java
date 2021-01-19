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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private NetworkTableEntry shooterSpeed, feederSpeed;

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);
    // feederController.setFeedbackDevice(feederMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,
    // 8192));

    // Set shooter PID values on controller
    shooterController.setP(shooterPID.kP);
    shooterController.setI(shooterPID.kI);
    shooterController.setD(shooterPID.kD);
    shooterController.setFF(shooterPID.kFF);
    shooterController.setOutputRange(shooterPID.kMin, shooterPID.kMax);

    // Set feeder PID values on controller
    feederController.setP(feederPID.kP);
    feederController.setI(feederPID.kI);
    feederController.setD(feederPID.kD);
    feederController.setFF(feederPID.kFF);
    feederController.setOutputRange(feederPID.kMin, feederPID.kMax);

    // Graph of shooterSpeed
    shooterSpeed = tuningTab.add("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(5, 0).getEntry();
    // Graph of feederSpeed
    feederSpeed = tuningTab.add("Feeder Speed", -feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the ShooterSpeed and FeederSpeed Graphs
    shooterSpeed.setDouble(shooterLeftMotor.getEncoder().getVelocity());
    feederSpeed.setDouble(feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity());
    SmartDashboard.putNumber("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity());
  }

  public void startShooter() {
    
    // Set shooter PID values on controller
    shooterController.setP(shooterPID.kP);
    shooterController.setI(shooterPID.kI);
    shooterController.setD(shooterPID.kD);
    shooterController.setFF(shooterPID.kFF);
    shooterController.setOutputRange(shooterPID.kMin, shooterPID.kMax);

    // Set feeder PID values on controller
    feederController.setP(feederPID.kP);
    feederController.setI(feederPID.kI);
    feederController.setD(feederPID.kD);
    feederController.setFF(feederPID.kFF);
    feederController.setOutputRange(feederPID.kMin, feederPID.kMax);
    
    shooterController.setReference(Constants.shooterSpeed, ControlType.kVelocity);
    feederController.setReference(Constants.feederSpeed, ControlType.kVelocity);
  }

  public void setPower(double s_power, double f_power) {
    shooterController.setReference(s_power, ControlType.kVoltage);
    feederController.setReference(f_power, ControlType.kVoltage);
  }

  public void stopShooter() {
    shooterController.setReference(0, ControlType.kVoltage);
    feederController.setReference(0, ControlType.kVoltage);
  }

  public double shooterSpeed() {
    return shooterLeftMotor.getEncoder().getVelocity();
  }
}
