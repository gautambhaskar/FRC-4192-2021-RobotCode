/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
//import frc.robot.Constants.feederPID;
//import frc.robot.Constants.shooterPID;
import frc.robot.Constants.shooterPID;
import frc.robot.Constants.feederPID;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
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

  // Motors
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.feeder, MotorType.kBrushed);
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.shooterLeft, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.shooterRight, MotorType.kBrushless);

  // Flywheel Shaft Encoder
  private final Encoder flyWheelEncoder = new Encoder(5, 6);

  // PID Controller
  private CANPIDController shooterController = shooterLeftMotor.getPIDController();
  private CANPIDController feederController = feederMotor.getPIDController();
  //private CANEncoder flyWheelEncoder = shooterLeftMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

  // Shuffleboard Tabs
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
  private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  private NetworkTableEntry shooterSpeed, shooterRightSpeed, feederSpeed, atSetpoint, flyWheelSpeed;

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);
    flyWheelEncoder.setDistancePerPulse(0.01);

    // shooter spark max is connected to through bore like this, if plugged in using
    // alternate encoder adapter.
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

    shooterRightSpeed = tuningTab.add("Shooter Right Speed", shooterRightMotor.getEncoder().getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(5, 0).getEntry();
    // Graph of feederSpeed
    feederSpeed = tuningTab.add("Feeder Speed", -feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(0, 0).getEntry();

    // Graph of FlyWheelSpeed
    flyWheelSpeed = tuningTab.add("FlyWheel Speed", flyWheelEncoder.getRate()).withWidget(BuiltInWidgets.kGraph)
        .withSize(2, 2).withPosition(5, 5).getEntry();

    atSetpoint = mainTab.add("At Setpoint", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the ShooterSpeed and FeederSpeed Graphs
    shooterSpeed.setDouble(shooterLeftMotor.getEncoder().getVelocity());
    shooterRightSpeed.setDouble(shooterRightMotor.getEncoder().getVelocity());
    Globals.flyWheelSpeed = flyWheelEncoder.getRate();
    feederSpeed.setDouble(feederMotor.getEncoder(EncoderType.kQuadrature, 8192).getVelocity());
    flyWheelSpeed.setDouble(flyWheelEncoder.getRate());

    if (flyWheelEncoder.getRate() > shooterPID.flyWheelSpeedMinimum) {
      atSetpoint.setBoolean(true);
    } else {
      atSetpoint.setBoolean(false);
    }

  }

  public void startShooter() {
    shooterController.setReference(Constants.flywheelSpeed, ControlType.kVelocity);
    feederController.setReference(Constants.feederSpeed, ControlType.kVelocity);
  }

  public double getFlywheelSpeed() {
    return flyWheelEncoder.getRate();
  }

  public void initializeShooter() {
    // Set shooter PID values on controller
    shooterController.setP(shooterPID.kP);
    shooterController.setI(shooterPID.kI);
    shooterController.setD(shooterPID.kD);
    shooterController.setFF(shooterPID.kFF);
    shooterController.setOutputRange(shooterPID.kMin, shooterPID.kMax);
    //shooterController.setFeedbackDevice(flyWheelEncoder);

    // Set feeder PID values on controller
    feederController.setP(feederPID.kP);
    feederController.setI(feederPID.kI);
    feederController.setD(feederPID.kD);
    feederController.setFF(feederPID.kFF);
    feederController.setOutputRange(feederPID.kMin, feederPID.kMax);
  }

  public void setPower(double s_power, double f_power) {
    shooterLeftMotor.set(s_power);
    feederController.setReference(f_power, ControlType.kVoltage);
  }

  public void stopShooter() {
    shooterLeftMotor.set(0);
    feederController.setReference(0, ControlType.kVoltage);
  }

  // public double shooterSpeed() {
  // return shooterLeftMotor.getEncoder().getVelocity();
  // }
}
