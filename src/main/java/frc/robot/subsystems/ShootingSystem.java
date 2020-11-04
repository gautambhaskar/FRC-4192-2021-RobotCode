/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.feederPID;
import frc.robot.Constants.shooterPID;

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
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_maxVel, f_minVel, f_maxAcc,
      f_allowedErr;

  // Motors
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.feeder, MotorType.kBrushed);
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.shooterLeft, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.shooterRight, MotorType.kBrushless);

  // PID Controller
  private CANPIDController shooterController = shooterLeftMotor.getPIDController();
  private CANPIDController feederController = feederMotor.getPIDController();

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);

    SmartDashboard.putNumber("Shooter kP", Constants.shooterPID.kP);
    SmartDashboard.putNumber("Shooter kI", Constants.shooterPID.kI);
    SmartDashboard.putNumber("Shooter kD", Constants.shooterPID.kD);
    SmartDashboard.putNumber("Shooter kFF", Constants.shooterPID.kFF);
    SmartDashboard.putNumber("Shooter kMin", Constants.shooterPID.kMin);
    SmartDashboard.putNumber("Shooter kMax", Constants.shooterPID.kMax);

    SmartDashboard.putNumber("Feeder kP", Constants.feederPID.kP);
    SmartDashboard.putNumber("Feeder kI", Constants.feederPID.kI);
    SmartDashboard.putNumber("Feeder kD", Constants.feederPID.kD);
    SmartDashboard.putNumber("Feeder kFF", Constants.feederPID.kFF);
    SmartDashboard.putNumber("Feeder kMin", Constants.feederPID.kMin);
    SmartDashboard.putNumber("Feeder kMax", Constants.feederPID.kMax);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Speed", feederMotor.getEncoder().getVelocity());

    // Grab numbers from SmartDashboard and set to motors

    // TEST THIS!!! CHECK "SOURCES" AND PULL VALUE TO A WIDGET FOR THE PID
    // CONTROLLER. CHECK IF YOU CAN MANIPULATE VALUES DIRECTLY WHEN IN TEST MODE...

    double[] newShooterPIDconstants = { SmartDashboard.getNumber("Shooter kP", 0),
        SmartDashboard.getNumber("Shooter kI", 0), SmartDashboard.getNumber("Shooter kD", 0),
        SmartDashboard.getNumber("Shooter kFF", 0), SmartDashboard.getNumber("Shooter kMin", 0),
        SmartDashboard.getNumber("Shooter kMax", 0) };
    double[] newFeederPIDconstants = { SmartDashboard.getNumber("Feeder kP", 0),
        SmartDashboard.getNumber("Feeder kI", 0), SmartDashboard.getNumber("Feeder kD", 0),
        SmartDashboard.getNumber("Feeder kFF", 0), SmartDashboard.getNumber("Feeder kMin", 0),
        SmartDashboard.getNumber("Feeder kMax", 0) };
    Constants.distributePID(newShooterPIDconstants, shooterController);
    Constants.distributePID(newFeederPIDconstants, feederController);

    // Grab constants from motors and post to SmartDashboard
    SmartDashboard.putNumber("Shooter kP", shooterController.getP());
    SmartDashboard.putNumber("Shooter kI", shooterController.getI());
    SmartDashboard.putNumber("Shooter kD", shooterController.getD());
    SmartDashboard.putNumber("Shooter kFF", shooterController.getFF());
    SmartDashboard.putNumber("Shooter kMin", shooterController.getOutputMin());
    SmartDashboard.putNumber("Shooter kMax", shooterController.getOutputMax());

    SmartDashboard.putNumber("Feeder kP", feederController.getP());
    SmartDashboard.putNumber("Feeder kI", feederController.getI());
    SmartDashboard.putNumber("Feeder kD", feederController.getD());
    SmartDashboard.putNumber("Feeder kFF", feederController.getFF());
    SmartDashboard.putNumber("Feeder kMin", feederController.getOutputMin());
    SmartDashboard.putNumber("Feeder kMax", feederController.getOutputMax());
  }

  public void startShooter(double shooterSpeed, double feederSpeed) {
    shooterController.setReference(shooterSpeed, ControlType.kVelocity);
    feederController.setReference(feederSpeed, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Setpoint", shooterSpeed);
    SmartDashboard.putNumber("Feeder Setpoint", feederSpeed);
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
