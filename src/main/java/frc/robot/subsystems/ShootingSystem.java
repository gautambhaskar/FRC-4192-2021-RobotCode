/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, MotorType.kBrushless);

  // PID Controller
  private CANPIDController shooterController = shooterLeftMotor.getPIDController();
  private CANPIDController feederController = feederMotor.getPIDController();

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);

    // PID coefficients
    kP = 5e-8;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0.8;
    kMinOutput = -0.8;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    shooterController.setP(kP);
    shooterController.setI(kI);
    shooterController.setD(kD);
    shooterController.setIZone(kIz);
    shooterController.setFF(kFF);
    shooterController.setOutputRange(kMinOutput, kMaxOutput);

    // PID coefficients
    f_kP = 5e-8;
    f_kI = 0;
    f_kD = 0;
    f_kIz = 0;
    f_kFF = 0;
    f_kMaxOutput = 0.8;
    f_kMinOutput = -0.8;
    f_maxRPM = 5700;

    // Smart Motion Coefficients
    f_maxVel = 2000; // rpm
    f_maxAcc = 1500;

    feederController.setP(f_kP);
    feederController.setI(f_kI);
    feederController.setD(f_kD);
    feederController.setIZone(f_kIz);
    feederController.setFF(f_kFF);
    feederController.setOutputRange(f_kMinOutput, f_kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Speed", feederMotor.getEncoder().getVelocity());
  }

  public void startShooter(double shooterSpeed, double feederSpeed) {
    shooterController.setReference(shooterSpeed, ControlType.kSmartVelocity);
    feederController.setReference(feederSpeed, ControlType.kSmartVelocity);
  }

  public void setPower(double s_power, double f_power) {
    shooterLeftMotor.set(s_power);
    feederMotor.set(f_power);
  }

  public void stopShooter() {
    shooterLeftMotor.set(0);
    feederMotor.set(0);
  }

  public void hood(double hoodRotate) {
    hoodMotor.set(hoodRotate);
  }
}
