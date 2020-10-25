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

  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, MotorType.kBrushless);

  // PID Controller
  private CANPIDController shooterController = shooterLeftMotor.getPIDController();
  private CANPIDController feederController = feederMotor.getPIDController();

  public ShootingSystem() {
    shooterRightMotor.follow(shooterLeftMotor, true);

    double[] initialShooterConstants = { shooterPID.kP, shooterPID.kI, shooterPID.kD, shooterPID.kFF, shooterPID.kMin,
        shooterPID.kMax };
    Constants.distributePID(initialShooterConstants, shooterController);
    SmartDashboard.putNumberArray("Shooter PID Constants", initialShooterConstants);

    double[] initialFeederConstants = { feederPID.kP, feederPID.kI, feederPID.kD, feederPID.kFF, feederPID.kMax,
        feederPID.kMin };
    Constants.distributePID(initialFeederConstants, feederController);
    SmartDashboard.putNumberArray("Feeder PID Constants", initialFeederConstants);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterLeftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Feeder Speed", feederMotor.getEncoder().getVelocity());

    // Grab numbers from SmartDashboard and set to motors
    double[] defaultPID = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double[] newShooterPIDconstants = SmartDashboard.getNumberArray("Shooter PID Constants", defaultPID);
    double[] newFeederPIDconstants = SmartDashboard.getNumberArray("Feeder PID Constants", defaultPID);
    Constants.distributePID(newShooterPIDconstants, shooterController);
    Constants.distributePID(newFeederPIDconstants, feederController);

    // Grab constants from motors and post to SmartDashboard
    double[] shooterPIDconstants = { shooterController.getP(), shooterController.getI(), shooterController.getD(),
        shooterController.getFF(), shooterController.getOutputMin(), shooterController.getOutputMax() };
    double[] feederPIDconstants = { feederController.getP(), feederController.getI(), feederController.getD(),
        feederController.getFF(), feederController.getOutputMin(), feederController.getOutputMax() };
    SmartDashboard.putNumberArray("Shooter PID Constants", shooterPIDconstants);
    SmartDashboard.putNumberArray("Feeder PID Constants", feederPIDconstants);
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
