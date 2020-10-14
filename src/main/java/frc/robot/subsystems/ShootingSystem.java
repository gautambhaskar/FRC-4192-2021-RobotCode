/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
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

  // Motors
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.index, MotorType.kBrushless);
  private final CANSparkMax turretMotor = new CANSparkMax(Constants.index, MotorType.kBrushless);
  private final CANSparkMax feederMotor = new CANSparkMax(Constants.feeder, MotorType.kBrushless);
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.shooterLeft, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.shooterRight, MotorType.kBrushless);
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, MotorType.kBrushless);

  // Encoders
  private final CANEncoder shooterEncoder = shooterLeftMotor.getEncoder();

  private final CANPIDController shooterController = shooterLeftMotor.getPIDController();

  public ShootingSystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void index(double indexSpeed) {
    indexMotor.set(indexSpeed);
  }

  public void turret(double turretTurn) {
    turretMotor.set(turretTurn);
  }

  public void feeder(double feederSpeed) {
    feederMotor.set(feederSpeed);
  }

  public void shooter(double shooterSpeed) {
    shooterLeftMotor.set(shooterSpeed);
    shooterRightMotor.set(shooterSpeed);
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  public void hood(double hoodRotate) {
    hoodMotor.set(hoodRotate);
  }
}
