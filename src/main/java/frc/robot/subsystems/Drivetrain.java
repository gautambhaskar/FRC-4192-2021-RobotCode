/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private final CANSparkMax leftLead = new CANSparkMax(Constants.leftLeader, MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(Constants.rightLeader, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftLead,
      new CANSparkMax(Constants.leftFollower1, MotorType.kBrushless),
      new CANSparkMax(Constants.leftFollower2, MotorType.kBrushless));
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightLead,
      new CANSparkMax(Constants.rightFollower1, MotorType.kBrushless),
      new CANSparkMax(Constants.rightFollower2, MotorType.kBrushless));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");

  private AHRS ahrs;
  private double init_angle;

  public Drivetrain() {
    // declare any encoders/odemetry stuff here...
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      init_angle = ahrs.getAngle();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    tab.add("Drivetrain Left RPM", leftLead.getEncoder().getVelocity());
    tab.add("Drivetrain Right RPM", rightLead.getEncoder().getVelocity());
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double turn) {
    m_drive.arcadeDrive(-fwd, -turn); // <<<<<<Need to invert motors instead >>>>>>>
  }

  public double returnAngle() {
    return (ahrs.getAngle() - init_angle);
  }
}
