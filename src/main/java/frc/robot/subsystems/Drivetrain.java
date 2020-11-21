/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  private static final ADIS16448_IMU imu = new ADIS16448_IMU();
  private double init_angle;

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

  NetworkTableEntry leftRPM, rightRPM, robotAngle;

  public Drivetrain() {
    // declare any encoders/odemetry stuff here...
    init_angle = imu.getAngle();

    leftRPM = tab.add("Drivetrain Left RPM", leftLead.getEncoder().getVelocity()).getEntry();
    rightRPM = tab.add("Drivetrain Right RPM", rightLead.getEncoder().getVelocity()).getEntry();
    robotAngle = tab.add("Robot Angle", imu.getAngle()).getEntry();
  }

  @Override
  public void periodic() {
    leftRPM.setDouble(leftLead.getEncoder().getVelocity());
    rightRPM.setDouble(rightLead.getEncoder().getVelocity());
    robotAngle.setDouble(imu.getAngle());
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double turn) {
    m_drive.arcadeDrive(-fwd, -turn); // <<<<<<Need to invert motors instead >>>>>>>
  }

  public double returnAngle() {
    return (imu.getAngle() - init_angle); // Replace 0 w sensor val
  }
}
