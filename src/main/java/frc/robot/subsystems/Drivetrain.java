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
// import edu.twpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  private static final ADIS16448_IMU imu = new ADIS16448_IMU();
  private double init_angle;
  private double init_position;

  private final CANSparkMax leftLead = new CANSparkMax(Constants.leftLeader, MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(Constants.rightLeader, MotorType.kBrushless);
  private final CANSparkMax leftFollower1 = new CANSparkMax(Constants.leftFollower1, MotorType.kBrushless);
  private final CANSparkMax leftFollower2 = new CANSparkMax(Constants.leftFollower2, MotorType.kBrushless);
  private final CANSparkMax rightFollower1 = new CANSparkMax(Constants.rightFollower1, MotorType.kBrushless);
  private final CANSparkMax rightFollower2 = new CANSparkMax(Constants.rightFollower2, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftLead, leftFollower1, leftFollower2);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightLead, rightFollower1,
      rightFollower2);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

  NetworkTableEntry leftRPM, rightRPM, robotAngle, drivetrainPosition, drivetrainSpeed;

  public Drivetrain() {
    // declare any encoders/odemetry stuff here...
    init_angle = imu.getAngle();

    leftRPM = tab.add("Drivetrain Left RPM", leftLead.getEncoder().getVelocity()).getEntry();
    rightRPM = tab.add("Drivetrain Right RPM", rightLead.getEncoder().getVelocity()).getEntry();
    robotAngle = tab.add("Robot Angle", init_angle - imu.getAngle()).getEntry();
    leftLead.setInverted(true);
    leftLead.getEncoder().setPosition(0);
    rightLead.setInverted(true);
    leftFollower1.setInverted(true);
    leftFollower2.setInverted(true);
    rightFollower1.setInverted(true);
    rightFollower2.setInverted(true);

    drivetrainSpeed = tuningTab.add("Drivetrain Speed", leftLead.getEncoder().getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(5, 4).getEntry();
    drivetrainPosition = tuningTab.add("Drivetrain Position", init_position - leftLead.getEncoder().getPosition())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(0, 4).getEntry();
  }

  @Override
  public void periodic() {
    leftRPM.setDouble(leftLead.getEncoder().getVelocity());
    rightRPM.setDouble(rightLead.getEncoder().getVelocity());
    robotAngle.setDouble(init_angle - imu.getAngle());
    drivetrainSpeed.setDouble(leftLead.getEncoder().getVelocity());
    drivetrainPosition.setDouble(init_position - leftLead.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }

  public void recalibrateAngle() {
    init_angle = imu.getAngle();
  }

  public void arcadeDrive(double fwd, double turn) {
    m_drive.arcadeDrive(fwd, turn); // <<<<<<Need to invert motors instead >>>>>>>
  }

  public double returnAngle() {
    return (init_angle - imu.getAngle()); // Replace 0 w sensor val
  }

  public double returnDrivetrainPosition() {
    return (init_position - leftLead.getEncoder().getPosition());
  }
  
  public void recalibratePosition() {
    init_position = leftLead.getEncoder().getPosition();
  }
}
