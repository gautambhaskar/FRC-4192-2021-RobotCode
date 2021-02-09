/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.twpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private double init_angle, init_original_angle;
  private double init_position;

  private final CANSparkMax leftLead = new CANSparkMax(Constants.leftLeader, MotorType.kBrushless); // 1
  private final CANSparkMax rightLead = new CANSparkMax(Constants.rightLeader, MotorType.kBrushless); // 5
  private final CANSparkMax leftFollower1 = new CANSparkMax(Constants.leftFollower1, MotorType.kBrushless); // 2
  // private final CANSparkMax leftFollower2 = new
  // CANSparkMax(Constants.leftFollower2, MotorType.kBrushless); //3
  private final CANSparkMax rightFollower1 = new CANSparkMax(Constants.rightFollower1, MotorType.kBrushless); // 6
  // private final CANSparkMax rightFollower2 = new
  // CANSparkMax(Constants.rightFollower2, MotorType.kBrushless); //7

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftLead, leftFollower1);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightLead, rightFollower1);

  private static final ADIS16448_IMU imu = new ADIS16448_IMU();

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final DifferentialDriveOdometry m_odometry;

  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

  NetworkTableEntry leftRPM, rightRPM, robotAngle, drivetrainPosition, drivetrainSpeed;

  public Drivetrain() {
    // declare any encoders/odemetry stuff here...
    init_angle = imu.getAngle();
    init_original_angle = imu.getAngle();
    init_position = leftLead.getEncoder().getPosition();

    // leftLead.getEncoder().setPositionConversionFactor(drivePID.positionConversionFactor);
    // leftLead.getEncoder().setVelocityConversionFactor(drivePID.positionConversionFactor);
    // rightLead.getEncoder().setPositionConversionFactor(drivePID.positionConversionFactor);
    // rightLead.getEncoder().setVelocityConversionFactor(drivePID.positionConversionFactor);

    leftLead.setInverted(true);
    leftLead.setIdleMode(IdleMode.kCoast);
    leftLead.getEncoder().setPosition(0);
    rightLead.getEncoder().setPosition(0);
    rightLead.setIdleMode(IdleMode.kCoast);
    rightLead.setInverted(true);
    leftFollower1.setInverted(true);
    leftFollower1.setIdleMode(IdleMode.kCoast);
    // leftFollower2.setInverted(true);
    rightFollower1.setInverted(true);
    rightFollower1.setIdleMode(IdleMode.kCoast);
    // rightFollower2.setInverted(true);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(returnNativeAngle()));

    leftRPM = tab.add("Drivetrain Left RPM", leftLead.getEncoder().getVelocity()).getEntry();
    rightRPM = tab.add("Drivetrain Right RPM", rightLead.getEncoder().getVelocity()).getEntry();
    robotAngle = tab.add("Robot Angle", imu.getAngle()-init_angle).getEntry();
    drivetrainSpeed = tuningTab.add("Drivetrain Speed", leftLead.getEncoder().getVelocity())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(5, 4).getEntry();
    drivetrainPosition = tuningTab.add("Drivetrain Position", init_position - leftLead.getEncoder().getPosition())
        .withWidget(BuiltInWidgets.kGraph).withSize(2, 2).withPosition(0, 4).getEntry();
  }

  @Override
  public void periodic() {
    leftRPM.setDouble(leftLead.getEncoder().getVelocity());
    rightRPM.setDouble(rightLead.getEncoder().getVelocity());
    // robotAngle.setDouble(init_angle - imu.getAngle());
    SmartDashboard.putNumber("Drivetrain Angle Diff", returnAngle());
    drivetrainSpeed.setDouble(leftLead.getEncoder().getVelocity());
    drivetrainPosition.setDouble(leftLead.getEncoder().getPosition()-init_position);
    // This method will be called once per scheduler run

    m_odometry.update(Rotation2d.fromDegrees(returnNativeAngle()), leftLead.getEncoder().getPosition(),
        rightLead.getEncoder().getPosition());
  }

  // makes a method to drive with parameters for forward speed and rotation
  public void arcadeDrive(double fwd, double turn) {
    m_drive.arcadeDrive(fwd, turn);
  }

  // sets up a tank drive given the voltage supplied to either side
  public void tankDrive(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  // gives the curent offset from the calibrated/recalibrated angle
  public double returnAngle() {
    return imu.getAngle()-init_angle;
    // return (init_angle - imu.getAngle()); // Replace 0 w sensor val
  }

  public Pose2d returnPose() {
    return m_odometry.getPoseMeters();
  }

  // gives the current offset from the original angle
  public double returnNativeAngle() {
    return imu.getAngle()-init_original_angle;
    // return (init_original_angle - imu.getAngle());
  }

  // shows the distance the robot has traveled relative to starting position
  public double returnDrivetrainPosition() {
    return (leftLead.getEncoder().getPosition()-init_position);
  }

  public double returnAverageEncoderDistance() {
    return (leftLead.getEncoder().getPosition() + rightLead.getEncoder().getPosition()) / 2.0;
  }

  public CANEncoder returnLeftEncoder() {
    return leftLead.getEncoder();
  }

  public CANEncoder returnRightEncoder() {
    return rightLead.getEncoder();
  }

  public double returnAngularRate() {
    return imu.getRate();
  }

  public DifferentialDriveWheelSpeeds returnWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLead.getEncoder().getVelocity(), rightLead.getEncoder().getVelocity());
  }

  // makes the current angle the initial angle
  public void recalibrateAngle() {
    init_angle = imu.getAngle();
  }

  // sets the current position as the new initial position
  public void recalibratePosition() {
    init_position = leftLead.getEncoder().getPosition();
  }

  public void resetEncoders() {
    leftLead.getEncoder().setPosition(0);
    rightLead.getEncoder().setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(returnNativeAngle()));
  }

}
