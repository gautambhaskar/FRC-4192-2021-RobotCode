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

public class RotateConstantSpeed extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  private static final ADIS16448_IMU imu = new ADIS16448_IMU();
  private double init_angle;

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

  public void arcadeDrive(double fwd, double turn) {
    m_drive.arcadeDrive(fwd, turn); // <<<<<<Need to invert motors instead >>>>>>>
  }