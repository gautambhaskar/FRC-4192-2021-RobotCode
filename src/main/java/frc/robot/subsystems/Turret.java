/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.turretPID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  // PID Constants
  private NetworkTableEntry kP, kI, kD, kFF, kMax, kMin;

  // Motor
  private final CANSparkMax turretMotor = new CANSparkMax(Constants.turret, MotorType.kBrushed);

  // Controller

  // Encoder
  private final CANEncoder turretEncoder = turretMotor.getEncoder(EncoderType.kNoSensor, 0);

  // past PID constants
  private double[] pastPIDconstants;

  // tabs
  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

  private NetworkTable table;
  private NetworkTableEntry tx, ty, ta;
  private double camMode;

  NetworkTableEntry limelightX, limelightY, limelightA, visionMode, turretSetOutput, turretAppliedOutput;

  public Turret() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    limelightX = cameraTab.add("LimelightX", tx.getDouble(0)).getEntry();
    limelightY = cameraTab.add("LimelightY", ty.getDouble(0)).getEntry();
    limelightA = cameraTab.add("LimelightArea", ta.getDouble(0)).getEntry();

    visionMode = cameraTab.add("Camera Mode", "Camera").getEntry();

    turretSetOutput = tab.add("turret set output", 0).getEntry();
    turretAppliedOutput = tab.add("turret applied output", turretMotor.getAppliedOutput()).getEntry();
  }

  @Override
  public void periodic() {
    camMode = getCamMode();
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Turret Angular Position", initialPosition -
    // turretMotor.getEncoder().getPosition());
    limelightA.setDouble(area);
    limelightX.setDouble(x);
    limelightY.setDouble(y);
  }

  // gets camMode
  public double getCamMode() {
    camMode = table.getEntry("camMode").getDouble(0);
    return camMode;
  }

  // puts camera into smart dashboard
  public void switchCameraMode() {
    if (getCamMode() == 0) {
      table.getEntry("camMode").setDouble(1);
      visionMode.setString("Camera");
      // cameraTab.add("Camera Mode", "Camera");
    } else if (getCamMode() == 1) {
      table.getEntry("camMode").setDouble(0);
      visionMode.setString("Vision");
      // cameraTab.add("Camera Mode", "Vision");
    }
  }

  // Set Turret Speed
  public void turn(double turretSpeed) {
    turretMotor.set(turretSpeed); // didn't turn.
    turretSetOutput.setDouble(turretSpeed);
    turretAppliedOutput.setDouble(turretMotor.getAppliedOutput());
  }

  public double limelightOffset() {
    return tx.getDouble(0.0);
  }

  public void startAlign() {
    if (tx.getDouble(0.0) > 1) {
      turretMotor.set(-turretPID.kP * tx.getDouble(0));
      turretSetOutput.setDouble(turretPID.kP * tx.getDouble(0));
      turretAppliedOutput.setDouble(turretMotor.getAppliedOutput());
    } else if (tx.getDouble(0.0) < 1) {
      turretMotor.set(-turretPID.kP * tx.getDouble(0));
      turretSetOutput.setDouble(-turretPID.kP * tx.getDouble(0));
      turretAppliedOutput.setDouble(turretMotor.getAppliedOutput());
    }
  }
}
