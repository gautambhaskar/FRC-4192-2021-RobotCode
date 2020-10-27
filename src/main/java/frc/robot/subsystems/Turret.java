/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */

  // Motor
  private final CANSparkMax turretMotor = new CANSparkMax(Constants.turret, MotorType.kBrushed);
  private final double initialPosition;
  // limelight
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private double camMode;

  public Turret() {
    initialPosition = turretMotor.getEncoder().getPosition();
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

  }

  @Override
  public void periodic() {
    camMode = getCamMode();
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angular Position", initialPosition - turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
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
      SmartDashboard.putString("Camera Mode", "Camera");
    } else if (getCamMode() == 1) {
      table.getEntry("camMode").setDouble(0);
      SmartDashboard.putString("Camera Mode", "Vision");
    }
  }

  // Set Turret Speed
  public void turn(double turretSpeed) {
    turretMotor.set(turretSpeed);
    SmartDashboard.putString("Message", "subsystem-level command run");
  }
}
