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

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // private final CANSparkMax turretMotor = new CANSparkMax(Constants.turret,
  // MotorType.kBrushed);

  // Controller
  // private CANPIDController turretController = turretMotor.getPIDController();
  // private final double initialPosition;

  // past PID constants
  private double[] pastPIDconstants;

  // tabs
  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private double camMode;

  public Turret() {
    // initialPosition = turretMotor.getEncoder().getPosition();
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    // turretController.setFeedbackDevice(tx.getDouble(0));

    // kP = tuningTab.add("Turret kP", turretController.getP()).getEntry();
    // kI = tuningTab.add("Turret kI", turretController.getI()).getEntry();
    // kD = tuningTab.add("Turret kD", turretController.getD()).getEntry();
    // kFF = tuningTab.add("Turret kFF", turretController.getFF()).getEntry();
    // kMin = tuningTab.add("Turret kMin",
    // turretController.getOutputMin()).getEntry();
    // kMax = tuningTab.add("Turret kMax",
    // turretController.getOutputMax()).getEntry();
    // pastPIDconstants = new double[] { turretController.getP(),
    // turretController.getI(), turretController.getD(),
    // turretController.getFF(), turretController.getOutputMin(),
    // turretController.getOutputMax() };

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
    cameraTab.add("LimelightX", x);
    cameraTab.add("LimelightY", y);
    cameraTab.add("LimelightArea", area);

    double[] newTurretPIDconstants = { kP.getDouble(0), kI.getDouble(0), kD.getDouble(0), kFF.getDouble(0),
        kMin.getDouble(0), kMax.getDouble(0) };

    // if (Arrays.equals(newTurretPIDconstants, pastPIDconstants) == false) {
    // pastPIDconstants = newTurretPIDconstants;
    // Constants.distributePID(newTurretPIDconstants, turretController);
    // }

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
      cameraTab.add("Camera Mode", "Camera");
    } else if (getCamMode() == 1) {
      table.getEntry("camMode").setDouble(0);
      cameraTab.add("Camera Mode", "Vision");
    }
  }

  // Set Turret Speed
  public void turn(double turretSpeed) {
    // turretMotor.set(turretSpeed); // didn't turn.
    // tab.add("turret set output", turretSpeed); // didn't show anything besides 0.
    // tab.add("turret applied output", turretMotor.getAppliedOutput()); // showed
    // 10
  }

  public double limelightOffset() {
    return tx.getDouble(0.0);
  }

  public void startAlign() {
    if (tx.getDouble(0.0) > 1) {
      // turretMotor.set(turretPID.kP*tx.getDouble(0));
      // tab.add("turret set output", turretPID.kP*tx.getDouble(0));
      // tab.add("turret applied output", turretMotor.getAppliedOutput());
    } else if (tx.getDouble(0.0) < 1) {
      // turretMotor.set(-turretPID.kP*tx.getDouble(0));
      // tab.add("turret set output", -turretPID.kP*tx.getDouble(0));
      // tab.add("turret applied output", turretMotor.getAppliedOutput());
    }
  }
}
