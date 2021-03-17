/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intake, MotorType.kBrushless);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(16, 0, 7);
  private ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");

  private NetworkTableEntry intakeEngaged;

  public Intake() {
    intakeEngaged = subsystemTab.add("Intake engaged", false).getEntry();
    intakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // runs the intake wheels/motors
  public void intake(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  // lifts up intake by activating pneumatics
  public void raise() {
    intakeSolenoid.set(Value.kReverse);
    intakeEngaged.setBoolean(true);
  }

  public void setIntake(boolean up) {
    if (up) {
      intakeSolenoid.set(Value.kReverse);
    } else {
      intakeSolenoid.set(Value.kForward);
    }
  }

  // brings down intake by releasing pneumatics
  public void lower() {
    intakeSolenoid.set(Value.kForward);
    intakeEngaged.setBoolean(false);
  }
}
