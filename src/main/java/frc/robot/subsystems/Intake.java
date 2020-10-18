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

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intake, MotorType.kBrushless);
  // private final DoubleSolenoid solenoid1 = new DoubleSolenoid(3, 4);
  // private final DoubleSolenoid solenoid2 = new DoubleSolenoid(2, 5);
  // private final DoubleSolenoid solenoid3 = new DoubleSolenoid(1, 6);
  // private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 7);

  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake RPM", intakeMotor.getEncoder().getVelocity());
  }

  public void intake(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public void raise() {
    // intakeSolenoid.set(Value.kReverse);
  }

  public void lower() {
    // intakeSolenoid.set(Value.kForward);
  }
}
