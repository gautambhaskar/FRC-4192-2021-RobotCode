/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hood extends SubsystemBase {
  /**
   * Creates a new Hood.
   */
  private final DoubleSolenoid hoodSolenoid = new DoubleSolenoid(0, 0, 0);
  private NetworkTableEntry hoodPositionEntry;

  public Hood() {
    hoodPositionEntry = Shuffleboard.getTab("Data Tab").add("Hood Up?", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up() {
    hoodSolenoid.set(Value.kForward);
    hoodPositionEntry.setBoolean(true);
  }

  public void down() {
    hoodSolenoid.set(Value.kReverse);
    hoodPositionEntry.setBoolean(false);
  }
}
