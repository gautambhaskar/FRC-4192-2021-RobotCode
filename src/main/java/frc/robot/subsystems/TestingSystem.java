/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TestingSystem extends SubsystemBase {
  /**
   * Creates a new TestingSystem.
   */
  private final CANSparkMax m_motor = new CANSparkMax(4, MotorType.kBrushless);
  private final ShuffleboardTab subsystemsTab = Shuffleboard.getTab("Subsystems");
  private NetworkTableEntry testMotorEngaged = subsystemsTab.add("Test Motor engaged", false).getEntry();

  public TestingSystem() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  // runs motor with power spd
  public void runMotor(double spd) {
    m_motor.set(spd);
    if (spd != 0) {
      testMotorEngaged.setBoolean(true);
    } else {
      testMotorEngaged.setBoolean(false);
    }
  }
}
