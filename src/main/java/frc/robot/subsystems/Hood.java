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

public class Hood extends SubsystemBase {
  /**
   * Creates a new Hood.
   */
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, MotorType.kBrushless);

  private ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");

  private NetworkTableEntry hoodEngaged;

  public Hood() {
    hoodEngaged = subsystemTab.add("Hood engaged", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hood(double hoodRotateSpeed) {
    hoodMotor.set(hoodRotateSpeed);
  }
}
