/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  /**
   * Creates a new Index.
   */
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.index, MotorType.kBrushless);

  public Index() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void run(double indexSpeed) {
    indexMotor.set(indexSpeed);
  }
}
