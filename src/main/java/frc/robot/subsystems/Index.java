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
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  /**
   * Creates a new Index.
   */
  //private final CANSparkMax indexMotor = new CANSparkMax(Constants.index, MotorType.kBrushless);
  private ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");

  private NetworkTableEntry indexEngaged;

  public Index() {
    indexEngaged = subsystemTab.add("Index engaged", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double indexSpeed) { 
    // runs motor and changes the value of the widget based on index status
    //indexMotor.set(indexSpeed);
    if (indexSpeed != 0) {
      indexEngaged.setBoolean(true);
    } else {
      indexEngaged.setBoolean(false);
    }
  }
}
