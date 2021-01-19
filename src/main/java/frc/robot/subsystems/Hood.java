/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterModel;

public class Hood extends SubsystemBase {
  /**
   * Creates a new Hood.
   */
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, MotorType.kBrushless);
  private final CANPIDController hoodController = hoodMotor.getPIDController();
  private final CANEncoder hoodEncoder = hoodMotor.getEncoder();
  
  private double initPosition;
  private NetworkTableEntry hoodPositionEntry;
  
  public Hood() {

    //set PID Constants on hoodController
    hoodController.setP(shooterModel.kP);
    hoodController.setI(shooterModel.kI);
    hoodController.setD(shooterModel.kD);
    hoodController.setFF(shooterModel.kFF);
    hoodController.setOutputRange(shooterModel.kMin, shooterModel.kMax);

    initPosition = hoodEncoder.getPosition();
    hoodPositionEntry = Shuffleboard.getTab("Data Tab").add("Hood Position", getPosition()).getEntry();
  }

  @Override
  public void periodic() {
    hoodPositionEntry.setDouble(getPosition());
    // This method will be called once per scheduler run
  }

  public void runMotor(double output) {
    hoodMotor.set(output);
  }
  public CANPIDController getPIDController() {
    return hoodController;
  }
  public double getPosition() {
    return (hoodEncoder.getPosition()-initPosition);
  }
}
