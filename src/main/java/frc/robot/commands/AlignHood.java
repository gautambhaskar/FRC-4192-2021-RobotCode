// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.shooterModel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AlignHood extends CommandBase {
  /** Creates a new AlignHood. */
  private Turret turret;
  private Hood hood;
  private double area;
  private double hoodPosition;
  
  public AlignHood(Turret m_turret, Hood m_hood) {
    turret = m_turret;
    hood = m_hood;
    area = turret.limelightArea();
    hoodPosition = shooterModel.a * (Math.pow(area,2)) + shooterModel.b * (area) + shooterModel.c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.getPIDController().setReference(hoodPosition, ControlType.kPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(hoodPosition-hood.getPosition()) < shooterModel.tolerance;
  }
}
