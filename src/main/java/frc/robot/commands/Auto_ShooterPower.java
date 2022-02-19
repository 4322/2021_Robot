/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class Auto_ShooterPower extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Shooter shooter;
  private double m_rpm;


  public Auto_ShooterPower(Shooter shooterSubsystem, double rpm) {
    shooter = shooterSubsystem;
    m_rpm = rpm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(m_rpm, m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when flywheel up to speed.
  @Override
  public boolean isFinished() {
    return Math.abs(m_rpm - shooter.getSpeedBig()) <= Constants.Shooter_Constants.tolerance;
  }
}
