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
import frc.robot.XboxController;

public class Enable_ShooterPower extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Shooter shooter;
  private double m_rpm;
  private XboxController m_copilot;


  public Enable_ShooterPower(Shooter shooterSubsystem, double rpm, XboxController copilot) {
    shooter = shooterSubsystem;
    m_rpm = rpm;
    m_copilot = copilot;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Inform copilot that shooter is up to speed
    if (Math.abs(m_rpm - shooter.getSpeed()) <= Constants.Shooter_Constants.tolerance) {
      m_copilot.setRumble(Constants.Shooter_Constants.rumbleIntensity);
    }
    else {
      m_copilot.setRumble(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_copilot.setRumble(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;     // run until interrupted
  }
}
