/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Enable_Kicker extends CommandBase {
  /**
   * Creates a new Enable_Kicker.
   */

   private Kicker kicker;

  public Enable_Kicker(Kicker kickerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    kicker = kickerSubsystem;
    addRequirements(kicker);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    kicker.enableKicker();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
