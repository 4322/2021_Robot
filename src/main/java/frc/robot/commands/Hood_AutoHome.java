/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter_Hood;

public class Hood_AutoHome extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

   private Shooter_Hood shooterHood;

  public Hood_AutoHome(Shooter_Hood shooterHoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    shooterHood = shooterHoodSubsystem;
    addRequirements(shooterHood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooterHood.isHomed()) shooterHood.setHood(-1);
    else shooterHood.setHood(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterHood.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterHood.isHomed() && shooterHood.getPosition() >=
      Constants.Hood_Constants.hoodMaxDistance_talon - Constants.Hood_Constants.hoodTolerance;
  }
}
