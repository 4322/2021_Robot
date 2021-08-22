/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter_Hood;

public class Hood_Manual extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

   private Shooter_Hood shooterHood;
   private double power;

  public Hood_Manual(Shooter_Hood shooterHoodSubsystem) {
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
    boolean pidEnabled = shooterHood.getPIDEnabled();
    if (pidEnabled == false) {
      power = RobotContainer.coPilot.leftStick.getY();
      shooterHood.setHood(power);
    }
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
