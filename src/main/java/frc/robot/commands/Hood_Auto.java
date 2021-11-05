/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter_Hood;

public class Hood_Auto extends CommandBase {

  private Shooter_Hood shooterHood;
  private double targetPosition;
  private Timer timer = new Timer();

  public Hood_Auto(Shooter_Hood shooterHoodSubsystem, double position) {
    shooterHood = shooterHoodSubsystem;
    targetPosition = position;
    addRequirements(shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooterHood.isHomed()) {    // not safe to move if not homed
      timer.reset();
      timer.start();
      shooterHood.setTargetPosition(targetPosition);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterHood.setHoodPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't check closed loop error from the controller because this methos is called
    // before the movement starts and the error may be zero from the prior movement
    if (!shooterHood.isHomed() ||
        Math.abs(targetPosition - shooterHood.getPosition()) < Constants.Hood_Constants.hoodTolerance || 
        timer.hasElapsed(Constants.Hood_Constants.autoTimeout)) {
      return true;
    }
    return false;
  }
}
