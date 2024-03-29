/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivebase;

public class Drive_Manual extends CommandBase {
  /**
   * Creates a new Drive_Manual.
   */

   private Drivebase drivebase;
   private double power;
   private double turn;
   //private boolean quickTurnState;
   //private boolean crawlModeState;


  public Drive_Manual(Drivebase driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivebase = driveSubsystem;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    power = RobotContainer.pilot.leftStick.getY();
    turn = RobotContainer.pilot.rightStick.getX();
    if (Constants.demo || !Constants.driveEnabled) {
      power = 0;
      turn = 0;
    }
    drivebase.arcadeDrive(power, turn, true);

    //CurveDrive for Skills competition
    //quickTurnState = RobotContainer.pilot.rt.get();
    //crawlModeState = RobotContainer.pilot.lt.get();
    //drivebase.curveDrive(power, turn, quickTurnState, crawlModeState);
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
