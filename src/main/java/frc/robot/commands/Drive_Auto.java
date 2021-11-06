/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.Timer;

public class Drive_Auto extends CommandBase {
  /**
   * Creates a new Drive_Manual.
   */

  private Timer timer = new Timer();

   private Drivebase drivebase;
   private double power;
   private double turn;
   private double time;


  public Drive_Auto(Drivebase driveSubsystem, double _power, double _turn, double _time) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivebase = driveSubsystem;
    power = _power;
    turn = _turn;
    time = _time;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    drivebase.arcadeDrive(power, turn, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivebase.arcadeDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
