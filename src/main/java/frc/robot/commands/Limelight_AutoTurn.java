/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivebase;

public class Limelight_AutoTurn extends CommandBase {
  /**
   * Creates a new Limelight_AutoTurn.
   */

   private Drivebase drivebase;
   private Limelight limelight;
   private double Kp = Constants.Drivebase_Constants.PID_Values.kP; //Kp is 0 in PID_Values
   private double min_command = 0.05;
   private double left_command = 0;
   private double right_command = 0;
   private double steering_adjust = 0.0;


  public Limelight_AutoTurn(Drivebase driveSubsystem, Limelight limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivebase = driveSubsystem;
    limelight = limelightSubsystem;
    addRequirements(drivebase,limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    double tx = limelight.getX_Offset();
    double heading_error = -tx;
    if (tx > 1.0)
      {
        steering_adjust = Kp*heading_error - min_command;
      }
    else if (tx < 1.0)
      {
        steering_adjust = Kp*heading_error + min_command;
      }
      left_command += steering_adjust;
      right_command -= steering_adjust;
      drivebase.tankDrive(left_command, right_command);
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
