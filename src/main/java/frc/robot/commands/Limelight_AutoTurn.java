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
   private double steering_adjust = 0.0;
   private double heading_error = 0;


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
  public void execute() { // https://docs.limelightvision.io/en/latest/cs_aiming.html
    heading_error = -limelight.getX_Offset(); // positive error means clockwise turn
    steering_adjust = Constants.Drivebase_Constants.PID_Values.kP*heading_error;
    if (steering_adjust < Constants.Drivebase_Constants.PID_Values.minDriveRotationPower) 
      {
      steering_adjust = Constants.Drivebase_Constants.PID_Values.minDriveRotationPower;
      }
    drivebase.tankDrive(steering_adjust, -steering_adjust);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(heading_error) < Constants.Drivebase_Constants.PID_Values.aimingTolerance) {
      drivebase.tankDrive(0, 0);
      return true;
    } else {
      return false;
    }
  }
}