/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.Arm_Manual;
import frc.robot.commands.Collecter_Collect;
import frc.robot.commands.Collector_Eject;
import frc.robot.commands.Collector_Stop;
import frc.robot.commands.Disable_Kicker;
import frc.robot.commands.Disable_Shooter;
import frc.robot.commands.Drive_Manual;
import frc.robot.commands.Enable_Kicker;
import frc.robot.commands.Enable_Shooter;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extend_Climber;
import frc.robot.commands.Hood_Manual;
import frc.robot.commands.Hopper_Eject;
import frc.robot.commands.Hopper_Intake;
import frc.robot.commands.Hopper_Stop;
import frc.robot.commands.Retract_Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter_Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Limelight limelight = new Limelight();
  public final Drivebase drivebase = new Drivebase();
  public final Shooter shooter = new Shooter();
  public final Shooter_Hood shooterHood = new Shooter_Hood();
  public final Kicker kicker = new Kicker();
  public final Collector collector = new Collector();
  public final Arm arm = new Arm();
  public final Climber climber = new Climber();
  public final Hopper hopper = new Hopper();
  

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final Drive_Manual driveManual = new Drive_Manual(drivebase);

  public final Hood_Manual hoodManual = new Hood_Manual(shooterHood);

  public final Enable_Shooter enableShooter = new Enable_Shooter(shooter);
  public final Disable_Shooter disableShooter = new Disable_Shooter(shooter);

  public final Enable_Kicker enableKicker = new Enable_Kicker(kicker);
  public final Disable_Kicker disableKicker = new Disable_Kicker(kicker);

  public final Collecter_Collect collectorCollect = new Collecter_Collect(collector);
  public final Collector_Eject collectorEject = new Collector_Eject(collector);
  public final Collector_Stop collectorStop = new Collector_Stop(collector);

  public final Extend_Climber extendClimber = new Extend_Climber(climber);
  public final Retract_Climber retractClimber = new Retract_Climber(climber);
  
  public final Arm_Manual armManual = new Arm_Manual(arm);

  public final Hopper_Intake hopperIntake = new Hopper_Intake(hopper);
  public final Hopper_Eject hopperEject = new Hopper_Eject(hopper);
  public final Hopper_Stop hopperStop = new Hopper_Stop(hopper);

  public static frc.robot.XboxController pilot = new frc.robot.XboxController(0);
  public static frc.robot.XboxController coPilot = new frc.robot.XboxController(1);

  


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivebase.setDefaultCommand(driveManual);
    shooterHood.setDefaultCommand(hoodManual);
    collector.setDefaultCommand(collectorStop);
    arm.setDefaultCommand(armManual);
    hopper.setDefaultCommand(hopperStop);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    coPilot.lb.whenPressed(enableShooter);
    coPilot.rb.whenPressed(disableShooter);

    coPilot.x.whenPressed(enableKicker);
    coPilot.b.whenPressed(disableKicker);
    
    coPilot.lt.whileHeld(hopperEject);
    coPilot.rt.whileHeld(hopperIntake);

    pilot.lt.whileHeld(collectorCollect, true);
    pilot.rt.whileHeld(collectorEject, true);

    coPilot.y.whenPressed(extendClimber);
    coPilot.a.whenPressed(retractClimber);

    

    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.Drivebase_Constants.PID_Values.ksVolts,
                                   Constants.Drivebase_Constants.PID_Values.kaVoltSecondsSquaredPerMeter,
                                   Constants.Drivebase_Constants.PID_Values.kvVoltSecondsPerMeter),
                                   Constants.Drivebase_Constants.kinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.Drivebase_Constants.kMaxSpeedMetersPerSecond, Constants.Drivebase_Constants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Drivebase_Constants.kinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory driveForward = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),  
    
    List.of
    (
      new Translation2d(.5, 0),
      new Translation2d(1, 0)
    ),
      new Pose2d(2, 0, new Rotation2d(0)), config);
    
    
    
      return m_autoCommand;
  }
}
