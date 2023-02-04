/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Timer disableTimer = new Timer();

  // Define controllers
  public static XboxController pilot = new XboxController(0);
  public static XboxController coPilot = new XboxController(1);
  
  // The robot's subsystems and commands are defined here...
  public final Limelight limelight = new Limelight();
  public final Drivebase drivebase = new Drivebase();
  public final Shooter shooter = new Shooter();
  public final Shooter_Hood shooterHood = new Shooter_Hood();
  public final Kicker kicker = new Kicker();
  public final Collector collector = new Collector();
  public final Arm arm = new Arm();
  public final Climber climber = new Climber();
  public final Hopper hopper = new Hopper();
  
  // Define robot commands
  public final Drive_Manual driveManual = new Drive_Manual(drivebase);

  public final Hood_Manual hoodManual = new Hood_Manual(shooterHood);
  public Command hoodReset;

  public final Disable_Shooter disableShooter = new Disable_Shooter(shooter);
  public final ParallelCommandGroup shootFromPos1 = new ParallelCommandGroup(
    new Hood_Auto(shooterHood, Constants.Hood_Constants.Positions.pos1), 
    new Enable_ShooterPower(shooter, Constants.Shooter_Constants.shooterVel1, coPilot));
  public final ParallelCommandGroup shootFromPos2 = new ParallelCommandGroup(
    new Hood_Auto(shooterHood, Constants.Hood_Constants.Positions.pos2), 
    new Enable_ShooterPower(shooter, Constants.Shooter_Constants.shooterVel2, coPilot));
  public final ParallelCommandGroup shootFromPos3 = new ParallelCommandGroup(
    new Hood_Auto(shooterHood, Constants.Hood_Constants.Positions.pos3), 
    new Enable_ShooterPower(shooter, Constants.Shooter_Constants.shooterVel3, coPilot));
  public final Enable_ShooterPower shooterTest = 
    new Enable_ShooterPower(shooter, 3000, coPilot);

  public final Enable_Kicker enableKicker = new Enable_Kicker(kicker, shooter);

  public final Collecter_Collect collectorCollect = new Collecter_Collect(collector);
  public final Collector_Eject collectorEject = new Collector_Eject(collector);
  public final Collector_Stop collectorStop = new Collector_Stop(collector);
  
  public final Arm_Toggle armToggle = new Arm_Toggle(arm);

  public final Hopper_Intake hopperIntake = new Hopper_Intake(hopper);
  public final Hopper_Eject hopperEject = new Hopper_Eject(hopper);
  public final Hopper_Stop hopperStop = new Hopper_Stop(hopper);

  public final Climber_Manual climberManual = new Climber_Manual(climber);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.demo) {
      hoodReset =  new SequentialCommandGroup(   // don't create at instaniation since min position not yet overridden
      new Hood_Reset(shooterHood),
      new Hood_Auto(shooterHood, Constants.Hood_Constants.hoodMinPosition)); //leaves hood half way up
    } else {
      hoodReset = new Hood_Reset(shooterHood);
    }
    configureButtonBindings();
    disableSubsystems();

    drivebase.setDefaultCommand(driveManual); 
    shooterHood.setDefaultCommand(hoodManual);
    collector.setDefaultCommand(collectorStop);
    hopper.setDefaultCommand(hopperStop);
    climber.setDefaultCommand(climberManual);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVEBASE CONTROLS
    pilot.dPad.up.whenPressed(() -> drivebase.changePower("up"));
    pilot.dPad.down.whenPressed(() -> drivebase.changePower("down"));

    // COLLECTOR CONTROLS
    pilot.rb.whileHeld(collectorCollect);
    pilot.lb.whileHeld(collectorEject);

    // HOPPER CONTROLS
    coPilot.lt.whileHeld(hopperEject);
    coPilot.rt.whileHeld(hopperIntake);
    
    // SHOOTER CONROLS
    if (Constants.demo) {
      coPilot.y.whenPressed(new Enable_ShooterPower(shooter, Constants.Shooter_Constants.demoVel1, coPilot));
      coPilot.x.whenPressed(new Enable_ShooterPower(shooter, Constants.Shooter_Constants.demoVel2, coPilot));
      coPilot.a.whenPressed(new Enable_ShooterPower(shooter, Constants.Shooter_Constants.demoVel3, coPilot));
    } else {
      coPilot.y.whenPressed(shootFromPos1);   // interruptable by default
      coPilot.x.whenPressed(shootFromPos2);
      coPilot.a.whenPressed(shootFromPos3);
    }
    coPilot.b.whenPressed(disableShooter);
    
    // KICKER CONTROLS
    coPilot.rt.whileHeld(enableKicker);
    
    // HOOD CONTROLS
    coPilot.back.whenPressed(hoodReset, false);   // move to limit switch without any interrupts

    // ARM CONTROLS
    pilot.a.whenPressed(armToggle);
  
    // CLIMBER CONTROLS
    // pilot.y.whenPressed(extendClimber);
    // pilot.a.whenPressed(retractClimber);
  }

  public void disableSubsystems() {
    drivebase.setBrakeMode();
    shooter.stopShooter();
    disableTimer.reset();
    disableTimer.start();
    shooterHood.setCoastMode();   // allow hood to be moved manually
    limelight.setLed(Limelight.LedMode.Off);
    coPilot.setRumble(0);
  }

  public void resetSubsystems() {
    drivebase.setBrakeMode();
    shooterHood.setBrakeMode();   // don't let hood move while shooting
    limelight.setLed(Limelight.LedMode.On);
    disableTimer.stop();
    disableTimer.reset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Constants.demo) {
      return null;
    }
    return new SequentialCommandGroup(
      new Hood_Reset(shooterHood),
      new Arm_Toggle(arm),
      new Hopper_IntakeAuto(hopper), 
      // set for 10 foot shot:
      new Auto_ShooterPower(shooter, Constants.Shooter_Constants.shooterVel2),
      new Hood_Auto(shooterHood, Constants.Hood_Constants.Positions.pos2),
      new Enable_KickerAuto(kicker, shooter),
      new WaitCommand(5.0), // wait for balls to shoot
      // disable subsystems
      new Disable_KickerAuto(kicker),
      new Disable_Shooter(shooter),
      new Hopper_StopAuto(hopper),
      // drive backward
      new Drive_Auto(drivebase, -0.3, 0, 1.0)
    );


    /*
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.Drivebase_Constants.PID_Values.ksVolts,
                                   Constants.Drivebase_Constants.PID_Values.kaVoltSecondsSquaredPerMeter,
                                   Constants.Drivebase_Constants.PID_Values.kvVoltSecondsPerMeter),
        Constants.Drivebase_Constants.kinematics, 10);

    TrajectoryConfig config =
        new TrajectoryConfig(Constants.Drivebase_Constants.kMaxSpeedMetersPerSecond,
                             Constants.Drivebase_Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Drivebase_Constants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivebase::getPose,
        new RamseteController(Constants.Drivebase_Constants.kRamseteB, Constants.Drivebase_Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.Drivebase_Constants.PID_Values.ksVolts,
                                   Constants.Drivebase_Constants.PID_Values.kvVoltSecondsPerMeter,
                                   Constants.Drivebase_Constants.PID_Values.kaVoltSecondsSquaredPerMeter),
        Constants.Drivebase_Constants.kinematics,
        drivebase::getWheelSpeeds,
        new PIDController(Constants.Drivebase_Constants.PID_Values.kPDriveVel, 0, 0),
        new PIDController(Constants.Drivebase_Constants.PID_Values.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivebase::tankDriveVolts,
        drivebase
    );

    // drivebase.zeroHeading();
    // Reset odometry to the starting pose of the trajectory.
    drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0));
    */
  }

  public void hoodReset() {
    if (!shooterHood.isHomed()) {
      hoodReset.schedule(false);   // move to limit switch without any interrupts
    }
  }

public void disabledPeriodic() {
  if (disableTimer.hasElapsed(Constants.Drivebase_Constants.disableBreakSec)) {
    drivebase.setCoastMode();  // robot has stopped, safe to enter coast mode
    disableTimer.stop();
    disableTimer.reset();
  }
}

}
