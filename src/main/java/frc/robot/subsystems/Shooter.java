/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // private SendableChooser<String> shooterControlMethod = new SendableChooser<String>();

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;

  private CANEncoder flywheelEncoder;
  private CANPIDController flywheelPID;
  
  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  // private NetworkTableEntry shooterEnabled =
  //   tab.add("Shooter Enabled", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .withPosition(0,3)
  //   .getEntry();
  
  private NetworkTableEntry power =
    tab.add("Power", 0)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", 0, "max", 1))
    .withPosition(0,0)
    .withSize(1,1)
    .getEntry();

  private NetworkTableEntry currentRPM =
    tab.add("Current RPM", 0)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", 100, "max", Constants.Shooter_Constants.maxRPM))
    .withPosition(1,0)
    .withSize(1,1)
    .getEntry();

  private NetworkTableEntry targetRPM =
    tab.add("Target RPM", 0)
    .withPosition(2,0)
    .withSize(1,1)
    .getEntry();

  public Shooter() {
    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.Shooter_Constants.flywheelTwoSpark_ID, MotorType.kBrushless);

    flywheelOne.restoreFactoryDefaults();
    flywheelOne.setInverted(true);
    flywheelTwo.restoreFactoryDefaults();
    flywheelTwo.follow(flywheelOne, true);

    flywheelEncoder = flywheelOne.getEncoder();
    flywheelPID = flywheelOne.getPIDController();

    flywheelPID.setP(Constants.Shooter_Constants.PID_Values.kP);
    flywheelPID.setI(Constants.Shooter_Constants.PID_Values.kI);
    flywheelPID.setD(Constants.Shooter_Constants.PID_Values.kD);
    flywheelPID.setIZone(Constants.Shooter_Constants.PID_Values.kIz);
    flywheelPID.setFF(Constants.Shooter_Constants.PID_Values.kFF);
    flywheelPID.setOutputRange(Constants.Shooter_Constants.PID_Values.kMin, Constants.Shooter_Constants.PID_Values.kMax);
  }

  @Override
  public void periodic() {
    power.setDouble(flywheelOne.getBusVoltage());
    currentRPM.setDouble(flywheelEncoder.getVelocity());
  }

  public void changeSpeed(double rpm) {
    flywheelPID.setReference(rpm, ControlType.kVelocity);
    targetRPM.setDouble(rpm);
  }
  
  public void stopShooter() {
    flywheelOne.stopMotor();
  }
}