/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // private SendableChooser<String> shooterControlMethod = new SendableChooser<String>();

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;

  private CANEncoder flywheelOneEncoder;
  private CANPIDController flywheelOnePID;

  private CANEncoder flywheelTwoEncoder;
  private CANPIDController flywheelTwoPID;
  
  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private NetworkTableEntry currentRPMBig =
    tab.add("Current RPM", 0)
    .withPosition(1,0)
    .withSize(1,1)
    .getEntry();
    
    private NetworkTableEntry currentRPMSmall =
    tab.add("Current RPM", 0)
    .withPosition(1,1)
    .withSize(1,1)
    .getEntry();

    private NetworkTableEntry targetRPMBig =
    tab.add("Target RPM", 0)
    .withPosition(2,0)
    .withSize(1,1)
    .getEntry();

    private NetworkTableEntry targetRPMSmall =
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
    flywheelOne.setIdleMode(IdleMode.kCoast);
    flywheelTwo.setIdleMode(IdleMode.kCoast);
    flywheelOne.setClosedLoopRampRate(Constants.Shooter_Constants.closedLoopRampRate);  // don't eject the shooter
    flywheelOne.setClosedLoopRampRate(Constants.Shooter_Constants.closedLoopRampRate);

    flywheelOneEncoder = flywheelOne.getEncoder();
    flywheelOnePID = flywheelOne.getPIDController();

    flywheelTwoEncoder = flywheelTwo.getEncoder();
    flywheelTwoPID = flywheelTwo.getPIDController();

    flywheelOnePID.setP(Constants.Shooter_Constants.PID_Values.kP);
    flywheelOnePID.setI(Constants.Shooter_Constants.PID_Values.kI);
    flywheelOnePID.setD(Constants.Shooter_Constants.PID_Values.kD);
    flywheelOnePID.setIZone(Constants.Shooter_Constants.PID_Values.kIz);
    flywheelOnePID.setFF(Constants.Shooter_Constants.PID_Values.kFF);
    flywheelOnePID.setOutputRange(Constants.Shooter_Constants.PID_Values.kMin, Constants.Shooter_Constants.PID_Values.kMax);

    flywheelTwoPID.setP(Constants.Shooter_Constants.PID_Values.kP);
    flywheelTwoPID.setI(Constants.Shooter_Constants.PID_Values.kI);
    flywheelTwoPID.setD(Constants.Shooter_Constants.PID_Values.kD);
    flywheelTwoPID.setIZone(Constants.Shooter_Constants.PID_Values.kIz);
    flywheelTwoPID.setFF(Constants.Shooter_Constants.PID_Values.kFF);
    flywheelTwoPID.setOutputRange(Constants.Shooter_Constants.PID_Values.kMin, Constants.Shooter_Constants.PID_Values.kMax);
  }

  @Override
  public void periodic() {
    currentRPMBig.setDouble(getSpeedBig());
    currentRPMSmall.setDouble(getSpeedSmall());
  }

  public void setSpeed(double rpm1, double rpm2) {
    flywheelOnePID.setReference(rpm1, ControlType.kVelocity);
    targetRPMBig.setDouble(rpm1);
    flywheelTwoPID.setReference(rpm2, ControlType.kVelocity);
    targetRPMSmall.setDouble(rpm2);
  }

  public double getSpeedBig() {
    return flywheelOneEncoder.getVelocity();
  }

  public double getSpeedSmall() {
    return flywheelTwoEncoder.getVelocity();
  }

  // don't let balls get stuck in the shooter
  public boolean isAbleToEject() {
    return getSpeedBig() >= Constants.Shooter_Constants.minEjectVel;
  }
  
  public void stopShooter() {
    flywheelOne.stopMotor();
  }
}