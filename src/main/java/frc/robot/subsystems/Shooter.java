/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;

  private CANPIDController flywheelPID;

  // private CANPIDController pidController;

  private CANEncoder flywheelOne_Encoder;
  private CANEncoder flywheelTwo_Encoder;
  
  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry maxSpeed =
    tab.add("Max Speed", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();
  private NetworkTableEntry rpm =
    tab.add("RPM", Constants.Shooter_Constants.maxRPM)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 100, "max", Constants.Shooter_Constants.maxRPM))
    .getEntry();
  private NetworkTableEntry shooterVelocity =
    tab.add("Shoter Velocity", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .getEntry();

  public Shooter() {
    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.Shooter_Constants.flywheelTwoSpark_ID, MotorType.kBrushless);

    flywheelOne_Encoder = new CANEncoder(flywheelOne);
    flywheelTwo_Encoder = new CANEncoder(flywheelTwo);

    flywheelPID = flywheelOne.getPIDController();

    flywheelOne.setInverted(true);
    flywheelTwo.follow(flywheelOne, true);

    this.refreshPID();
    flywheelPID.setIZone(Constants.Shooter_Constants.PID_Values.kIz);
    flywheelPID.setFF(Constants.Shooter_Constants.PID_Values.kFF);
    flywheelPID.setOutputRange(Constants.Shooter_Constants.PID_Values.kMin, Constants.Shooter_Constants.PID_Values.kMax);
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    SmartDashboard.putNumber("Flywheel1", flywheelOne_Encoder.getVelocity());
    SmartDashboard.putNumber("Flywheel2", flywheelTwo_Encoder.getVelocity());

    shooterVelocity.setDouble(flywheelOne_Encoder.getVelocity());
  }

  // @Override
  // public void useOutput(double output, double setpoint) {
  //   // Use the output here
  // }

  // @Override
  // public double getMeasurement() {
  //   // Return the process variable measurement here
  //   return 0;
  // }

  public void changePower(String direction) {
    double max = maxSpeed.getDouble(1.0);

    switch(direction) {
      case "up": {
        if (max < 1.0) maxSpeed.setDouble(max + 0.1);
        break;
      }
      case "down": {
        if (max > 0) maxSpeed.setDouble(max - 0.1);
        break;
      }
      default: break;
    }
  }

  public void changeRPM(String direction) {
    double _rpm = rpm.getDouble(1.0);

    switch(direction) {
      case "up": {
        if (_rpm < Constants.Shooter_Constants.maxRPM) rpm.setDouble(_rpm + 100);
        break;
      }
      case "down": {
        if (_rpm > 100) rpm.setDouble(_rpm - 100);
        break;
      }
      default: break;
    }
  }
  
  public void stopShooter()
  {
    flywheelOne.set(0);
  }

  public void spinShooter()
  {
    double speed = maxSpeed.getDouble(0.5);
    flywheelOne.set(speed);
  }

  public void reachSetpoint(double setPoint)
  {
    // double _rpm = rpm.getDouble(Constants.Shooter_Constants.maxRPM);
    flywheelPID.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
  }

  public double getShooterEncoder_Position()
  {
    return ((flywheelOne_Encoder.getPosition() + flywheelTwo_Encoder.getPosition()) / 2);
  }

  public double getShooterEncoder_Velocity()
  {
    return ((flywheelOne_Encoder.getVelocity() + flywheelTwo_Encoder.getVelocity() / 2));
  }

  public void refreshPID() {
    flywheelPID.setP(SmartDashboard.getNumber("kP", Constants.Shooter_Constants.PID_Values.kP));
    flywheelPID.setI(SmartDashboard.getNumber("kI", Constants.Shooter_Constants.PID_Values.kI));
    flywheelPID.setD(SmartDashboard.getNumber("kD", Constants.Shooter_Constants.PID_Values.kD));
  }
}
