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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax flywheelOne;
  // private CANSparkMax flywheelTwo;

  private CANPIDController flywheelPID;
  
  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  // private NetworkTableEntry maxSpeed =
  //   tab.add("Max Speed", 1)
  //   .withWidget(BuiltInWidgets.kNumberSlider)
  //   .withProperties(Map.of("min", 0, "max", 1))
  //   .getEntry();
  private NetworkTableEntry targetRPM =
    tab.add("RPM", Constants.Shooter_Constants.maxRPM)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 100, "max", Constants.Shooter_Constants.maxRPM))
    .getEntry();

  private NetworkTableEntry shooterVelocity =
    tab.add("Shooter Velocity", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .getEntry();
  private NetworkTableEntry appliedOutput =
    tab.add("Applied Output", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .getEntry();

  private ShuffleboardLayout shufflePID =
    tab.getLayout("Shooter PID", BuiltInLayouts.kGrid)
    .withSize(2,3);
  private NetworkTableEntry kP =
    shufflePID.add("kP", Constants.Shooter_Constants.PID_Values.kP).withPosition(0, 0).getEntry();
  private NetworkTableEntry kI =
    shufflePID.add("kI", Constants.Shooter_Constants.PID_Values.kI).withPosition(0,1).getEntry();
  private NetworkTableEntry kD =
    shufflePID.add("kD", Constants.Shooter_Constants.PID_Values.kD).withPosition(0,2).getEntry();
  private NetworkTableEntry kIz =
    shufflePID.add("kIz", Constants.Shooter_Constants.PID_Values.kIz).withPosition(0,3).getEntry();
  private NetworkTableEntry kFF =
    shufflePID.add("kFF", Constants.Shooter_Constants.PID_Values.kFF).withPosition(1,0).getEntry();
  private NetworkTableEntry kMaxOutput =
    shufflePID.add("kMax", Constants.Shooter_Constants.PID_Values.kMax).withPosition(1,1).getEntry();
  private NetworkTableEntry kMinOutput =
    shufflePID.add("kMin", Constants.Shooter_Constants.PID_Values.kMin).withPosition(1,2).getEntry();

  public Shooter() {
    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);

    flywheelOne.restoreFactoryDefaults();
    flywheelOne.setInverted(true);
    // flywheelTwo.follow(flywheelOne, true);

    flywheelPID = flywheelOne.getPIDController();

    flywheelPID.setP(kP.getDouble(0));
    flywheelPID.setI(kI.getDouble(0));
    flywheelPID.setD(kD.getDouble(0));
    flywheelPID.setIZone(kIz.getDouble(0));
    flywheelPID.setFF(kFF.getDouble(0));
    flywheelPID.setOutputRange(kMinOutput.getDouble(0), kMaxOutput.getDouble(0));
  }

  @Override
  public void periodic() {
    double p = kP.getDouble(0);
    double i = kI.getDouble(0);
    double d = kD.getDouble(0);
    double iz = kIz.getDouble(0);
    double ff = kFF.getDouble(0);
    double max = kMaxOutput.getDouble(0);
    double min = kMinOutput.getDouble(0);

    if((p != kP.getDouble(0))) { flywheelPID.setP(p); kP.setDouble(p); }
    if((p != kP.getDouble(0))) { flywheelPID.setI(i); kP.setDouble(i); }
    if((p != kP.getDouble(0))) { flywheelPID.setD(d); kP.setDouble(d); }
    if((p != kP.getDouble(0))) { flywheelPID.setIZone(iz); kP.setDouble(iz); }
    if((p != kP.getDouble(0))) { flywheelPID.setFF(ff); kP.setDouble(ff); }
    if((max != kMaxOutput.getDouble(0)) || (min != kMinOutput.getDouble(0))) {
      flywheelPID.setOutputRange(min, max);
      kMinOutput.setDouble(min); kMaxOutput.setDouble(max);
    }

    updateShuffleboard();
  }

  // public void changePower(String direction) {
  //   double max = maxSpeed.getDouble(1.0);

  //   switch(direction) {
  //     case "up": {
  //       if (max < 1.0) maxSpeed.setDouble(max + 0.1);
  //       break;
  //     }
  //     case "down": {
  //       if (max > 0) maxSpeed.setDouble(max - 0.1);
  //       break;
  //     }
  //     default: break;
  //   }
  // }

  public void changeRPM(String direction) {
    double _rpm = targetRPM.getDouble(1.0);

    switch(direction) {
      case "up": {
        if (_rpm < Constants.Shooter_Constants.maxRPM) targetRPM.setDouble(_rpm + 100);
        break;
      }
      case "down": {
        if (_rpm > 100) targetRPM.setDouble(_rpm - 100);
        break;
      }
      default: break;
    }
  }
  
  public void stopShooter() {
    flywheelPID.setReference(0, ControlType.kVelocity);
  }

  public void startShooter() {
    double rpm = targetRPM.getDouble(Constants.Shooter_Constants.maxRPM);
    flywheelPID.setReference(rpm, ControlType.kVelocity);
  }

  private void updateShuffleboard() {
    shooterVelocity.setDouble(flywheelOne.getEncoder().getVelocity());
    appliedOutput.setDouble(flywheelOne.getAppliedOutput());
  }
}
