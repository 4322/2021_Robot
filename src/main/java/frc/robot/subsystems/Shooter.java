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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private SendableChooser<String> shooterControlMethod = new SendableChooser<String>();

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;

  private CANPIDController flywheelPID;
  private double flywheelRPM = 0;
  
  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  private NetworkTableEntry shooterEnabled =
    tab.add("Shooter Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,3)
    .getEntry();
  
  private NetworkTableEntry power =
    tab.add("Manual Power", 0.5)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .withPosition(3,3)
    .withSize(3,1)
    .getEntry();

  private NetworkTableEntry targetRPM =
    tab.add("RPM", Constants.Shooter_Constants.setPoint)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 100, "max", Constants.Shooter_Constants.maxRPM))
    .withSize(3,1)
    .withPosition(3,0)
    .getEntry();

  private NetworkTableEntry shooterVelocity =
    tab.add("Shooter Velocity Graph", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withProperties(Map.of("unit", "RPM"))
    .withPosition(0,0)
    .withSize(3,3)
    .getEntry();

  private NetworkTableEntry setpointError =
    tab.add("Setpoint Error", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withProperties(Map.of("unit", "RPM difference"))
    .withPosition(6,0)
    .withSize(3,3)
    .getEntry();

  private ShuffleboardLayout shufflePID =
    tab.getLayout("Shooter PID", BuiltInLayouts.kGrid)
    .withProperties(Map.of("Label position", "RIGHT"))
    .withSize(3,2)
    .withPosition(3,1);
  private NetworkTableEntry kP =
    shufflePID.add("kP", Constants.Shooter_Constants.PID_Values.kP).withPosition(0,0).getEntry();
  private NetworkTableEntry kI =
    shufflePID.add("kI", Constants.Shooter_Constants.PID_Values.kI).withPosition(1,0).getEntry();
  private NetworkTableEntry kD =
    shufflePID.add("kD", Constants.Shooter_Constants.PID_Values.kD).withPosition(2,0).getEntry();
  private NetworkTableEntry kIz =
    shufflePID.add("kIz", Constants.Shooter_Constants.PID_Values.kIz).withPosition(1,1).getEntry();
  private NetworkTableEntry kFF =
    shufflePID.add("kFF", Constants.Shooter_Constants.PID_Values.kFF).withPosition(0,2).getEntry();
  private NetworkTableEntry kMaxOutput =
    shufflePID.add("kMax", Constants.Shooter_Constants.PID_Values.kMax).withPosition(1,2).getEntry();
  private NetworkTableEntry kMinOutput =
    shufflePID.add("kMin", Constants.Shooter_Constants.PID_Values.kMin).withPosition(2,2).getEntry();
  private NetworkTableEntry pidEnabled =
    shufflePID.add("PID Enabled", false).withPosition(2,1).getEntry();

  private NetworkTableEntry setpointErrorVal =
    shufflePID.add("Error", 0).withPosition(0,1).getEntry();

  public Shooter() {
    shooterControlMethod.setDefaultOption("PID Control (RPM)", "pid");
    shooterControlMethod.addOption("Manual Control (power)", "manual");
    tab.add("Shooter Control", shooterControlMethod)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(1,3)
      .withSize(2,1);

    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.Shooter_Constants.flywheelTwoSpark_ID, MotorType.kBrushless);

    flywheelOne.restoreFactoryDefaults();
    flywheelOne.setInverted(true);
    flywheelTwo.restoreFactoryDefaults();
    flywheelTwo.follow(flywheelOne, true);

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

    // Refresh PID values from Shuffleboard
    if(p != flywheelPID.getP()) { flywheelPID.setP(p); }
    if(i != flywheelPID.getI()) { flywheelPID.setI(i); }
    if(d != flywheelPID.getD()) { flywheelPID.setD(d); }
    if(iz != flywheelPID.getIZone()) { flywheelPID.setIZone(iz); }
    if(ff != flywheelPID.getFF()) { flywheelPID.setFF(ff); }
    if(max != flywheelPID.getOutputMax() || (min != flywheelPID.getOutputMin())) {
      flywheelPID.setOutputRange(min, max);
    }

    double rpm = targetRPM.getDouble(0);
    if(rpm != flywheelRPM && pidEnabled.getBoolean(false)) {
      setShooterRPM(rpm, true);
    }

    updateShuffleboard();
  }

  public void changeSpeed(String direction) {
    double _power = power.getDouble(1);
    double _rpm = targetRPM.getDouble(Constants.Shooter_Constants.setPoint);

    switch(shooterControlMethod.getSelected()) {
      case "manual": {
        switch(direction) {
          case "up": {
            if ((_power + 0.2) <= 1) power.setDouble(_power + 0.1);
            else power.setDouble(1);
            break;
          }
          case "down": {
            if ((_power - 0.2) >= 0) power.setDouble(_power - 0.1);
            else power.setDouble(0);
            break;
          }
          default: break;
        }
        break;
      }
      case "pid": {
        switch(direction) {
          case "up": {
            if ((_rpm + 100) <= Constants.Shooter_Constants.maxRPM) targetRPM.setDouble(_rpm + 100);
            else targetRPM.setDouble(Constants.Shooter_Constants.maxRPM);
            break;
          }
          case "down": {
            if ((_rpm - 100) >= 100) targetRPM.setDouble(_rpm - 100);
            else targetRPM.setDouble(100);
            break;
          }
          default: break;
        }
        break;
      }
      default: break;
    }

    
  }
  
  public void stopShooter() {
    flywheelOne.stopMotor();
    setpointError.setDouble(0);
    setpointErrorVal.setDouble(0);
    pidEnabled.setBoolean(false);

    shooterEnabled.setBoolean(false);
    RobotContainer.coPilot.setRumble(0);
  }

  public void startShooter() {
    switch(shooterControlMethod.getSelected()) {
      case "pid": {
        double rpm = targetRPM.getDouble(Constants.Shooter_Constants.setPoint);
        setShooterRPM(rpm, false);
        double _setpointError = flywheelOne.getEncoder().getVelocity() - rpm;
        setpointError.setDouble(_setpointError);
        setpointErrorVal.setDouble(_setpointError);
        
        break;
      }
      case "manual": {
        if (pidEnabled.getBoolean(true)) {
          setpointError.setDouble(0);
          pidEnabled.setBoolean(false);
        }
        flywheelOne.set(power.getDouble(0.5));
        break;
      }
      default: break;
    }
    
    if (!shooterEnabled.getBoolean(false)) shooterEnabled.setBoolean(true);
    RobotContainer.coPilot.setRumble(0.1);
  }

  private void setShooterRPM(double rpm, boolean override) {
    if (!pidEnabled.getBoolean(false) || override) {
      flywheelPID.setReference(rpm, ControlType.kVelocity);
      pidEnabled.setBoolean(true);
    }
  }

  private void updateShuffleboard() {
    shooterVelocity.setDouble(flywheelOne.getEncoder().getVelocity());
  }
}
