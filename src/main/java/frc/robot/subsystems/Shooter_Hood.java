/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {

  private WPI_TalonSRX shooterHood;
  private Encoder hoodEncoder;
  private PIDController hoodPIDController;
  // private Limelight limelight;

  // Hood control
  private boolean homed = false;
  private boolean pidEnabled = false;
  private double targetSetpoint = 0;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

  private NetworkTableEntry hoodPosition =
    tab.add("Hood Position", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", Constants.Hood_Constants.hoodMaxDistance))
    .withPosition(0,0)
    .withSize(2,2)
    .getEntry();
  private NetworkTableEntry hoodPower =
    tab.add("Hood Power", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .withPosition(2,0)
    .withSize(2,2)
    .getEntry();
  private NetworkTableEntry isHomeIndicator =
    tab.add("Is @ home", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,2)
    .withSize(1,1)
    .getEntry();
  private NetworkTableEntry isHomedIndicator =
    tab.add("Homed", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(1,2)
    .withSize(1,1)
    .getEntry();

  private ShuffleboardLayout hoodPID =
    tab.getLayout("Hood PID", BuiltInLayouts.kGrid)
    .withProperties(Map.of("Label position", "RIGHT"))
    .withSize(4,2)
    .withPosition(4,0);
  private NetworkTableEntry kP =
    hoodPID.add("kP", Constants.Hood_Constants.PID_Values.kP).withPosition(0,0).getEntry();
  private NetworkTableEntry kI =
    hoodPID.add("kI", Constants.Hood_Constants.PID_Values.kI).withPosition(1,0).getEntry();
  private NetworkTableEntry kD =
    hoodPID.add("kD", Constants.Hood_Constants.PID_Values.kD).withPosition(2,0).getEntry();
  private NetworkTableEntry errorTollerance =
    hoodPID.add("Error Tolerance", Constants.Hood_Constants.PID_Values.errorTollerance).withPosition(1,1).getEntry();
  private NetworkTableEntry errorDerivativeTolerance =
    hoodPID.add("Error Derivative Tolerance", Constants.Hood_Constants.PID_Values.errorDerivativeTolerance).withPosition(2,1).getEntry();
  // private NetworkTableEntry kIz =
  //   hoodPID.add("kIz", Constants.Shooter_Constants.PID_Values.kIz).withPosition(1,1).getEntry();
  // private NetworkTableEntry kFF =
  //   hoodPID.add("kFF", Constants.Shooter_Constants.PID_Values.kFF).withPosition(0,2).getEntry();
  // private NetworkTableEntry kMaxOutput =
  //   hoodPID.add("kMax", Constants.Shooter_Constants.PID_Values.kMax).withPosition(1,2).getEntry();
  // private NetworkTableEntry kMinOutput =
  //   hoodPID.add("kMin", Constants.Shooter_Constants.PID_Values.kMin).withPosition(2,2).getEntry();
  private NetworkTableEntry atSetpoint =
    hoodPID.add("At Setpoint", false).withPosition(0,1).getEntry();
  private NetworkTableEntry shufflePIDenabled =
    hoodPID.add("PID Enabled", false).withPosition(0,2).getEntry();
  private NetworkTableEntry shuffleTargetSetpoint =
    hoodPID.add("Target Setpoint", targetSetpoint).withPosition(1,2).getEntry();
  private NetworkTableEntry shuffleCurrentError =
    hoodPID.add("Setpoint Error", 0).withPosition(1,2).getEntry();

  public Shooter_Hood() {
    // The PIDController used by the subsystem
    hoodPIDController = new PIDController(
      Constants.Hood_Constants.PID_Values.kP,
      Constants.Hood_Constants.PID_Values.kI,
      Constants.Hood_Constants.PID_Values.kD
    );

    hoodPIDController.disableContinuousInput();
    // hoodPIDController.reset();

    hoodPIDController.setTolerance(
      Constants.Hood_Constants.PID_Values.errorTollerance,
      Constants.Hood_Constants.PID_Values.errorDerivativeTolerance
    );

    shooterHood = new WPI_TalonSRX(Constants.Hood_Constants.hoodTalon_ID);
    hoodEncoder = new Encoder(0, 1);
    // limelight = new Limelight();

    // Check if the hood is already in the home position
    if (isAtHome()) homed = true;
  }

  @Override
  public void periodic() {
    checkHome();

    hoodPosition.setDouble(getPosition());
    isHomeIndicator.setBoolean(isAtHome());
    isHomedIndicator.setBoolean(isHomed());

    double p = kP.getDouble(0);
    double i = kI.getDouble(0);
    double d = kD.getDouble(0);
    // double errTol = errorTollerance.getDouble(0);
    // double errDerivTol = errorDerivativeTolerance.getDouble(0);
    // double iz = kIz.getDouble(0);
    // double ff = kFF.getDouble(0);
    // double max = kMaxOutput.getDouble(0);
    // double min = kMinOutput.getDouble(0);

    // Refresh PID values from Shuffleboard
    if(p != hoodPIDController.getP()) { hoodPIDController.setP(p); }
    if(i != hoodPIDController.getI()) { hoodPIDController.setI(i); }
    if(d != hoodPIDController.getD()) { hoodPIDController.setD(d); }
    // if(iz != flywheelPID.getIZone()) { flywheelPID.setIZone(iz); }
    // if(ff != flywheelPID.getFF()) { flywheelPID.setFF(ff); }
    // if(max != flywheelPID.getOutputMax() || (min != flywheelPID.getOutputMin())) {
    //   flywheelPID.setOutputRange(min, max);
    // }

    // HOOD PID CONTROL
    if (pidEnabled && !hoodPIDController.atSetpoint()) setHoodPosition(targetSetpoint);
    else pidEnabled = false;
    shufflePIDenabled.setBoolean(pidEnabled);
    shuffleTargetSetpoint.setDouble(targetSetpoint);
    shuffleCurrentError.setDouble(hoodPIDController.getPositionError());
    atSetpoint.setBoolean(hoodPIDController.atSetpoint());
  }

  public double getPosition()
  {
    return hoodEncoder.getDistance() * -1; 
  }

  public void setHood(double power)
  {
    // if (pidEnabled) pidEnabled = false;

    hoodPower.setDouble(power);

    double encValue = this.getPosition();
    if (this.getPosition() <= Constants.Hood_Constants.hoodMaxDistance || power < 0) {
      if (this.getPosition() >= 4200 && power > 0) {
        double _power = power * ((Constants.Hood_Constants.hoodMaxDistance - (encValue))/600);
        if (_power < 0.1) {
          _power = 0.1;
        }
        shooterHood.set(_power);
      }
      else shooterHood.set(power);
    }
    else {
      shooterHood.stopMotor();
    }
  }

  public void setHoodPosition(double setpoint) {
    double output = MathUtil.clamp(hoodPIDController.calculate(getPosition(), setpoint), -1, 1);
    // System.out.println("Hood PID Output: " + output);
    shooterHood.set(output);
    hoodPower.setDouble(output);

    atSetpoint.setBoolean(hoodPIDController.atSetpoint());
  }

  public void changeSetpoint(String direction) {
    switch(direction) {
      case "up": {
        if (targetSetpoint + 600 <= Constants.Hood_Constants.hoodMaxDistance) targetSetpoint += 600;
        else targetSetpoint = Constants.Hood_Constants.hoodMaxDistance;
        hoodPIDController.setSetpoint(targetSetpoint);
        pidEnabled = true;
        break;
      }
      case "down": {
        if (targetSetpoint - 600 >= 0) targetSetpoint -= 600;
        else targetSetpoint = 0;
        hoodPIDController.setSetpoint(targetSetpoint);
        pidEnabled = true;
        break;
      }
      default: break;
    }
  }

  public void checkHome() {
    if (isAtHome() && getPosition() != 0) {
      hoodEncoder.reset();
      homed = true;
    }
  }

  public boolean isAtHome() {
    return shooterHood.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public boolean isHomed() {
    return homed;
  }

  public void resetPIDValues() {
    kP.setDouble(Constants.Shooter_Constants.PID_Values.kP);
    kI.setDouble(Constants.Shooter_Constants.PID_Values.kI);
    kD.setDouble(Constants.Shooter_Constants.PID_Values.kD);
    // kIz.setDouble(Constants.Shooter_Constants.PID_Values.kIz);
    // kFF.setDouble(Constants.Shooter_Constants.PID_Values.kFF);
    // kMaxOutput.setDouble(Constants.Shooter_Constants.PID_Values.kMax);
    // kMinOutput.setDouble(Constants.Shooter_Constants.PID_Values.kMin);
  }
}