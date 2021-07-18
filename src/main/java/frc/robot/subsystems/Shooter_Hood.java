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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {

  private WPI_TalonSRX shooterHood;
  private Encoder hoodEncoder;
  private PIDController hoodPIDController;
  // private Limelight limelight;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

  private NetworkTableEntry hoodPosition =
    tab.add("Hood Position", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", Constants.Hood_Constants.hoodMaxDistance))
    .withPosition(0,0)
    .withSize(2,2)
    .getEntry();
  private NetworkTableEntry isHomeIndicator =
    tab.add("Homed?", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,2)
    .withSize(1,2)
    .getEntry();

  public Shooter_Hood() {
    // The PIDController used by the subsystem
    hoodPIDController = new PIDController(
      Constants.Hood_Constants.PID_Values.kP,
      Constants.Hood_Constants.PID_Values.kI,
      Constants.Hood_Constants.PID_Values.kD
    );
    shooterHood = new WPI_TalonSRX(Constants.Hood_Constants.hoodTalon_ID);
    hoodEncoder = new Encoder(0, 1);
    // limelight = new Limelight();
  }

  @Override
  public void periodic() {
    checkHome();

    hoodPosition.setDouble(getPosition());
    isHomeIndicator.setBoolean(isHome());
  }

  public double getPosition()
  {
    return hoodEncoder.getDistance(); 
  }

  public void setHood(double power)
  {
    double encValue = this.getPosition();
    if (this.getPosition() >= -4800 || power < 0) {
      if (this.getPosition() <= -4200 && power > 0) {
        double _power = power * ((4800 - (-encValue))/600);
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

  public void checkHome() {
    if (isHome() && getPosition() != 0) hoodEncoder.reset();
  }

  public boolean isHome() {
    return shooterHood.isRevLimitSwitchClosed() == 1 ? true : false;
  }
}