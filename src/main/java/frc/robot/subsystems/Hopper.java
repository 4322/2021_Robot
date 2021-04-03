/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */

  private WPI_TalonSRX hopperMotorMaster;
  private WPI_TalonSRX hopperMotorslave;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
  private NetworkTableEntry power = tab.add("Power", 0)
  .withWidget(BuiltInWidgets.kDial)
  .withProperties(Map.of("min", -1, "max", 1))
  .getEntry();

  public Hopper() {

  hopperMotorMaster = new WPI_TalonSRX(Constants.Hopper_Constants.HopperMotormasterID);
  hopperMotorslave = new WPI_TalonSRX(Constants.Hopper_Constants.HopperMotorslaveID);

  hopperMotorMaster.setInverted(true);
  hopperMotorslave.follow(hopperMotorMaster);
  hopperMotorslave.setInverted(InvertType.OpposeMaster);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake()
  {
    hopperMotorMaster.set(.6);
    power.setDouble(.6);
  }

  public void eject()
  {
    hopperMotorMaster.set(-.6);
    power.setDouble(-.6);
  }

  public void stop()
  {
    hopperMotorMaster.set(0);
    power.setDouble(0);
  }
}
