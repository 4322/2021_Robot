/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */

  private WPI_TalonSRX hopperMotorMaster;
  private WPI_TalonSRX hopperMotorslave;

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
  }

  public void eject()
  {
    hopperMotorMaster.set(-.6);
  }

  public void stop()
  {
    hopperMotorMaster.set(0);
  }
}
