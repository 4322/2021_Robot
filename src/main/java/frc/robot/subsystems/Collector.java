/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  /**
   * Creates a new Collector.
   */

   private WPI_TalonSRX collectorMotor;

  public Collector() {

    collectorMotor = new WPI_TalonSRX(Constants.ArmConstants.collectorTalonID);
    collectorMotor.setInverted(true);
  }

  public void collect()
  {
    collectorMotor.set(.7);
  }

  public void eject()
  {
    collectorMotor.set(-.7);
  }

  public void stop()
  {
    collectorMotor.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
