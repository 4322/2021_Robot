/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private DoubleSolenoid dSolenoid;

  public Arm() {
    if (Constants.pcmEnabled) {
      dSolenoid = new DoubleSolenoid(0, 1);
      dSolenoid.set(Value.kForward);
    }
  }

  public void toggle(){
    if (Constants.pcmEnabled) {
      dSolenoid.toggle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
