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

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private DoubleSolenoid climbPistons;

  public Climber() {

    climbPistons = new DoubleSolenoid(0, 1);

  }

  public void extend()
  {
    climbPistons.set(Value.kForward);
  }

  public void retract()
  {
    climbPistons.set(Value.kReverse);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
