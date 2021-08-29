/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private CANSparkMax climberRight;
  private CANSparkMax climberLeft;

  public Climber() {

    climberRight = new CANSparkMax(Constants.ArmConstants.rightMotor_ID, MotorType.kBrushless);
    climberLeft = new CANSparkMax(Constants.ArmConstants.leftMotor_ID, MotorType.kBrushless);

  }

  public void setClimber(double speed) {

    climberRight.set(speed);
    climberLeft.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
