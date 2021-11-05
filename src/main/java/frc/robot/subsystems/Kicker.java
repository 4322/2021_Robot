/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  private CANSparkMax kickerMotor;;

  public Kicker() {

    kickerMotor = new CANSparkMax(Constants.Shooter_Constants.kickerSpark_ID, MotorType.kBrushless);
    kickerMotor.restoreFactoryDefaults();
    kickerMotor.setInverted(true);
    kickerMotor.setIdleMode(IdleMode.kBrake);   // don't let balls partially fall into the shooter
    kickerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableKicker()
  {
    kickerMotor.set(Constants.Shooter_Constants.kickerPower);
  }

  public void disableKicker()
  {
    kickerMotor.stopMotor();
  }
}
