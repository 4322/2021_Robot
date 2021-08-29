/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private CANSparkMax leftArm;
  private CANSparkMax rightArm;
  private CANEncoder leftArm_encoder;
  private CANEncoder rightArm_encoder;

  private CANPIDController ArmPidController;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  NetworkTableEntry armPower = tab.add("Arm Power", 0)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();
  
  public Arm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
