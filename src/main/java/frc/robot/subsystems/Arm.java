/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  public Arm() {
      leftArm = new CANSparkMax(Constants.ArmConstants.leftMotor_ID, MotorType.kBrushless);
      rightArm = new CANSparkMax(Constants.ArmConstants.rightMotor_ID, MotorType.kBrushless);

      leftArm_encoder = new CANEncoder(leftArm);
      rightArm_encoder = new CANEncoder(rightArm);

      ArmPidController = rightArm.getPIDController(); 

      rightArm.follow(leftArm, true);

      ArmPidController.setP(Constants.ArmConstants.PID_Values.kP);
      ArmPidController.setI(Constants.ArmConstants.PID_Values.kI);
      ArmPidController.setD(Constants.ArmConstants.PID_Values.kD);
      ArmPidController.setIZone(Constants.ArmConstants.PID_Values.kIz);
      ArmPidController.setFF(Constants.ArmConstants.PID_Values.kFF);
      ArmPidController.setOutputRange(Constants.ArmConstants.PID_Values.kMinOutput,Constants.ArmConstants.PID_Values.kMaxOutput);

      int smartmotionslot = 0;
      ArmPidController.setSmartMotionMaxVelocity(Constants.ArmConstants.PID_Values.maxVelocity, smartmotionslot);
      ArmPidController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.PID_Values.minVelocity, smartmotionslot);
      ArmPidController.setSmartMotionMaxAccel(Constants.ArmConstants.PID_Values.maxAcceleration, smartmotionslot);
    
      ArmPidController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.PID_Values.allowed_error, smartmotionslot);
  
  }

  public double getArmEncoderPosition() {
    return (leftArm_encoder.getPosition() + rightArm_encoder.getPosition())/2;
  }

  public double getArmEncoderVelocity() {
    return (leftArm_encoder.getVelocity() + leftArm_encoder.getVelocity()) /2;
  }

  public void displayArmEncoderValues() {
    SmartDashboard.putNumber("Arm Encoder Position", getArmEncoderPosition());
    SmartDashboard.putNumber("Arm Encoder Velocity", getArmEncoderVelocity());
  }

  public void reachCollectPosition()
  {
    ArmPidController.setReference(Constants.ArmConstants.collectSetpoint, ControlType.kSmartMotion);
  }

  public void set(double power)
  {
    leftArm.set(power);
  }

  public void reachStartingConfiguration()
  {
    ArmPidController.setReference(Constants.ArmConstants.startingConfigSetpoint, ControlType.kSmartMotion);
  }

  public void reachClimbPosition()
  {
    ArmPidController.setReference(Constants.ArmConstants.climbSetpoint, ControlType.kSmartMotion);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayArmEncoderValues();
  }
}
