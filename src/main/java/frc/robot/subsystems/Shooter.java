/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Shooter extends PIDSubsystem {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;


  private CANPIDController pidController;

  private CANEncoder flywheelOne_Encoder;
  private CANEncoder flywheelTwo_Encoder;
  




  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Shooter_Constants.PID_Values.kP, Constants.Shooter_Constants.PID_Values.kI, Constants.Shooter_Constants.PID_Values.kD));
    
    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.Shooter_Constants.flywheelTwoSpark_ID, MotorType.kBrushless);
    
    

    flywheelOne_Encoder = new CANEncoder(flywheelOne);
    flywheelTwo_Encoder = new CANEncoder(flywheelTwo);
    
    pidController = new CANPIDController(flywheelOne);

    flywheelOne.setIdleMode(IdleMode.kCoast);
    flywheelTwo.setIdleMode(IdleMode.kCoast);

    flywheelOne.burnFlash();
    flywheelTwo.burnFlash();

    pidController.setP(Constants.Shooter_Constants.PID_Values.kP);
    pidController.setI(Constants.Shooter_Constants.PID_Values.kI);
    pidController.setD(Constants.Shooter_Constants.PID_Values.kD);
    pidController.setIZone(Constants.Shooter_Constants.PID_Values.kIz);
    pidController.setFF(Constants.Shooter_Constants.PID_Values.kFF);
    pidController.setOutputRange(Constants.Shooter_Constants.PID_Values.kMin, Constants.Shooter_Constants.PID_Values.kMax);


    flywheelOne.setInverted(true);
    
    flywheelTwo.follow(flywheelOne, true);

    
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();

    displayEncoderValues();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void stopShooter()
  {
    flywheelOne.set(0);
  }


  public void spinShooter()
  {
    flywheelOne.set(.7);
  }


  public void reachSetpoint()
  {
    pidController.setReference(Constants.Shooter_Constants.maxRPM, ControlType.kVelocity);
  }


  public double getShooterEncoder_Position()
  {
    return ((flywheelOne_Encoder.getPosition() + flywheelTwo_Encoder.getPosition()) / 2);
  }

  public double getShooterEncoder_Velocity()
  {
    return ((flywheelOne_Encoder.getVelocity() + flywheelTwo_Encoder.getVelocity() / 2));
  }

  public void displayEncoderValues()
  {
    SmartDashboard.putNumber("Shooter Velocity", getShooterEncoder_Velocity());
    SmartDashboard.putNumber("Shooter Position", getShooterEncoder_Position());
  }
}
