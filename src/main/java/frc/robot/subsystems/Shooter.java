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
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends PIDSubsystem {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;


  private CANPIDController pidController;

  private CANEncoder flywheelOne_Encoder;
  private CANEncoder flywheelTwo_Encoder;
  
  private double rpm = Constants.Shooter_Constants.maxRPM;
  private double power = 0.7;
  private boolean shooterOn = false;


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
    // SmartDashboard.putNumber("Shoter RPM", rpm);
    // if (RobotContainer.coPilot.dPad.up.get() && rpm < Constants.Shooter_Constants.maxRPM) rpm = rpm + 200;
    // else if (RobotContainer.coPilot.dPad.down.get() && rpm > 100) rpm = rpm - 200;
    SmartDashboard.putNumber("Shooter Power", power);
    if (RobotContainer.coPilot.dPad.up.get() && power < 1) {
      power = power + 0.05;
    }
    else if (RobotContainer.coPilot.dPad.down.get() && power > 0) {
      power = power - 0.05;
    }
    if (shooterOn) flywheelOne.set(power);
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

  public double getRPM() {
    return rpm;
  }

  public void setPower(double newRPM) {
    rpm = newRPM;
  }
  
  public void stopShooter()
  {
    shooterOn = false;
    flywheelOne.set(0);
  }


  public void spinShooter()
  {
    shooterOn = true;
    flywheelOne.set(power);
  }


  public void reachSetpoint()
  {
    pidController.setReference(rpm, ControlType.kVelocity);
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
