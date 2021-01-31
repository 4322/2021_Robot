/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {
  /**
   * Creates a new Shooter_Hood.
   */

   private WPI_TalonSRX shooterHood;
   private Encoder hoodEncoder;
   private PIDController hoodPIDController;
   private Limelight limelight;

  public Shooter_Hood() {
        // The PIDController used by the subsystem
        hoodPIDController = new PIDController(Constants.Hood_Constants.PID_Values.kP, Constants.Hood_Constants.PID_Values.kI, Constants.Hood_Constants.PID_Values.kD);
        shooterHood = new WPI_TalonSRX(Constants.Hood_Constants.hoodTalon_ID);
        hoodEncoder = new Encoder(0, 1);
        limelight = new Limelight();
        
        
  }

  

  public double generateSetpoint()
  {
    double distance = limelight.getDistance();
    double x = distance;

    double hoodPosition = Math.pow(x, 2); //NEED TO MAKE FUNCTION USING REGRESSION AND TESTED POINTS
    return hoodPosition;
  }

  public double getPosition()
  {
    return hoodEncoder.getDistance(); 
  }

  public void reachSetpoint()
  {
    double error = getPosition() - generateSetpoint();
  }

  public void setHood(double power)
  {
    shooterHood.set(power);
  }
}
