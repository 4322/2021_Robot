/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight extends SubsystemBase {
  
  NetworkTableEntry tx = Constants.Limelight_Constants.tx;
  NetworkTableEntry ty = Constants.Limelight_Constants.ty;
  NetworkTableEntry ta = Constants.Limelight_Constants.ta;
  NetworkTableEntry tv = Constants.Limelight_Constants.tv;

  // SHUFFLEBOARD
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
  NetworkTableEntry distanceToTarget =
    tab.add("Distance to Target", getDistance())
    .withPosition(1,0)
    .withSize(2,1)
    .getEntry();

  public Limelight() {
    // Nothing to do here :)
  }

  @Override
  public void periodic() {
    getDistance();
  }

  public double getX_Offset()
  {
    return tx.getDouble(0);
  }

  public double getY_Offset()
  {
    return ty.getDouble(0);
  }

  public double getTargetArea()
  {
    return ta.getDouble(0);
  }

  public boolean getTarget()
  {
    return tv.getBoolean(false);
  }

  //Formula Referenced From: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public double getDistance()
  {
    // double distance = 
    // (Constants.Limelight_Constants.targetHeight - Constants.Limelight_Constants.limelightHeight)
    //   / (Math.tan(Math.toRadians(Constants.Limelight_Constants.limelightAngle + getY_Offset())));
    // distanceToTarget.setDouble(distance);
    // return distance;
    return 0;
  }
}
