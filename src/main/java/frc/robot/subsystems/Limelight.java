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

  NetworkTableEntry _tv =
    tab.add("Target Visible", getTarget())
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,0)
    .getEntry();
  NetworkTableEntry distanceToTarget =
    tab.add("Distance to Target", getDistance())
    .withPosition(1,0)
    .withSize(2,1)
    .getEntry();
  NetworkTableEntry _ta = tab.add("Target Area", getTargetArea()).withPosition(0,1).getEntry();
  NetworkTableEntry _tx = tab.add("X Offset", getX_Offset()).withPosition(1,1).getEntry();
  NetworkTableEntry _ty = tab.add("Y Offset", getY_Offset()).withPosition(2,1).getEntry();

  public Limelight() {
    // Nothing to do here :)
  }

  @Override
  public void periodic() {
    if(getTarget()) RobotContainer.pilot.setRumble(0.2);

    updateShuffleboard();
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
    double distance = 
    (Constants.Limelight_Constants.targetHeight - Constants.Limelight_Constants.limelightHeight)
      / (Math.tan(Constants.Limelight_Constants.limelightAngle + getY_Offset()));
    return distance;
  }

  private void updateShuffleboard() {
    if (getX_Offset() != _tx.getDouble(0)) _tx.setDouble(getX_Offset());
    if (getY_Offset() != _ty.getDouble(0)) _ty.setDouble(getY_Offset());
    if (getTargetArea() != _ta.getDouble(0)) _ta.setDouble(getTargetArea());
    if (getTarget () != _tv.getBoolean(false)) _tv.setBoolean(getTarget());
    if (getDistance() != distanceToTarget.getDouble(0)) distanceToTarget.setDouble(getDistance());
  }

}
