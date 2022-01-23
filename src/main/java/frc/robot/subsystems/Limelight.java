/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  NetworkTableEntry camMode = table.getEntry("camMode");
  NetworkTableEntry pipeline = table.getEntry("pipeline");

  // SHUFFLEBOARD
  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

  NetworkTableEntry distanceToTarget =
    tab.add("Distance to Target", 0)
    .withPosition(1,0)
    .withSize(1,1)
    .getEntry();

  NetworkTableEntry targetVisible =
    tab.add("Target Visible", false)
    .withPosition(1,0)
    .withSize(1,1)
    .getEntry();

  public Limelight() {
    // Nothing to do here :)
  }

  @Override
  public void periodic() {
    getDistance();
    targetVisible.setBoolean(getTarget());
    updateLimelightValues();
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

  public void setLed(LedMode mode) {
    ledMode.setNumber(mode.value);
  }

  public void setCamMode(CamMode mode) {
      if (mode == CamMode.VisionProcessor) {
          camMode.setNumber(0);
      } else if (mode == CamMode.DriverCamera) {
          camMode.setNumber(1);
      }
  }

  public enum LedMode {
    Off(1),
    Blink(2),
    On(3);

    private int value;

    LedMode(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }
  }

  public enum CamMode {
    VisionProcessor,
    DriverCamera;
  }

  //Formula Referenced From: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public double getDistance()
  {
    double distance = 0;

    if (getTarget()) {
      double angleToTarget = Constants.Limelight_Constants.limelightAngle + getY_Offset();
      distance = (Constants.Limelight_Constants.targetHeight - Constants.Limelight_Constants.limelightHeight) / Math.tan(Math.toRadians(angleToTarget));
    }
    distanceToTarget.setDouble(distance);
    return distance;
  }

  public void updateLimelightValues() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
    pipeline = table.getEntry("pipeline");
  }
}
