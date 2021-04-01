/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/** 
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double deadband = 0.2;
    public static final double differentialDriveDeadband = 0.15;

    public static class Drivebase_Constants {

        public static final int rightMasterSpark_ID = 1;
        public static final int rightSlave1Spark_ID = 3;
      
        
        public static final int leftMasterSpark_ID = 9;
        public static final int leftSlave1Spark_ID = 11;
        

        public static final int SparkMax_CurrentLimit = 60;

        // CHARACTERIZED DRIVE VALUES
        public static final double kTrackwidthMeters = 0.6771275509857637;
        // 2020: 0.68

        public static final double kMaxSpeedMetersPerSecond = 4.572/6; // divide by 6 to slow down
        // 2020: 2.438
        // 2021: 4.572
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // 2020: 3

        // RAMESETE STANDARD VALUES FROM WPILIB DOCS
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double distPerPulse = .0011114506; // In Meters
        public static final double velocityConversion = .0007780154; // In Meters


        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static class PID_Values {

            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double ksVolts = 0.153;
            // 2020: 0.361
            public static final double kvVoltSecondsPerMeter = 2.7;
            // 2020: 3.73
            public static final double kaVoltSecondsSquaredPerMeter = 0.478;
            // 2020: 0.55

            public static final double kPDriveVel = 0.00742;
            // 2020: 16.9
            // 2021.1: 16.1
            // 2021.1: 0.00742

        }
    }

    public static class ArmConstants
    {
        public static final int leftMotor_ID = 7;
        public static final int rightMotor_ID = 6;
        public static final int collectorTalonID = 15;

        public static final double collectSetpoint = 0;
        public static final double startingConfigSetpoint = 0;
        public static final double climbSetpoint = 0;

        public static class PID_Values {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0;
            public static final double kMaxOutput = 0;
            public static final double kMinOutput = 0;
            public static final double maxRPM = 0;

            public static final double maxVelocity = 0;
            public static final double maxAcceleration = 0;
            public static final double minVelocity = 0;

            public static final double allowed_error = 0;
        }
    }

    public static class Shooter_Constants
    {

        public static final int flywheelOneSpark_ID = 4;
        public static final int flywheelTwoSpark_ID = 8;
        public static final int kickerSpark_ID = 5;

        public static final int maxRPM = 4900;

        public static class PID_Values  {

            public static final double kP = .01;
            // 2020: 0.4
            public static final double kI = 0;
            public static final double kD = 0;
            // 2020: 0
            public static final double kIz = 0;
            public static final double kFF = 0;
            // 2020: 0.01;
            public static final int kMax = 1;
            public static final int kMin = -1;

        }
    }

    public static class Hood_Constants
    {
        public static final int hoodTalon_ID = 12;
        public static final int hoodTolerance = 10;
        public static final int setpointOne = 50; 

        public static class PID_Values
        {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;

        }
    }
    public static class Hopper_Constants
    {
    public static final int HopperMotorslaveID = 14; //CHANGE
    public static final int HopperMotormasterID = 13; //change

    }
    public static class Limelight_Constants
    {
        // Network Tables for Vision (from Robojacks 2019 https://github.com/Robojacks/FRC-2019-Rampage/blob/master/src/main/java/frc/robot/Constants.java)
        public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            
        public static final NetworkTableEntry tx = table.getEntry("tx");
        public static final NetworkTableEntry ty = table.getEntry("ty");
        public static final NetworkTableEntry ta = table.getEntry("ta");
        public static final NetworkTableEntry tv = table.getEntry("tv");

        public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
        public static final NetworkTableEntry camMode = table.getEntry("camMode");
        public static final NetworkTableEntry pipeline = table.getEntry("pipeline");

        public static final double limelightAngle = 30; //NEED TO CALCULATE IN DEGREES
        public static final double targetHeight = 98; //NEED TO MEASURE IN INCHES
        public static final double limelightHeight = 22.5; //NEED TO MEASURE IN INCHES

        public static class PID_Values
        {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }


}
