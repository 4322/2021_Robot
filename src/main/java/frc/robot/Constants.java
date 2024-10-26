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

    public static final boolean debug = false;
    public static final boolean demo = true;
    public static final boolean pcmEnabled = false;
    public static final boolean driveEnabled = false;  // breakers are pulled, so don't report errors

    public static class Drivebase_Constants {

        public static final int rightMasterSpark_ID = 1;
        public static final int rightSlave1Spark_ID = 5;  // this SPARK MAX isn't physically on the robot
      
        public static final int leftMasterSpark_ID = 9;
        public static final int leftSlave1Spark_ID = 11;

        public static final int SparkMax_CurrentLimit = 60;
        public static final double openLoopRampRate = 1.0;      // seconds to go from 0 to full power
        public static final double maxTurn = 1.0;  // fast rotation while defending against Nemo

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
		public static final double disableBreakSec = 2.0;

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
        public static final int leftMotor_ID = 7; // remove after climber is reworked
        public static final int rightMotor_ID = 6; // this too
        public static final int collectorTalon_ID = 15;  // should be 15, use for bad hopper Talon
        public static final double collector_speed = 0.85;
    }

    public static class Shooter_Constants
    {
        public static final int flywheelOneSpark_ID = 4;
        public static final int flywheelTwoSpark_ID = 8;
        public static final int kickerSpark_ID = 3;  // re-purposed drive SPARK MAX since we're short one

        public static final int shooterVel1 = 2250;     // wall
        public static final int shooterVel2 = 3000;     // 10 foot line
        public static final int shooterVel3 = 3500;     // trench
        public static final int minEjectVel = 1000;
        public static final int demoVel1 = 2200;
        public static final int demoVel2 = 2100;
        public static final int demoVel3 = 2000;
        public static final int tolerance = 100;
        public static final double rumbleIntensity = 1.0;

        public static final double closedLoopRampRate = 1.0;  // seconds to go from stopped to full power
        public static final double kickerPower = 0.5;

        public static class PID_Values  {
            public static final double kP = .00025;
            public static final double kI = 0.000001;
            public static final double kD = 0.004;
            public static final double kIz = 300;
            public static final double kFF = 0.00015;
            public static final int kMax = 1;
            public static final int kMin = 0;   // let flywheel coast down, don't apply power to slow it
        }
    }

    public static class Hood_Constants
    {
        public static final int hoodTalon_ID = 12;
        public static final int hoodMaxPosition = 9000;
        public static int hoodMinPosition = 0;     //allows to be changed in demo mode
        public static final int hoodDecellerationDistance = 500;
        public static final int hoodTolerance = 20;
        public static final double homingTimeout = 5.0; 
        public static final double autoTimeout = 3.0; 

        public static double maxForwardPower = 1.0;    //allows to be changed in demo mode
        public static double maxReversePower = -1.0;   //allows to be changed in demo mode
        public static final double minForwardPower = 0.1;
        public static final double minReversePower = -0.1;
        public static final double homingPower = -0.4;
        public static final double manualPower = 0.2;

        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public static final boolean kSensorPhase = false;
        public static final boolean kMotorInvert = false;

        public static class PID_Values
        {
            public static final double kP = 3.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static class Positions
        {
            public static final double pos1 = 2078;     // wall
            public static final double pos2 = 6450;     // 10 foot line
            public static final double pos3 = 7200;     // trench
        }
    }

    public static class Hopper_Constants
    {
        public static final int hopperMasterTalon_ID = 19;  // should be 13
        public static final int hopperSlaveTalon_ID = 14;  // should be 14
        public static final double hopperIntakePower = 0.6;
        public static final double hopperEjectPower = -0.6;
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

        public static final double limelightAngle = 30;     // degrees
        public static final double targetHeight = 98;       // inches
        public static final double limelightHeight = 22.5;  // inches

        public static class PID_Values
        {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
}
