/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

  // Drivebase Motors
  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave1;
  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave1;
  
  // Drivebase Motor Encoders
  private CANEncoder rightMaster_encoder;
  private CANEncoder rightSlave1_encoder;
  private CANEncoder leftMaster_encoder;
  private CANEncoder leftSlave1_encoder;
  
  // NavX Gyro
  private AHRS navX;

  // Odometry Object for Trajectory Following
  private DifferentialDriveOdometry odometry;

  //Speed Controller Groups for Each Side of Drivebase
  private SpeedControllerGroup rightMotors;
  private SpeedControllerGroup leftMotors;

  // PID Controller that Controls Turning the Robot Using Values From the Limelight
  // private PIDController limelightPidController;

  // Limelight Object to Access Vision Values
  // private Limelight limelight;

  // Drive Object to Enable Various Modes
  private DifferentialDrive drive;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");
  private NetworkTableEntry maxSpeed =
    tab.add("Max Speed", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .withPosition(0,3)
    .withSize(3,1)
    .getEntry();
  private NetworkTableEntry Shuffle_power =
    tab.add("Power", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .withPosition(3,2)
    .withSize(1,1)
    .getEntry();
  private NetworkTableEntry Shuffle_turn =
    tab.add("Turn", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .withPosition(4,2)
    .withSize(1,1)
    .getEntry();

  private ShuffleboardLayout positionList =
    tab.getLayout("Encoder Position", BuiltInLayouts.kGrid)
    .withProperties(Map.of("Label position", "RIGHT"))
    .withSize(3,2)
    .withPosition(5,0);
  // Left encoder position
  private NetworkTableEntry Shuffle_lencoder =
    positionList.add("Left Master Encoder", 0)
      .withPosition(0,0)
      .getEntry();
  private NetworkTableEntry Shuffle_lencoder_slave =
    positionList.add("Left Slave Encoder", 0)
      .withPosition(0,1)
      .getEntry();
  private NetworkTableEntry Shuffle_left_position =
    positionList.add("Left Encoder Posoition", 0)
      .withPosition(0,2)
      .getEntry();
  // Right encoder position
  private NetworkTableEntry Shuffle_rencoder =
   positionList.add("Right Master Encoder", 0)
    .withPosition(1,0)
    .getEntry();
  private NetworkTableEntry Shuffle_rencoder_slave =
   positionList.add("Right Slave Encoder", 0)
    .withPosition(1,1)
    .getEntry();
  private NetworkTableEntry Shuffle_right_position =
    positionList.add("Right Encoder Posoition", 0)
      .withPosition(1,2)
      .getEntry();

  private ShuffleboardLayout velocityList =
    tab.getLayout("Drivetrain Velocity", BuiltInLayouts.kGrid)
    .withProperties(Map.of("Label position", "RIGHT"))
    .withSize(3,2)
    .withPosition(5,2);
  // Left velocity
  private NetworkTableEntry Shuffle_lvelocity =
    velocityList.add("Left Master Velocity", 0)
      .withPosition(0,0)
      .getEntry();
  private NetworkTableEntry Shuffle_lvelocity_slave =
    velocityList.add("Left Slave Velocity", 0)
      .withPosition(0,1)
      .getEntry();
  private NetworkTableEntry Shuffle_left_velocity =
    velocityList.add("Left Velocity", 0)
      .withPosition(0,2)
      .getEntry();
  // Right velocity
  private NetworkTableEntry Shuffle_rvelocity =
    velocityList.add("Right Master Velocity", 0)
      .withPosition(1,0)
      .getEntry();
  private NetworkTableEntry Shuffle_rvelocity_slave =
    velocityList.add("Right Slave Velocity", 0)
      .withPosition(1,1)
      .getEntry();
  private NetworkTableEntry Shuffle_right_velocity =
    velocityList.add("Right Velocity", 0)
      .withPosition(1,2)
      .getEntry();

  /**
   * Creates a new Drivebase.
   */
  public Drivebase() {

    // Creates NavX
    navX = new AHRS(SPI.Port.kMXP);

    tab.add("Gyro", navX)
      .withWidget(BuiltInWidgets.kGyro)
      .withPosition(3,0)
      .withSize(2,2);

    // Creates Limlight and Limelight PID Controller
    // limelightPidController = new PIDController(Constants.Limelight_Constants.PID_Values.kP, Constants.Limelight_Constants.PID_Values.kI, Constants.Limelight_Constants.PID_Values.kD);
  
    // Creates Drivebase Motors
    rightMaster = new CANSparkMax(Constants.Drivebase_Constants.rightMasterSpark_ID, MotorType.kBrushless);
    rightSlave1 = new CANSparkMax(Constants.Drivebase_Constants.rightSlave1Spark_ID, MotorType.kBrushless);
    leftMaster = new CANSparkMax(Constants.Drivebase_Constants.leftMasterSpark_ID, MotorType.kBrushless);
    leftSlave1 = new CANSparkMax(Constants.Drivebase_Constants.leftSlave1Spark_ID, MotorType.kBrushless);
    
    // Creates Drivebase Encoders
    rightMaster_encoder = rightMaster.getEncoder();
    rightSlave1_encoder = rightSlave1.getEncoder();
    leftMaster_encoder = leftMaster.getEncoder();
    leftSlave1_encoder = leftSlave1.getEncoder();

    // Configures Motors For The Drivebase Gear Ratio, Current Limit, and Then Saves Settings to Motors
    setPositionConversionFactor(Constants.Drivebase_Constants.distPerPulse);
    setVelocityConversionFactor(Constants.Drivebase_Constants.velocityConversion);
    setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);
    saveMotorSettings();
    
    // Don't shred the belts
    rightMaster.setOpenLoopRampRate(Constants.Drivebase_Constants.openLoopRampRate);
    leftMaster.setOpenLoopRampRate(Constants.Drivebase_Constants.openLoopRampRate);

    // Groups Motor Controllers Instead of Making Slaves Follow Masters
    rightMotors = new SpeedControllerGroup(rightMaster, rightSlave1);
    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave1);

    //Creates New Drive Object to Allow for Tank, Arcade, and Curvature Drive
    drive = new DifferentialDrive(leftMotors, rightMotors);
    
    // Add drivetrain to Shuffleboard
    tab.add("Drivetrain", drive)
    .withWidget(BuiltInWidgets.kDifferentialDrive)
    .withPosition(0,0)
    .withSize(3,3);

    //Creates an Odometry Object That Is Used To Allow the Robot to Follow Trajectories
    odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    // odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  /**********************************************************************************************************************************
  ***********************************************************************************************************************************
  *************************************** DRIVEBASE METHODS ARE NOW STORED BELOW ****************************************************
  ***********************************************************************************************************************************
  ***********************************************************************************************************************************/
  
  @Override
  public void periodic() {
    
    // Odometry Calculates the Robot's Position On The Field so It Will Constantly Update by Reading Encoder and Gyro Values
    odometry.update(navX.getRotation2d(), getLeftEncoders_Position(), getRightEncoders_Position());

    getLeftEncoders_Velocity();
    getRightEncoders_Velocity();
  }

  public void changePower(String direction) {
    double max = maxSpeed.getDouble(1.0);

    switch(direction) {
      case "up": {
        if ((max + 0.1) <= 1.0) maxSpeed.setDouble(max + 0.1);
        else maxSpeed.setDouble(1.0);
        break;
      }
      case "down": {
        if ((max - 0.1) >= 0) maxSpeed.setDouble(max - 0.1);
        else maxSpeed.setDouble(0);
        break;
      }
      default: break;
    }
  }

  /****************************************************
   ********* METHODS FOR TRAJECTORY FOLLOWING  ********
   ****************************************************/
 
  // Gets Robot's Current Position on the Field
  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  // Resets the Robot's Current Position Read By The Odometry Object
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, navX.getRotation2d());
  }

  // Gets the Wheel Speeds of Each Side of the Drivebase
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoders_Velocity(), getRightEncoders_Velocity());
  }

  // Gets Heading Angle from the Gyro (NavX)
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
    // return Math.IEEEremainder(navX.getAngle(), 360);
  }

  // Gets Speed at Which the Robot is Turning
  public double getRurnRate()
  {
    return navX.getRate();
  }

  // Zero's the Reading from the NavX
  public void zeroHeading()
  {
    navX.zeroYaw();
  }

  // Allows For Independent Control of Each Side of the Robot Without Getting Controlled By the Tank Drive Object
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    drive.feed();
  }

  /****************************************************
   ************ DIFFERENTIAL DRIVE MODES **************
   ****************************************************/
  public void curveDrive(double power, double turn, boolean quickTurn, boolean crawlMode)
  {
    double max = maxSpeed.getDouble(1.0);
    if (crawlMode) {
      power = power/2;
    }
    drive.curvatureDrive(power * max, turn * max, quickTurn);
    if (Constants.debug) {
      Shuffle_power.setDouble(power * max);
      Shuffle_turn.setDouble(turn * max);
    }
  }

  public void arcadeDrive(double power, double turn, boolean squaredInputs)
  {
    double max = maxSpeed.getDouble(1.0);
    drive.arcadeDrive(power * max, turn * max, squaredInputs);
    if (Constants.debug) {
      Shuffle_power.setDouble(power * max);
      Shuffle_turn.setDouble(turn * max);
    }
  }

  public void tankDrive(double left, double right)
  {
    double max = maxSpeed.getDouble(1.0);
    drive.tankDrive(left * max, right * max);
  }


  /****************************************************
   ********* GETTING GROUPED ENCODER VALUES ***********
   ************* (POSITION / VELOCITY) ****************
   ****************************************************/
  public double getLeftEncoders_Position()
  {
    double masterPos = leftMaster_encoder.getPosition();
    double slavePos = leftSlave1_encoder.getPosition();
    double position = (leftMaster_encoder.getPosition() + leftSlave1_encoder.getPosition()) / 2;
    
    if (Constants.debug) {
      Shuffle_lencoder.setDouble(masterPos);
      Shuffle_lencoder_slave.setDouble(slavePos);
      Shuffle_left_position.setDouble(position);
    }
    return position;
  }

  public double getRightEncoders_Position()
  {
    double masterPos = -rightMaster_encoder.getPosition();
    double slavePos = -rightSlave1_encoder.getPosition();
    double position = (rightMaster_encoder.getPosition() + rightSlave1_encoder.getPosition()) / 2;
    
    if (Constants.debug) {
      Shuffle_rencoder.setDouble(masterPos);
      Shuffle_rencoder_slave.setDouble(slavePos);
      Shuffle_right_position.setDouble(position);
    }
    return position;
  }

  public double getLeftEncoders_Velocity()
  {
    double masterVelocity = leftMaster_encoder.getVelocity();
    double slaveVelocity = leftSlave1_encoder.getVelocity();
    double velocity = ((leftMaster_encoder.getVelocity() + leftSlave1_encoder.getVelocity())/2);
    
    if (Constants.debug) {
      Shuffle_lvelocity.setDouble(masterVelocity);
      Shuffle_lvelocity_slave.setDouble(slaveVelocity);
      Shuffle_left_velocity.setDouble(velocity);
    }
    return velocity;
  }

  public double getRightEncoders_Velocity()
  {
    double masterVelocity = -rightMaster_encoder.getVelocity();
    double slaveVelocity = -rightSlave1_encoder.getVelocity();
    double velocity = ((rightMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity())/2);

    if (Constants.debug) {
      Shuffle_rvelocity.setDouble(masterVelocity);
      Shuffle_rvelocity_slave.setDouble(slaveVelocity);
      Shuffle_right_velocity.setDouble(velocity);
    }
    return velocity;
  }

  /****************************************************
   * SETTING POSITION AND VELOCITY CONVERSION FACTORS *
   ***** ALSO ANY MOTOR SETTING METHODS ARE HERE ******
   ****************************************************/
  public void setPositionConversionFactor(double conversionFactor)
  {
    rightMaster_encoder.setPositionConversionFactor(conversionFactor);
    rightSlave1_encoder.setPositionConversionFactor(conversionFactor);

    leftMaster_encoder.setPositionConversionFactor(conversionFactor);
    leftSlave1_encoder.setPositionConversionFactor(conversionFactor);
  }

  public void setVelocityConversionFactor(double conversionFactor)
  {
    rightMaster_encoder.setVelocityConversionFactor(conversionFactor);
    rightSlave1_encoder.setVelocityConversionFactor(conversionFactor);

    leftMaster_encoder.setVelocityConversionFactor(conversionFactor);
    leftSlave1_encoder.setVelocityConversionFactor(conversionFactor);
  }

  public void setSmartCurrentLimit(int currentLimit)
  {
    rightMaster.setSmartCurrentLimit(currentLimit);
    rightSlave1.setSmartCurrentLimit(currentLimit);

    leftMaster.setSmartCurrentLimit(currentLimit);
    leftSlave1.setSmartCurrentLimit(currentLimit);
  }

  public void saveMotorSettings()
  {
    rightMaster.burnFlash();
    rightSlave1.burnFlash();
    leftMaster.burnFlash();
    leftSlave1.burnFlash();
  }

  public void resetEncoders()
  {
    rightMaster_encoder.setPosition(0);
    rightSlave1_encoder.setPosition(0);
    
    leftMaster_encoder.setPosition(0);
    leftSlave1_encoder.setPosition(0);
  }
}