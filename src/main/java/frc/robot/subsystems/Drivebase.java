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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private NetworkTableEntry Shuffle_gyro =
    tab.add("Gyro angle", 0)
    .getEntry();
  private NetworkTableEntry maxSpeed =
    tab.add("Max Speed", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();
  private NetworkTableEntry Shuffle_power =
    tab.add("Power", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
  private NetworkTableEntry Shuffle_turn =
    tab.add("Turn", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

  private NetworkTableEntry Shuffle_lencoder =
    tab.add("Left Master Encoder", 0).getEntry();
  private NetworkTableEntry Shuffle_lencoder_slave =
    tab.add("Left Slave Encoder", 0).getEntry();
  private NetworkTableEntry Shuffle_left_position =
    tab.add("Left Encoder Posoition", 0).getEntry();
  private NetworkTableEntry Shuffle_rencoder =
    tab.add("Right Master Encoder", 0).getEntry();
  private NetworkTableEntry Shuffle_rencoder_slave =
    tab.add("Right Slave Encoder", 0).getEntry();
  private NetworkTableEntry Shuffle_right_position =
    tab.add("Right Encoder Posoition", 0).getEntry();

  private NetworkTableEntry Shuffle_lvelocity =
    tab.add("Left Master Velocity", 0).getEntry();
  private NetworkTableEntry Shuffle_lvelocity_slave =
    tab.add("Left Slave Velocity", 0).getEntry();
  private NetworkTableEntry Shuffle_left_velocity =
    tab.add("Left Velocity", 0).getEntry();
  
  private NetworkTableEntry Shuffle_rvelocity =
    tab.add("Right Master Velocity", 0).getEntry();
  private NetworkTableEntry Shuffle_rvelocity_slave =
    tab.add("Right Slave Velocity", 0).getEntry();
  private NetworkTableEntry Shuffle_right_velocity =
    tab.add("Right Velocity", 0).getEntry();

  /**
   * Creates a new Drivebase.
   */
  public Drivebase() {

    // Creates NavX
    navX = new AHRS(SPI.Port.kMXP);

    // Creates Limlight and Limelight PID Controller
    // limelightPidController = new PIDController(Constants.Limelight_Constants.PID_Values.kP, Constants.Limelight_Constants.PID_Values.kI, Constants.Limelight_Constants.PID_Values.kD);
    // limelight = new Limelight();
  
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
    
    // Groups Motor Controllers Instead of Making Slaves Follow Masters
    rightMotors = new SpeedControllerGroup(rightMaster, rightSlave1);
    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave1);

    //Creates New Drive Object to Allow for Tank, Arcade, and Curvature Drive
    drive = new DifferentialDrive(leftMotors, rightMotors);
    
    tab.add("Drivetrain", drive)
    .withWidget(BuiltInWidgets.kDifferentialDrive);

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

    Shuffle_gyro.setDouble(navX.getAngle());
    getLeftEncoders_Velocity();
    getRightEncoders_Velocity();
  }

  public void changePower(String direction) {
    double max = maxSpeed.getDouble(1.0);

    switch(direction) {
      case "up": {
        if (max < 1.0) maxSpeed.setDouble(max + 0.1);
        break;
      }
      case "down": {
        if (max > 0) maxSpeed.setDouble(max - 0.1);
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
  public void curveDrive(double power, double turn, boolean quickTurn)
  {
    double max = maxSpeed.getDouble(1.0);
    drive.curvatureDrive(power * max, turn * max, quickTurn);
    Shuffle_power.setDouble(power * max);
    Shuffle_turn.setDouble(turn * max);
  }

  public void arcadeDrive(double power, double turn, boolean squaredInputs)
  {
    double max = maxSpeed.getDouble(1.0);
    drive.arcadeDrive(power * max, turn * max, squaredInputs);
    Shuffle_power.setDouble(power * max);
    Shuffle_turn.setDouble(turn * max);
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
    Shuffle_lencoder.setDouble(leftMaster_encoder.getPosition());
    Shuffle_lencoder_slave.setDouble(leftSlave1_encoder.getPosition());

    var position = ((leftMaster_encoder.getPosition() + leftSlave1_encoder.getPosition())/2);
    Shuffle_left_position.setDouble(position);
    return position;
  }

  public double getRightEncoders_Position()
  {
    Shuffle_rencoder.setDouble(rightMaster_encoder.getPosition() * -1);
    Shuffle_rencoder_slave.setDouble(rightSlave1_encoder.getPosition() * -1);

    var position = ((rightMaster_encoder.getPosition() + rightSlave1_encoder.getPosition())/2) * -1;
    Shuffle_right_position.setDouble(position);
    return position;
  }

  public double getRightEncoders_Velocity()
  {
    Shuffle_rvelocity.setDouble(rightMaster_encoder.getVelocity() * -1);
    Shuffle_rvelocity_slave.setDouble(rightSlave1_encoder.getVelocity() *- 1);

    var velocity = ((rightMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity())/2) * -1;
    Shuffle_right_velocity.setDouble(velocity);
    return velocity;
  }

  public double getLeftEncoders_Velocity()
  {
    Shuffle_lvelocity.setDouble(leftMaster_encoder.getVelocity());
    Shuffle_lvelocity_slave.setDouble(leftSlave1_encoder.getVelocity());

    var velocity = ((leftMaster_encoder.getVelocity() + leftSlave1_encoder.getVelocity())/2);
    Shuffle_left_velocity.setDouble(velocity);
    return velocity;
  }

  /****************************************************
   ********* GETTING SINGLE ENCODER VALUES ************
   ****************** (POSITION) **********************
   ****************************************************/
   public double getRightMasterEncoderPosition()
   {
    var position = rightMaster_encoder.getPosition();
    // Shuffle_rencoder.setDouble(position);
    return position;
   }

   public double getRightSlave1EncoderPosition()
   {
    var position = rightSlave1_encoder.getPosition();
    // Shuffle_rencoder_slave.setDouble(position);
    return position;
   }

   public double getLeftMasterEncoderPosition()
   {
    var position = leftMaster_encoder.getPosition();
    // Shuffle_lencoder.setDouble(position);
    return position;
   }

   public double getLeftSlave1EncoderPosition()
   {
    var position = leftSlave1_encoder.getPosition();
    // Shuffle_lencoder_slave.setDouble(position);
    return position;
   }

  /****************************************************
   ********* GETTING SINGLE ENCODER VALUES ************
   ****************** (VELOCITY) **********************
   ****************************************************/
   public double getRightMasterEncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }
   
   public double getRightSlave1EncoderVelocity()
   {
      return rightSlave1_encoder.getVelocity();
   }
  
   public double getLeftMasterEncoderVelocity()
   {
      return leftMaster_encoder.getVelocity();
   }

   public double getLeftSlave1EncoderVelocity()
   {
      return leftSlave1_encoder.getVelocity();
   }

  /****************************************************
   ** DISPLAYING GROUPED ENCODER VALUES ON DASHBOARD **
   ************* (POSITION / VELOCITY) ****************
   ****************************************************/
  public void displayLeftEncodersPosition()
  {
    SmartDashboard.putNumber("Left Side Encoder", getLeftEncoders_Position());
  }

  public void displayRightEncodersPosition()
  {
    SmartDashboard.putNumber("Right Side Encoder Position", getRightEncoders_Position());
  }

  public void displayLeftEncodersVelocity()
  {
    SmartDashboard.putNumber("Left Side Encoder Velocity", getLeftEncoders_Velocity());
  }

  public void displayRightEncodersVelocity()
  {
    SmartDashboard.putNumber("Right Side Encoder Velocity", getRightEncoders_Velocity());
  }

  /*****************************************************
   * DISPLAYING INDIVIDUAL ENCODER VALUES ON DASHBOARD *
   ************* (POSITION / VELOCITY) *****************
   *****************************************************/
  public void displayAllLeftSideEncoders_Position()
  {
    double[] leftEncoders = new double[]{getLeftMasterEncoderPosition(), getLeftSlave1EncoderPosition()};
    SmartDashboard.putNumberArray("Left Encoder Position (Master, Slave1, Slave2)", leftEncoders);
  }

  public void displayAllRightSideEncoders_Position()
  {
    double[] rightEncoders = new double[]{getRightMasterEncoderPosition(), getRightSlave1EncoderPosition()};
    SmartDashboard.putNumberArray("Right Encoder Position (Master, Slave1, Slave2)", rightEncoders);
  }

  public void displayAllRightSideEncoders_Velocity()
  {
    double[] rightEncoders = new double[]{getRightMasterEncoderVelocity(), getRightSlave1EncoderVelocity()};
    SmartDashboard.putNumberArray("Right Encoder Velocity (Master, Slave1, Slave2)", rightEncoders);
  }

  public void displayAllLeftSideEncoders_Velocity()
  {
    double[] leftEncoders = new double[]{getLeftMasterEncoderVelocity(), getLeftSlave1EncoderVelocity()};
    SmartDashboard.putNumberArray("Left Encoder Velocity (Master, Slave1, Slave2)", leftEncoders);
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
