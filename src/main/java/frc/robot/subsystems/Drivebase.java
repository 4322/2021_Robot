/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
  private PIDController limelightPidController;

  // Limelight Object to Access Vision Values
  private Limelight limelight;

  // Drive Object to Enable Various Modes
  private DifferentialDrive drive;

  /**
   * Creates a new Drivebase.
   */
  public Drivebase() {

    // Creates NavX
    navX = new AHRS(SPI.Port.kMXP);

    // Creates Limlight and Limelight PID Controller
    limelightPidController = new PIDController(Constants.Limelight_Constants.PID_Values.kP, Constants.Limelight_Constants.PID_Values.kI, Constants.Limelight_Constants.PID_Values.kD);
    limelight = new Limelight();
  
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

    //Creates an Odometry Object That Is Used To Allow the Robot to Follow Trajectories
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  /**********************************************************************************************************************************
  ***********************************************************************************************************************************
  *************************************** DRIVEBASE METHODS ARE NOW STORED BELOW ****************************************************
  ***********************************************************************************************************************************
  ***********************************************************************************************************************************/
  
  @Override
  public void periodic() {
    
    // Odometry Calculates the Robot's Position On The Field so It Will Constantly Update by Reading Encoder and Gyro Values
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoders_Position(), getRightEncoders_Position());

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
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  // Gets the Wheel Speeds of Each Side of the Drivebase
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoders_Velocity(), getRightEncoders_Velocity());
  }

  // Gets Heading Angle from the Gyro (NavX)
  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360);
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
   *********** LIMELIGHT AUTO-AIM METHODS *************
   ****************************************************/
  
  // Automatically Turns the Robot to Face the Vision Target Using a PID Controller
  public void limelightAim_PID()
  {
    double offset = limelight.getX_Offset();
    double error = offset * .01;
    limelightPidController.setTolerance(.02);
    double motorOutput = limelightPidController.calculate(error, 0);
    SmartDashboard.putNumber("Turn Output", motorOutput);
    tankDrive(motorOutput, -motorOutput);
  }

  // Automatically Turns the Robot to Face the Vision Target Using a Predefined P Constant and Tolerances
  public void limelightAim_Simple()
  {
    double kP = -0.1;
    double minCommand = .05;
    double heading_error = -limelight.getX_Offset();
    double steering_adjust = 0.0f;

        if (limelight.getX_Offset() > 1.0)
        {
                steering_adjust = kP * heading_error - minCommand;
        }
        else if (limelight.getX_Offset() < 1.0)
        {
                steering_adjust = kP * heading_error + minCommand;
        }
    drive.tankDrive(steering_adjust, -steering_adjust);
  }

  
  /****************************************************
   ************ DIFFERENTIAL DRIVE MODES **************
   ****************************************************/
  public void curveDrive(double power, double turn, boolean quickTurn)
  {
    drive.curvatureDrive(power, turn, quickTurn);
  }

  public void arcadeDrive(double power, double turn, boolean squaredInputs)
  {
    drive.arcadeDrive(power, turn, squaredInputs);
  }

  public void tankDrive(double left, double right)
  {
    drive.tankDrive(left, right);
  }


  /****************************************************
   ********* GETTING GROUPED ENCODER VALUES ***********
   ************* (POSITION / VELOCITY) ****************
   ****************************************************/
  public double getRightEncoders_Position()
  {
    return ((leftMaster_encoder.getPosition() + leftSlave1_encoder.getPosition())/2);
  }

  public double getLeftEncoders_Position()
  {
    return ((rightMaster_encoder.getPosition() + rightSlave1_encoder.getPosition())/2);
  }

  public double getRightEncoders_Velocity()
  {
    return ((rightMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity())/2);
  }

  public double getLeftEncoders_Velocity()
  {
    return ((leftMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity())/2);
  }

  /****************************************************
   ********* GETTING SINGLE ENCODER VALUES ************
   ****************** (POSITION) **********************
   ****************************************************/
   public double getRightMasterEncoderPosition()
   {
      return rightMaster_encoder.getPosition();
   }

   public double getRightSlave1EncoderPosition()
   {
      return rightSlave1_encoder.getPosition();
   }

   

   public double getLeftMasterEncoderPosition()
   {
      return leftMaster_encoder.getPosition();
   }

   public double getLeftSlave1EncoderPosition()
   {
      return leftSlave1_encoder.getPosition();
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
      return rightMaster_encoder.getVelocity();
   }

   public double getLeftSlave1EncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }

   public double getLeftSlave2EncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
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
    double[] leftEncoders = new double[]{getLeftMasterEncoderVelocity(), getLeftSlave1EncoderVelocity(), getLeftSlave2EncoderVelocity()};
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
    

    rightMaster_encoder.setVelocityConversionFactor(conversionFactor);
    rightSlave1_encoder.setVelocityConversionFactor(conversionFactor);
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
