/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {

  private WPI_TalonSRX shooterHood;

  // Hood control
  private boolean homed = false;
  private boolean pidEnabled = false;
  private double targetPosition = 0;

  // NEW PID
  private double currentP = 0;
  private double currentI = 0;
  private double currentD = 0;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

  private NetworkTableEntry hoodPositionTalon =
    tab.add("Hood Position (Talon)", 0)
    .withPosition(2,2)
    .withSize(1,1)
    .getEntry();
  private NetworkTableEntry hoodPower =
    tab.add("Hood Power", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .withPosition(2,0)
    .withSize(2,2)
    .getEntry();
  private NetworkTableEntry isHomeIndicator =
    tab.add("Is @ home", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,2)
    .withSize(1,1)
    .getEntry();
  private NetworkTableEntry isHomedIndicator =
    tab.add("Homed", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(1,2)
    .withSize(1,1)
    .getEntry();

  private ShuffleboardLayout hoodPID =
    tab.getLayout("Hood PID", BuiltInLayouts.kGrid)
    .withProperties(Map.of("Label position", "RIGHT"))
    .withSize(4,2)
    .withPosition(4,0);
  private NetworkTableEntry kP =
    hoodPID.add("kP", Constants.Hood_Constants.PID_Values.kP).withPosition(0,0).getEntry();
  private NetworkTableEntry kI =
    hoodPID.add("kI", Constants.Hood_Constants.PID_Values.kI).withPosition(1,0).getEntry();
  private NetworkTableEntry kD =
    hoodPID.add("kD", Constants.Hood_Constants.PID_Values.kD).withPosition(2,0).getEntry();
  private NetworkTableEntry shufflePIDenabled =
    hoodPID.add("PID Enabled", false).withPosition(0,2).getEntry();
  private NetworkTableEntry shuffleTargetSetpoint =
    hoodPID.add("Target Setpoint", targetPosition).withPosition(1,2).getEntry();
  private NetworkTableEntry shuffleCurrentError =
    hoodPID.add("Setpoint Error", 0).withPosition(1,2).getEntry();

  public Shooter_Hood() {
    shooterHood = new WPI_TalonSRX(Constants.Hood_Constants.hoodTalon_ID);

    shooterHood.configFactoryDefault();
    shooterHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                              Constants.Hood_Constants.kPIDLoopIdx,
                                              Constants.Hood_Constants.kTimeoutMs);
    shooterHood.setSensorPhase(Constants.Hood_Constants.kSensorPhase);

    /* Config the peak and nominal outputs, 12V means full */
		shooterHood.configNominalOutputForward(Constants.Hood_Constants.minForwardPower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configNominalOutputReverse(Constants.Hood_Constants.minReversePower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configPeakOutputForward(Constants.Hood_Constants.maxForwardPower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configPeakOutputReverse(Constants.Hood_Constants.maxReversePower, Constants.Hood_Constants.kTimeoutMs);

    /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		shooterHood.configAllowableClosedloopError(Constants.Hood_Constants.hoodTolerance,
                                          Constants.Hood_Constants.kPIDLoopIdx,
                                          Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kP(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kP, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kI(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kI, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kD(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kD, Constants.Hood_Constants.kTimeoutMs);
    currentP = Constants.Hood_Constants.PID_Values.kP;
    currentI = Constants.Hood_Constants.PID_Values.kI;
    currentD = Constants.Hood_Constants.PID_Values.kD;

    /**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = shooterHood.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.Hood_Constants.kSensorPhase) { absolutePosition *= -1; }
		if (Constants.Hood_Constants.kMotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		shooterHood.setSelectedSensorPosition(absolutePosition,
                                          Constants.Hood_Constants.kPIDLoopIdx,
                                          Constants.Hood_Constants.kTimeoutMs);

    // Check if the hood is already in the home position
    if (isAtHome()) homed = true;
  }

  @Override
  public void periodic() {
    checkHome();
    checkPIDDone();

    hoodPositionTalon.setDouble(getPosition());
    isHomeIndicator.setBoolean(isAtHome());
    isHomedIndicator.setBoolean(isHomed());

    double p = kP.getDouble(0);
    double i = kI.getDouble(0);
    double d = kD.getDouble(0);

    if(p != currentP) {
      shooterHood.config_kP(Constants.Hood_Constants.kPIDLoopIdx, p, Constants.Hood_Constants.kTimeoutMs);
      currentP = p;
    }
    if(i != currentI) {
      shooterHood.config_kI(Constants.Hood_Constants.kPIDLoopIdx, i, Constants.Hood_Constants.kTimeoutMs);
      currentI = i;
    }
    if(d != currentD) {
      shooterHood.config_kD(Constants.Hood_Constants.kPIDLoopIdx, d, Constants.Hood_Constants.kTimeoutMs);
      currentD = d;
    };

    shufflePIDenabled.setBoolean(pidEnabled);
    shuffleTargetSetpoint.setDouble(targetPosition);
    shuffleCurrentError.setDouble(shooterHood.getClosedLoopError());
    
    hoodPower.setDouble(shooterHood.getMotorOutputPercent());
  }

  public double getPosition() {
    return shooterHood.getSelectedSensorPosition(0);
  }

  public void setHoodPower(double power)
  {
    double encValue = getPosition();
    double _power = power;
    
    if (_power > 0) {
      if (encValue >= Constants.Hood_Constants.hoodMaxPosition) {
        shooterHood.stopMotor();
      } else {
        if (encValue >= Constants.Hood_Constants.hoodMaxPosition - Constants.Hood_Constants.hoodDecellerationDistance) {
          _power *= (Constants.Hood_Constants.hoodMaxPosition - encValue) /
                    Constants.Hood_Constants.hoodDecellerationDistance;
        }
        shooterHood.set(Math.min(Constants.Hood_Constants.maxForwardPower, 
                                 Math.max(_power, Constants.Hood_Constants.minForwardPower)));
      }
    } else if (_power < 0) {
      if (encValue <= 0) {
        shooterHood.stopMotor();
      } else {
        if (encValue <= Constants.Hood_Constants.hoodDecellerationDistance) {
          _power *= -encValue / Constants.Hood_Constants.hoodDecellerationDistance;
        }
        shooterHood.set(Math.max(Constants.Hood_Constants.maxReversePower, 
                                 Math.min(_power, Constants.Hood_Constants.minReversePower)));
      }
    } else {
      shooterHood.stopMotor();
    }
  }

  public void moveHome() {
    shooterHood.set(Constants.Hood_Constants.homingPower);
  }

  public void setTargetPosition(double setpoint) {
    shooterHood.set(ControlMode.Position, setpoint);
  }

  public void changeSetpoint(String direction) {
    switch(direction) {
      case "up": {
        targetPosition = Math.min(targetPosition + 1000, Constants.Hood_Constants.hoodMaxPosition);
        pidEnabled = true;
        setTargetPosition(targetPosition);
        break;
      }
      case "down": {
        targetPosition = Math.max(targetPosition - 1000, 0);
        pidEnabled = true;
        setTargetPosition(targetPosition);
        break;
      }
      default: break;
    }
  }

  public void goPos(int positionNum) {
    switch(positionNum) {
      case 1: {
        targetPosition = Constants.Hood_Constants.Positions.pos1;
        pidEnabled = true;
        setTargetPosition(targetPosition);
        break;
      }
      case 2: {
        targetPosition = Constants.Hood_Constants.Positions.pos2;
        pidEnabled = true;
        setTargetPosition(targetPosition);
        break;
      }
      default: break;
    }
  }

  public void checkHome() {
    if (isAtHome()) {
      shooterHood.setSelectedSensorPosition(0);
      homed = true;
    }
  }

  public boolean isAtHome() {
    return shooterHood.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public boolean isHomed() {
    return homed;
  }

  public void checkPIDDone() {
    if (Math.abs(getPosition() - targetPosition) <= 20) {
      pidEnabled = false;
    }
  }

  public boolean getPIDEnabled() {
    return pidEnabled;
  }
}