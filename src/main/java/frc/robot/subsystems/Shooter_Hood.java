/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {

  private WPI_TalonSRX shooterHood;

  // Hood control
  private boolean homed = false;

  // SHUFFLEBOARD
  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

  private NetworkTableEntry hoodPositionTalon =
    tab.add("Hood Position (Talon)", 0)
    .withPosition(2,2)
    .withSize(1,1)
    .getEntry();
  private NetworkTableEntry hoodPower =
    tab.add("Hood Power", 0)
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

  public Shooter_Hood() {
    shooterHood = new WPI_TalonSRX(Constants.Hood_Constants.hoodTalon_ID);

    shooterHood.configFactoryDefault();
    shooterHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                              Constants.Hood_Constants.kPIDLoopIdx,
                                              Constants.Hood_Constants.kTimeoutMs);
    shooterHood.setSensorPhase(Constants.Hood_Constants.kSensorPhase);

    /* Config the peak and nominal outputs */
		shooterHood.configNominalOutputForward(Constants.Hood_Constants.minForwardPower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configNominalOutputReverse(Constants.Hood_Constants.minReversePower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configPeakOutputForward(Constants.Hood_Constants.maxForwardPower, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.configPeakOutputReverse(Constants.Hood_Constants.maxReversePower, Constants.Hood_Constants.kTimeoutMs);
  
    setCoastMode();    // allow hood to be moved manually

    /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
    shooterHood.configAllowableClosedloopError(Constants.Hood_Constants.kPIDLoopIdx,
                                          Constants.Hood_Constants.hoodTolerance,
                                          Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kP(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kP, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kI(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kI, Constants.Hood_Constants.kTimeoutMs);
		shooterHood.config_kD(Constants.Hood_Constants.kPIDLoopIdx,
                          Constants.Hood_Constants.PID_Values.kD, Constants.Hood_Constants.kTimeoutMs);

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
    if (isAtHome()) {
      setAtHome();
    }
  }

  @Override
  public void periodic() {
    hoodPositionTalon.setDouble(getPosition());
    isHomeIndicator.setBoolean(isAtHome());
    isHomedIndicator.setBoolean(isHomed());
    hoodPower.setDouble(shooterHood.getMotorOutputPercent());
  }

  public void setCoastMode() {
    shooterHood.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    shooterHood.setNeutralMode(NeutralMode.Brake);
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

  // This is only valid following a set position command
  public boolean isAtTarget() {
    return (shooterHood.getClosedLoopError(Constants.Hood_Constants.kPIDLoopIdx) <= 
            Constants.Hood_Constants.hoodTolerance);
  }

  public void setAtHome() {
    shooterHood.setSelectedSensorPosition(0);
    homed = true;
  }

  public boolean isAtHome() {
    return shooterHood.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public boolean isHomed() {
    return homed;
  }
}