// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.NetworkValues;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // Motors
  private final TalonFX leftFlywheel  = new TalonFX(leftFlywheelID,  canBus);
  private final TalonFX rightFlywheel = new TalonFX(rightFlywheelID, canBus);
  private final TalonFX hood          = new TalonFX(hoodMotorID,     canBus);
  private final TalonFX feeder        = new TalonFX(feederMotorID,   canBus);

  // Reusable control requests
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final NeutralOut       neutralRequest = new NeutralOut();

  // Setpoints
  private double leftFlywheelSetpointRPM  = 0.0;
  private double rightFlywheelSetpointRPM = 0.0;
  private double hoodSetpointRad          = hoodMinAngleRad;

  // Disconnection alerts
  private final Alert leftFlywheelDisconnected =
      new Alert("Left flywheel motor disconnected!",  Alert.AlertType.kWarning);
  private final Alert rightFlywheelDisconnected =
      new Alert("Right flywheel motor disconnected!", Alert.AlertType.kWarning);
  private final Alert hoodDisconnected =
      new Alert("Hood motor disconnected!",   Alert.AlertType.kWarning);
  private final Alert feederDisconnected =
      new Alert("Feeder motor disconnected!", Alert.AlertType.kWarning);

  public ShooterSubsystem() {
    configureMotors();
  }

  private void configureMotors() {
    // --- Flywheel config ---
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kP = flywheelKp;
    flywheelConfig.Slot0.kI = flywheelKi;
    flywheelConfig.Slot0.kD = flywheelKd;
    flywheelConfig.Slot0.kV = flywheelKv;
    flywheelConfig.Slot0.kS = flywheelKs;
    flywheelConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftFlywheel.getConfigurator().apply(flywheelConfig);

    // Invert the right flywheel — flip to CounterClockwise_Positive if it spins the wrong way
    TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.Slot0.kP = flywheelKp;
    rightFlywheelConfig.Slot0.kI = flywheelKi;
    rightFlywheelConfig.Slot0.kD = flywheelKd;
    rightFlywheelConfig.Slot0.kV = flywheelKv;
    rightFlywheelConfig.Slot0.kS = flywheelKs;
    rightFlywheelConfig.Feedback.SensorToMechanismRatio = flywheelGearRatio;
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightFlywheel.getConfigurator().apply(rightFlywheelConfig);

    // --- Feeder config ---
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Slot0.kP = feederKp;
    feederConfig.Slot0.kI = feederKi;
    feederConfig.Slot0.kD = feederKd;
    feederConfig.Slot0.kV = feederKv;
    feederConfig.Feedback.SensorToMechanismRatio = feederGearRatio;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feeder.getConfigurator().apply(feederConfig);

    // --- Hood config ---
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = hoodKp;
    hoodConfig.Slot0.kI = hoodKi;
    hoodConfig.Slot0.kD = hoodKd;
    hoodConfig.Feedback.SensorToMechanismRatio = hoodGearRatio;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Soft limits to protect the mechanism (radians -> rotations)
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodMaxAngleRad / (2 * Math.PI);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = hoodMinAngleRad / (2 * Math.PI);
    hood.getConfigurator().apply(hoodConfig);

    // Seed the hood encoder to the min angle on startup
    hood.setPosition(hoodMinAngleRad / (2 * Math.PI));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Measured Hood Deg", Units.radiansToDegrees(getHoodPosition()));
}
    // Stop flywheels and feeder when disabled; hood holds position via brake mode
    if (DriverStation.isDisabled()) {
      leftFlywheelSetpointRPM  = 0.0;
      rightFlywheelSetpointRPM = 0.0;
      leftFlywheel.setControl(neutralRequest);
      rightFlywheel.setControl(neutralRequest);
      feeder.setControl(neutralRequest);
    }

    // Disconnection alerts
    leftFlywheelDisconnected.set(!leftFlywheel.isConnected());
    rightFlywheelDisconnected.set(!rightFlywheel.isConnected());
    hoodDisconnected.set(!hood.isConnected());
    feederDisconnected.set(!feeder.isConnected());

    //actual hood and shooting positions
    // System.out.println("Left flywheel velocity: " + getLeftFlywheelVelocity());
    // System.out.println("Right flywheel velocity: " + getRightFlywheelVelocity());
    // System.out.println("Hood position: " + Units.radiansToDegrees(getHoodPosition()));
  }

  // ---------------------------------------------------------------------------
  // Setters
  // ---------------------------------------------------------------------------

  /** Sets both flywheels to the same speed. */
  public void setFlywheelVelocity(double velocityRPM) {
    setFlywheelVelocities(velocityRPM, velocityRPM);

  }

  /** Sets left and right flywheels to independent speeds. */
  public void setFlywheelVelocities(double leftVelocityRPM, double rightVelocityRPM) {
    leftFlywheelSetpointRPM  = leftVelocityRPM;
    rightFlywheelSetpointRPM = rightVelocityRPM;
    // TalonFX velocity control uses rotations-per-second
    leftFlywheel.setControl(velocityRequest.withVelocity(leftVelocityRPM / 60.0));
    rightFlywheel.setControl(velocityRequest.withVelocity(rightVelocityRPM / 60.0));
  }

  /**
   * Sets the hood angle.
   *
   * @param angleRad Target angle in radians (mechanism angle after gear reduction).
   */
  public void setHoodPosition(double angleRad) {
    hoodSetpointRad = angleRad;
    // TalonFX position control uses rotations
    hood.setControl(positionRequest.withPosition(angleRad / (2 * Math.PI)));
  }

  /** Runs the feeder to feed a note up to the shooter. */
  public void feedNote() {
    feeder.setControl(velocityRequest.withVelocity(feederFeedSpeedRPM / 60.0));
  }

  /** Runs the feeder in reverse to eject a note back toward the hopper. */
  public void ejectNote() {
    feeder.setControl(velocityRequest.withVelocity(feederEjectSpeedRPM / 60.0));
  }

  /** Stops the feeder. */
  public void stopFeeder() {
    feeder.setControl(neutralRequest);
  }

  /** Stops flywheels and holds hood position. Does NOT stop the feeder. */
  public void stop() {
    leftFlywheelSetpointRPM  = 0.0;
    rightFlywheelSetpointRPM = 0.0;
    leftFlywheel.setControl(neutralRequest);
    rightFlywheel.setControl(neutralRequest);
    hood.setControl(positionRequest.withPosition(hoodSetpointRad / (2 * Math.PI)));
  }

  // ---------------------------------------------------------------------------
  // Getters
  // ---------------------------------------------------------------------------

  /** @return Current left flywheel velocity in RPM. */
  public double getLeftFlywheelVelocity() {
    return leftFlywheel.getVelocity().getValueAsDouble() * 60.0;
  }

  /** @return Current right flywheel velocity in RPM. */
  public double getRightFlywheelVelocity() {
    return rightFlywheel.getVelocity().getValueAsDouble() * 60.0;
  }

  /** @return Current hood mechanism angle in radians. */
  public double getHoodPosition() {
    return hood.getPosition().getValueAsDouble() * (2 * Math.PI);
  }

  /** @return Average of left and right flywheel velocities in RPM. */
  public double getAverageFlywheelVelocity() {
    return (getLeftFlywheelVelocity() + getRightFlywheelVelocity()) / 2.0;
  }

  // ---------------------------------------------------------------------------
  // At-goal checks
  // ---------------------------------------------------------------------------

  /** Returns true when the left flywheel is within tolerance of its setpoint. */
  public boolean isLeftFlywheelAtSpeed() {
    return Math.abs(getLeftFlywheelVelocity() - leftFlywheelSetpointRPM) <= flywheelToleranceRPM;
  }

  /** Returns true when the right flywheel is within tolerance of its setpoint. */
  public boolean isRightFlywheelAtSpeed() {
    return Math.abs(getRightFlywheelVelocity() - rightFlywheelSetpointRPM) <= flywheelToleranceRPM;
  }

  /** Returns true when both flywheels are at their setpoints. */
  public boolean areFlywheelsAtSpeed() {
    return leftFlywheelSetpointRPM > 0.0 && isLeftFlywheelAtSpeed() && isRightFlywheelAtSpeed();
  }

  /** Returns true when the hood is within tolerance of its setpoint. */
  public boolean isHoodAtPosition() {
    return Math.abs(getHoodPosition() - hoodSetpointRad) <= hoodToleranceRad;
  }

  /** Returns true when the robot is fully ready to shoot. */
  public boolean isReadyToShoot() {
    return areFlywheelsAtSpeed() && isHoodAtPosition();
  }

  /**
   * Convenience method — sets hood and both flywheels in one call.
   *
   * @param hoodAngleRad     Target hood angle in radians.
   * @param flywheelSpeedRPM Target flywheel speed in RPM (applied to both wheels equally).
   */
  public void applyShootingParameters(double hoodAngleRad, double flywheelSpeedRPM) {
    setHoodPosition(hoodAngleRad);
    setFlywheelVelocity(flywheelSpeedRPM);
  }
}