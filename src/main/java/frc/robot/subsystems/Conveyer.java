// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

public class Conveyer extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder;

  // Jam detection thresholds (motor-side RPM)
  private static final double JAM_VELOCITY_THRESHOLD = 100.0;
  private static final double UNJAM_VELOCITY_THRESHOLD = 200.0;
  private static final double STARTUP_GRACE_SECONDS = 0.25;

  /** Creates a new Conveyer. */
  public Conveyer() {
    motor = new SparkMax(23, MotorType.kBrushless);
    encoder = motor.getEncoder();
    
    configureDevices();
  }

  // Set current limits and config motor
  private void configureDevices() {
    try {
      motorConfig = new SparkMaxConfig();
      motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure Conveyer Subsystem", ex.getStackTrace());
    }
  }

  /**
   * Set the roller motor speed with duty cycle control.
   * @param speed [-1, 1] speed to set the roller to.
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  /**
   * Set the roller speed as a command.
   * @param speed [-1, 1] speed to set the roller to.
   * @return A command that sets the speed.
   */
  public Command setSpeedCommand(double speed) {
    return this.run(() -> setSpeed(speed)).finallyDo(() -> stop());
  }

  /** Get conveyer velocity in RPM (motor-side). */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /** Get conveyer motor current draw in amps. */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Smart conveyer command with jam detection.
   * Runs the conveyer at the given speed. If a jam is detected (velocity drops
   * below threshold), reverses until unjammed, then resumes. Repeats as needed.
   *
   * @param speed The normal speed (e.g. 1 for full forward)
   * @return A command that handles jam detection and reversal
   */
  public Command smartCommand(double speed) {
    return Commands.sequence(
      Commands.runOnce(() -> setSpeed(speed), this),
      Commands.waitSeconds(STARTUP_GRACE_SECONDS),
      this.run(() -> setSpeed(speed))
          .until(() -> Math.abs(getVelocity()) < JAM_VELOCITY_THRESHOLD),
      this.run(() -> setSpeed(-speed))
          .until(() -> Math.abs(getVelocity()) > UNJAM_VELOCITY_THRESHOLD)
    ).repeatedly()
     .finallyDo(() -> stop());
  }

  /**
   * Stop the roller.
   */
  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Conveyer/Velocity RPM", getVelocity());
    SmartDashboard.putNumber("Conveyer/Current Amps", getCurrent());
  }
}

