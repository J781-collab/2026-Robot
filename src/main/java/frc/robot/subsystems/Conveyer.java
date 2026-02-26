// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Conveyer extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;

  /** Creates a new Conveyer. */
  public Conveyer() {
    motor = new SparkMax(23, MotorType.kBrushless);
    
    configureDevices();
  }

  // Set current limits and config motor
  private void configureDevices() {
    try {
      motorConfig = new SparkMaxConfig();
      motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

      motor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
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

  /**
   * Stop the roller.
   */
  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

