// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.SmartMotorController;
import edu.wpi.first.math.controller.ArmFeedforward;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
  

public class IntakePivot extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig;
  private SparkMax spark;
  private SmartMotorController sparkSmartMotorController;
  private ArmConfig armCfg;
  private Arm arm;

  /** Creates a new IntakePivot. */
  public IntakePivot() {
    smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(9), DegreesPerSecondPerSecond.of(5))
      .withSimClosedLoopController(500, 0, 0, DegreesPerSecond.of(9), DegreesPerSecondPerSecond.of(5))
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

    spark = new SparkMax(20, MotorType.kBrushless);
    sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
    
    armCfg = new ArmConfig(sparkSmartMotorController)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Degrees.of(-40), Degrees.of(40))
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(-80), Degrees.of(80))
      // Starting position is where your arm starts
      .withStartingPosition(Degrees.of(-5))
      // Length and mass of your arm for sim.
      .withLength(Feet.of(1))
      .withMass(Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    arm = new Arm(armCfg);
    System.out.println("IntakePivot initialized successfully");
  }


  // Arm Mechanism
  
  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */

  public Command setAngle(Angle angle) { return arm.run(angle);}

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) { return arm.runTo(angle, tolerance);}

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) { arm.setMechanismPositionSetpoint(angle); }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  @Override
public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
    SmartDashboard.putNumber("Arm Angle", arm.getMechanismSetpoint().orElse(Degrees.of(0)).in(Degrees));
    SmartDashboard.putNumber("Mechanism Position Setpoint", arm.getMechanismSetpoint().orElse(Degrees.of(0)).in(Degrees));
  }

public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  arm.simIterate();
}
}
