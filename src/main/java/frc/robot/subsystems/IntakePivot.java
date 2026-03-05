// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.IntakePivotConstants;
import edu.wpi.first.wpilibj.XboxController;

public class IntakePivot extends SubsystemBase {

    /**
     * Pivot motor(s) pivot the intake wrist.
     */
    private SparkMax leaderPivotMotor;
    private SparkMaxConfig leadMotorConfig;

    private CANcoder pivotEncoder;
    private CANcoderConfiguration pivotCANcoderConfig;

    private PIDController pivotController;

    private double goalPosition;

    public IntakePivot() {
        // Config pivot motor
        leaderPivotMotor = new SparkMax(IntakePivotConstants.leaderMotorID, MotorType.kBrushless);

        // Config pivot encoder
        pivotEncoder = new CANcoder(IntakePivotConstants.encoderID);

        // Config pivot PID
        pivotController = new PIDController(
            IntakePivotConstants.IntakePivotPID.P,
            IntakePivotConstants.IntakePivotPID.I,
            IntakePivotConstants.IntakePivotPID.D
        );
        pivotController.enableContinuousInput(0, 360);

        // Apply motor/encoder configs
        configureDevices();

        // Set goal to idle position (-45 degrees)
        goalPosition = IntakePivotConstants.idlePosition;
    }

    /** Set current limits, configure motors and encoders. */
    private void configureDevices() {
        try {
            // Lead pivot motor
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(true)
                .smartCurrentLimit(40)
                .closedLoopRampRate(1)
                .idleMode(IdleMode.kBrake);

            leaderPivotMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Pivot CANcoder
            pivotCANcoderConfig = new CANcoderConfiguration();
            pivotEncoder.getConfigurator().apply(
                pivotCANcoderConfig.MagnetSensor
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(-IntakePivotConstants.encoderOffset)
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure IntakePivot Subsystem", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot Absolute Position", pivotEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakePivot Position", getPivotEncoderPosition());
        SmartDashboard.putNumber("IntakePivot Goal", getPivotGoal());
        SmartDashboard.putData(this);
    }

    /** Get the position of the pivotEncoder in degrees. */
    public double getPivotEncoderPosition() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    /** Get the current pivot goal of the PID. */
    public double getPivotGoal() {
        return goalPosition;
    }

    /**
     * Set the goal position of the pivot (degrees).
     * Clamps to min/max and rejects out-of-bounds requests.
     *
     * @param position target angle in degrees
     * @return a command
     */
    public Command setPivotGoal(double position) {
        return Commands
            .runOnce(
                () -> {
                    goalPosition = MathUtil.clamp(position, IntakePivotConstants.minPivotPos, IntakePivotConstants.maxPivotPos);
                }, this
            ).unless(
                () -> (position > IntakePivotConstants.maxPivotPos) || (position < IntakePivotConstants.minPivotPos)
            );
    }

    /**
     * Pivot the wrist toward the goal position using PID.
     * Ends when the position is within 0.5 degrees of the goal.
     *
     * @return a command
     */
    public Command pivotArm() {
        return Commands
            .runOnce(
                () -> {
                    leaderPivotMotor.set(0.0);
                }, this
            ).andThen(
                Commands.run(
                    () -> {
                        double pos = getPivotEncoderPosition();
                        if (pos > IntakePivotConstants.minPivotPos && pos < IntakePivotConstants.maxPivotPos) {
                            leaderPivotMotor.set(
                                MathUtil.clamp(
                                    pivotController.calculate(pos, goalPosition),
                                    -0.1, 0.1
                                )
                            );
                        } else {
                            leaderPivotMotor.set(0.0);
                        }
                    }, this
                ).until(
                    () -> Math.abs(getPivotEncoderPosition() - goalPosition) < 0.5
                ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }
}
