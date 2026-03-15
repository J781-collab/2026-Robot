// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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

public class IntakePivot extends SubsystemBase {

    /**
     * Pivot motor(s) pivot the intake wrist.
     */
    private SparkMax leaderPivotMotor;
    private SparkMaxConfig leadMotorConfig;

    private RelativeEncoder pivotEncoder;

    private PIDController pivotController;

    private double goalPosition;

    public IntakePivot() {
        // Config pivot motor
        leaderPivotMotor = new SparkMax(IntakePivotConstants.leaderMotorID, MotorType.kBrushless);

        // Get built-in NEO encoder
        pivotEncoder = leaderPivotMotor.getEncoder();



        // Config pivot PID
        pivotController = new PIDController(
            IntakePivotConstants.IntakePivotPID.P,
            IntakePivotConstants.IntakePivotPID.I,
            IntakePivotConstants.IntakePivotPID.D
        );
      //  pivotController.enableContinuousInput(0, 360);

        // Apply motor/encoder configs
        configureDevices();

        // Zero the encoder at startup so current position reads as 0
        zeroEncoder();

        // Set goal to idle position (-45 degrees)
        goalPosition = IntakePivotConstants.idlePosition;
    }

    /** Set current limits, configure motors and encoders. */
    private void configureDevices() {
        try {
            // Lead pivot motor
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)
                .smartCurrentLimit(40)
                .openLoopRampRate(1)
                .idleMode(IdleMode.kBrake);

            // Convert NEO encoder from motor rotations to mechanism degrees
            // 1 motor rotation = (360 / 35) degrees of pivot
            leadMotorConfig.encoder
                .positionConversionFactor(70.0 / 7.0)
                .velocityConversionFactor(70.0 / 7.0 / 12.0);

            leaderPivotMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure IntakePivot Subsystem", ex.getStackTrace());
        }
    }

    /**
     * Zero the NEO encoder so current position reads as 0 degrees.
     */
    public void zeroEncoder() {
        pivotEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot Position", getPivotEncoderPosition());
        SmartDashboard.putNumber("IntakePivot Goal", getPivotGoal());
        SmartDashboard.putNumber("IntakePivot Velocity", pivotEncoder.getVelocity());
        SmartDashboard.putNumber("IntakePivot Motor Output", leaderPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("IntakePivot Motor Current", leaderPivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakePivot PID Error", pivotController.getPositionError());
        SmartDashboard.putBoolean("IntakePivot At Goal", Math.abs(getPivotEncoderPosition() - goalPosition) < 0.5);
        SmartDashboard.putData("IntakePivot PID", pivotController);
        SmartDashboard.putData(this);
    }

    /** Get the position of the pivot in degrees (already converted via positionConversionFactor). */
    public double getPivotEncoderPosition() {
        return pivotEncoder.getPosition();
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

    public Command pivotArm(double desiredpos) {
        return Commands
            .runOnce(
                () -> {
                    leaderPivotMotor.set(0.0);
                }, this
            ).andThen(
                Commands.run(
                    () -> {
                        double pos = getPivotEncoderPosition();
                        double error = desiredpos-pos;
                        if (Math.abs(error) < 5){
                            leaderPivotMotor.set(0);
                        }else if (pos < desiredpos) {
                            leaderPivotMotor.set(0.25);
                        } else if (pos > desiredpos) {
                             leaderPivotMotor.set(-0.25);
                        }// negative makes it clockwise
                    }, this
                ).finallyDo(() -> leaderPivotMotor.set(0.0))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }
}
