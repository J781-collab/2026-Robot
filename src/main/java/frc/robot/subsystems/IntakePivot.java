// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
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
    private boolean isCoastMode = false;

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

    /** Set current limits, configure motors and encoders. Only burn flash if config differs. */
    private void configureDevices() {
        try {
            // Read the current flash config to see if we need to burn
            boolean needsBurnFlash = configNeedsBurnFlash();

            // Lead pivot motor
            leadMotorConfig = new SparkMaxConfig();
            leadMotorConfig
                .inverted(false)
                .smartCurrentLimit(20)  // Low limit — 100:1 gear ratio means huge torque
                .openLoopRampRate(0)
                .idleMode(IdleMode.kBrake);

            // Soft limits — prevent pivot from going past 0 (forward) or -420 (reverse)
            leadMotorConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(0.0)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(-420.0);

            // Convert NEO encoder from motor rotations to mechanism degrees
            // 1 motor rotation = (360 / 35) degrees of pivot
            leadMotorConfig.encoder
                .positionConversionFactor(70.0 / 7.0)
                .velocityConversionFactor(70.0 / 7.0 / 12.0);

            PersistMode persistMode = needsBurnFlash
                ? PersistMode.kPersistParameters
                : PersistMode.kNoPersistParameters;

            leaderPivotMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, persistMode);

            if (needsBurnFlash) {
                DriverStation.reportWarning("IntakePivot: Config changed — burned flash!", false);
            } else {
                DriverStation.reportWarning("IntakePivot: Config matches flash — skipped burn.", false);
            }

        } catch (Exception ex) {
            DriverStation.reportError("Failed to configure IntakePivot Subsystem", ex.getStackTrace());
        }
    }

    /** Compare desired config values against what's already stored in flash. */
    private boolean configNeedsBurnFlash() {
        try {
            var accessor = leaderPivotMotor.configAccessor;
            double epsilon = 0.01;

            // Check current limit
            if (accessor.getSmartCurrentLimit() != 20) return true;

            // Check inverted
            if (accessor.getInverted() != false) return true;

            // Check idle mode
            if (accessor.getIdleMode() != IdleMode.kBrake) return true;

            // Check open loop ramp rate
            if (Math.abs(accessor.getOpenLoopRampRate() - 0.0) > epsilon) return true;

            // Check soft limits
            if (!accessor.softLimit.getForwardSoftLimitEnabled()) return true;
            if (Math.abs(accessor.softLimit.getForwardSoftLimit() - 0.0) > epsilon) return true;
            if (!accessor.softLimit.getReverseSoftLimitEnabled()) return true;
            if (Math.abs(accessor.softLimit.getReverseSoftLimit() - (-420.0)) > epsilon) return true;

            // Check encoder conversion factors
            if (Math.abs(accessor.encoder.getPositionConversionFactor() - (70.0 / 7.0)) > epsilon) return true;
            if (Math.abs(accessor.encoder.getVelocityConversionFactor() - (70.0 / 7.0 / 12.0)) > epsilon) return true;

            return false;  // Everything matches — no burn needed
        } catch (Exception ex) {
            // If we can't read config, burn flash to be safe
            DriverStation.reportWarning("IntakePivot: Could not read config — will burn flash.", false);
            return true;
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
        SmartDashboard.putString("IntakePivot Idle Mode", isCoastMode ? "Coast" : "Brake");
        SmartDashboard.putData(this);
    }

    /** Get the position of the pivot in degrees (already converted via positionConversionFactor). */
    public double getPivotEncoderPosition() {
        return pivotEncoder.getPosition();
    }

    /** Get the pivot motor velocity in converted units. */
    public double getVelocity() {
        return pivotEncoder.getVelocity();
    }

    /** Get the pivot motor current draw in amps. */
    public double getCurrent() {
        return leaderPivotMotor.getOutputCurrent();
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
                            leaderPivotMotor.set(1.0);
                        } else if (pos > desiredpos) {
                             leaderPivotMotor.set(-1.0);
                        }// negative makes it clockwise
                    }, this
                ).finallyDo(() -> leaderPivotMotor.set(0.0))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }

    /**
     * Pivot the wrist toward the desired position using PID control.
     * Continuously runs the PID loop; stops the motor when interrupted.
     *
     * @param desiredPos target angle in converted encoder units
     * @return a command
     */
    public Command pivotArmPID(double desiredPos) {
        return Commands.run(
            () -> {
                double output = pivotController.calculate(getPivotEncoderPosition(), desiredPos);
                output = MathUtil.clamp(output, -1.0, 1.0);
                leaderPivotMotor.set(output);
            }, this
        ).finallyDo(() -> leaderPivotMotor.set(0.0))
         .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /**
     * Smart agitation command that drives inward until it hits the ball pile
     * (current spike + velocity drop), backs off, then slams back in.
     * Each cycle it can go deeper as balls are shot out and the pile shrinks.
     * 
     * Behavior:
     *   1. Drive continuously inward toward hardLimit
     *   2. When current spikes AND velocity drops → hit the ball pile
     *   3. Record the hit position, retreat by RETREAT_AMOUNT
     *   4. Once retreat position is reached, slam back inward toward hardLimit
     *   5. Repeat — will go deeper each time as pile shrinks
     * 
     * Uses PID control for smooth motion.
     *
     * @param startPos         Starting inward position (e.g. -100)
     * @param hardLimit        Absolute deepest allowed position (e.g. -420)
     * @return A command that agitates with adaptive depth
     */
    public Command smartAgitateCommand(double startPos, double hardLimit) {
        // Mutable state
        final double[] targetPos = { startPos };
        final boolean[] pushingInward = { true };

        // Thresholds — conservative for 100:1 gear reduction
        final double HIT_CURRENT_THRESHOLD = 15.0;   // amps — huge torque at 100:1
        final double STALL_VELOCITY_THRESHOLD = 250.0; // ~1/20 of max motor converted velocity — stalled
        final double RETREAT_AMOUNT = 200.0;         // back off this many encoder units after a hit
        final int GRACE_CYCLES = 10;                  // ~200ms — let motor accelerate before checking
        final double ARRIVE_TOLERANCE = 15.0;        // "close enough" to retreat target
        final int PAUSE_CYCLES = 14;                 // ~0.25s pause between cycles

        final int[] cycleCount = { 0 };
        final int[] hitCount = { 0 };
        final double[] lastHitPos = { startPos };
        final boolean[] pausing = { false };

        return Commands.run(() -> {
            double current = Math.abs(getCurrent());
            double velocity = Math.abs(getVelocity());
            double pos = getPivotEncoderPosition();

            cycleCount[0]++;

            if (pushingInward[0]) {
                // Driving inward — target is always the hard limit
                targetPos[0] = hardLimit;

                // Check for hit (only after grace period to let motor accelerate)
                // Trigger on current spike OR velocity stall while driving inward
                boolean currentHit = current > HIT_CURRENT_THRESHOLD;
                // Motor-side velocity — if magnitude is below threshold, arm is stalled
                boolean velocityHit = velocity < STALL_VELOCITY_THRESHOLD;

                if (cycleCount[0] > GRACE_CYCLES
                        && (currentHit || velocityHit)) {
                    // Hit the pile! Retreat from current ACTUAL position
                    pushingInward[0] = false;
                    cycleCount[0] = 0;
                    hitCount[0]++;
                    lastHitPos[0] = pos;
                    // Clamp retreat target: never go past startPos or past 0
                    targetPos[0] = Math.min(pos + RETREAT_AMOUNT, Math.min(startPos, 0.0));
                }
            } else if (!pausing[0]) {
                // Retreating — wait for arm to actually reach retreat position
                if (Math.abs(pos - targetPos[0]) < ARRIVE_TOLERANCE) {
                    // Done retreating — pause before next push
                    pausing[0] = true;
                    cycleCount[0] = 0;
                }
            } else {
                // Pausing — hold position for ~0.25s before slamming back in
                if (cycleCount[0] >= PAUSE_CYCLES) {
                    pushingInward[0] = true;
                    pausing[0] = false;
                    cycleCount[0] = 0;
                }
            }

            // Clamp target so we never go past 0 (positive direction)
            targetPos[0] = Math.min(targetPos[0], 0.0);

            // PID drive toward current target
            double output = pivotController.calculate(pos, targetPos[0]);
            output = MathUtil.clamp(output, -1.0, 1.0);
            leaderPivotMotor.set(output);

            // Telemetry
            SmartDashboard.putNumber("IntakePivot/Agitate Target", targetPos[0]);
            SmartDashboard.putNumber("IntakePivot/Agitate Position", pos);
            SmartDashboard.putNumber("IntakePivot/Agitate Current", current);
            SmartDashboard.putNumber("IntakePivot/Agitate Velocity", velocity);
            SmartDashboard.putNumber("IntakePivot/Agitate Current Threshold", HIT_CURRENT_THRESHOLD);
            SmartDashboard.putNumber("IntakePivot/Agitate Velocity Threshold", STALL_VELOCITY_THRESHOLD);
            SmartDashboard.putNumber("IntakePivot/Agitate PID Output", output);
            SmartDashboard.putNumber("IntakePivot/Agitate Grace Count", cycleCount[0]);
            SmartDashboard.putNumber("IntakePivot/Agitate Hit Count", hitCount[0]);
            SmartDashboard.putNumber("IntakePivot/Agitate Last Hit Pos", lastHitPos[0]);
            SmartDashboard.putNumber("IntakePivot/Agitate Raw Velocity", getVelocity());
            SmartDashboard.putBoolean("IntakePivot/Agitate Pushing", pushingInward[0]);
            SmartDashboard.putBoolean("IntakePivot/Agitate Pausing", pausing[0]);
        }, this)
        .finallyDo(() -> leaderPivotMotor.set(0.0))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /** Set the intake pivot motor to coast mode. */
    public void setCoastMode() {
        SparkMaxConfig coastConfig = new SparkMaxConfig();
        coastConfig.idleMode(IdleMode.kCoast);
        leaderPivotMotor.configure(coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        isCoastMode = true;
    }

    /** Set the intake pivot motor to brake mode. */
    public void setBrakeMode() {
        SparkMaxConfig brakeConfig = new SparkMaxConfig();
        brakeConfig.idleMode(IdleMode.kBrake);
        leaderPivotMotor.configure(brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        isCoastMode = false;
    }
}
