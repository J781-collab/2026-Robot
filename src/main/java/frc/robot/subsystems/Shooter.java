package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    // Control requests (reused to avoid garbage collection)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // PID gains for velocity control — tune these!
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;  // Static friction feedforward
    private static final double kV = 0.12; // Velocity feedforward (volts per RPS)

    public Shooter() {
        shooterMotor1 = new TalonFX(41);
        shooterMotor2 = new TalonFX(42);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Coast mode for shooter wheels
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity PID gains in Slot 0
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;

        shooterMotor1.getConfigurator().apply(config);
        shooterMotor2.getConfigurator().apply(config);
    }

    // ---- Velocity PID Control ---- //

    /**
     * Set both shooter motors to a target velocity using closed-loop PID.
     * @param velocityRPS Target velocity in rotations per second.
     */
    public void setVelocity(double velocityRPS) {
        shooterMotor1.setControl(velocityRequest.withVelocity(velocityRPS));
        shooterMotor2.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    /**
     * Set shooter velocity as a command.
     * @param velocityRPS Target velocity in rotations per second.
     * @return A command that runs the shooter at the target velocity and stops when finished.
     */
    public Command setVelocityCommand(double velocityRPS) {
        return this.run(() -> setVelocity(velocityRPS)).finallyDo(() -> stop());
    }

    /**
     * Check if the shooter is at the target velocity within a tolerance.
     * @param targetRPS Target velocity in rotations per second.
     * @param toleranceRPS Acceptable error in RPS.
     * @return True if both motors are within tolerance.
     */
    public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
        return Math.abs(getMotor1VelocityRPS() - targetRPS) < toleranceRPS
            && Math.abs(getMotor2VelocityRPS() - targetRPS) < toleranceRPS;
    }

    // ---- Duty Cycle (Open Loop) Control ---- //

    /**
     * Set both shooter motors with duty cycle (open loop).
     * @param speed [-1, 1] duty cycle percentage.
     */
    public void setDutyCycle(double speed) {
        shooterMotor1.setControl(dutyCycleRequest.withOutput(speed));
        shooterMotor2.setControl(dutyCycleRequest.withOutput(speed));
    }

    /**
     * Set shooter duty cycle as a command.
     * @param speed [-1, 1] duty cycle percentage.
     * @return A command that sets the duty cycle and stops when finished.
     */
    public Command setDutyCycleCommand(double speed) {
        return this.run(() -> setDutyCycle(speed)).finallyDo(() -> stop());
    }

    // ---- Getters ---- //

    /** Get motor 1 velocity in rotations per second. */
    public double getMotor1VelocityRPS() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    /** Get motor 2 velocity in rotations per second. */
    public double getMotor2VelocityRPS() {
        return shooterMotor2.getVelocity().getValueAsDouble();
    }

    /** Get motor 1 position in rotations. */
    public double getMotorPosition() {
        return shooterMotor1.getPosition().getValueAsDouble();
    }

    // ---- Stop ---- //

    /** Stop both shooter motors. */
    public void stop() {
        shooterMotor1.setControl(dutyCycleRequest.withOutput(0));
        shooterMotor2.setControl(dutyCycleRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor1 RPS", getMotor1VelocityRPS());
        SmartDashboard.putNumber("Shooter Motor2 RPS", getMotor2VelocityRPS());
    }
}