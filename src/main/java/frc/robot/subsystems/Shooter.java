package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    // Control requests (reused to avoid garbage collection)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);

    // SysId routine — initialized in constructor after motors exist
    private final SysIdRoutine sysIdRoutine;

    // PID gains for velocity control — tune these!
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;  // Static friction feedforward
    private static final double kV = 0.12; // Velocity feedforward (volts per RPS)

    public Shooter() {
        shooterMotor1 = new TalonFX(41);
        shooterMotor2 = new TalonFX(42);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Default ramp rate (1 V/s)
                Volts.of(4), // Step voltage — 4V to prevent brownout
                null,        // Default timeout (10s)
                state -> SignalLogger.writeString("ShooterSysId_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    shooterMotor1.setControl(sysIdControl.withOutput(voltage.in(Volts)));
                    shooterMotor2.setControl(sysIdControl.withOutput(voltage.in(Volts)));
                },
                null,
                this
            )
        );

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

    // ---- SysId Commands ---- //

    /** Returns a command that will execute a quasistatic test in the given direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /** Returns a command that will execute a dynamic test in the given direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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

    // ---- Distance-Based Interpolation ---- //

    /**
     * Interpolation table: distance (meters) → shooter velocity (RPS).
     * Fill in real data from testing! These are placeholder values.
     */
    private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    static {
        shooterSpeedMap.put(1.0,  30.0);   // 1m   → 30 RPS
        shooterSpeedMap.put(1.5,  35.0);   // 1.5m → 35 RPS
        shooterSpeedMap.put(2.0,  40.0);   // 2m   → 40 RPS
        shooterSpeedMap.put(2.5,  45.0);   // 2.5m → 45 RPS
        shooterSpeedMap.put(3.0,  50.0);   // 3m   → 50 RPS
        shooterSpeedMap.put(3.5,  55.0);   // 3.5m → 55 RPS
        shooterSpeedMap.put(4.0,  60.0);   // 4m   → 60 RPS
        shooterSpeedMap.put(4.5,  65.0);   // 4.5m → 65 RPS
        shooterSpeedMap.put(5.0,  70.0);   // 5m   → 70 RPS
        shooterSpeedMap.put(6.0,  80.0);   // 6m   → 80 RPS
    }

    /**
     * Gets the interpolated shooter speed (RPS) for a given distance (meters).
     * Uses WPILib's InterpolatingDoubleTreeMap for automatic linear interpolation.
     * @param distanceMeters Distance to target in meters.
     * @return Interpolated shooter velocity in RPS.
     */
    public static double getInterpolatedSpeed(double distanceMeters) {
        return shooterSpeedMap.get(distanceMeters);
    }

    /**
     * Runs the shooter at a velocity supplied dynamically each loop.
     * Use this with a distance supplier for live interpolation.
     * @param velocitySupplier Supplies the target velocity in RPS each cycle.
     * @return A command that continuously sets velocity and stops when finished.
     */
    public Command setVelocityDynamicCommand(DoubleSupplier velocitySupplier) {
        return this.run(() -> setVelocity(velocitySupplier.getAsDouble())).finallyDo(() -> stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor1 RPS", getMotor1VelocityRPS());
        SmartDashboard.putNumber("Shooter Motor2 RPS", getMotor2VelocityRPS());
 
       }  
}