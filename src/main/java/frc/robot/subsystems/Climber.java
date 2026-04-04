
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

public class Climber extends SubsystemBase {
    private SparkMax climbmotor1;
    private SparkMaxConfig motorConfig;
    
    // Encoder for closed-loop control
    private RelativeEncoder encoder1;
    
    // PID Controller
    private PIDController pidController1;
    
    // PID Gains
    private static final double kP = 0.1;
    private static final double kI = 0.01;
    private static final double kD = 0.05;
    
    private double targetHeight = 0.0;
    private boolean usePID = false;
    
    public Climber() {
        climbmotor1 = new SparkMax(51, MotorType.kBrushless);
        
        // Get encoder
        encoder1 = climbmotor1.getEncoder();
        
        // Initialize PID controller
        pidController1 = new PIDController(kP, kI, kD);
        
        // Set tolerance
        pidController1.setTolerance(0.5, 0.5);
        
        configureDevices();
    }
    
    private void configureDevices() {
        motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // 35:1 gear reduction, 0.75" diameter pulley
        // Position: inches of linear travel per motor rotation = (π × 0.75) / 35
        // Velocity: inches per second
        double inchesPerMotorRot = Math.PI * 0.75 / 35.0;
        motorConfig.encoder
            .positionConversionFactor(inchesPerMotorRot)
            .velocityConversionFactor(inchesPerMotorRot / 60.0);

        climbmotor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Set target height for PID control.
     * @param height Target height in rotations
     */
    public void setTargetHeight(double height) {
        targetHeight = height;
        usePID = true;
    }
    
    /**
     * Gets current height.
     * @return Current height in rotations
     */
    public double getCurrentHeight() {
        return encoder1.getPosition();
    }
    
    /**
     * Check if at target height.
     * @return true if motor is at target
     */
    public boolean atTarget() {
        return pidController1.atSetpoint();
    }
    
    /**
     * Reset encoder to zero.
     */
    public void resetEncoders() {
        encoder1.setPosition(0);
        targetHeight = 0;
    }
    
    /**
     * Update PID gains.
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public void setPIDGains(double p, double i, double d) {
        pidController1.setPID(p, i, d);
    }
    
    @Override
    public void periodic() {
        if (usePID) {
            double output1 = pidController1.calculate(encoder1.getPosition(), targetHeight);
            output1 = Math.max(-1.0, Math.min(1.0, output1));
            climbmotor1.set(output1);
            
            SmartDashboard.putNumber("Climber/Target Height", targetHeight);
            SmartDashboard.putNumber("Climber/Motor Height", encoder1.getPosition());
            SmartDashboard.putBoolean("Climber/At Target", atTarget());
            SmartDashboard.putNumber("Climber/Motor Output", output1);
        }
    }

    public void setSpeed(double speed) {
        usePID = false;
        climbmotor1.set(speed);
    }

    /**
     * Set the motor speed as a command (manual duty cycle control).
     * @param speed [-1, 1] speed to set the motor to.
     * @return A command that sets the speed.
     */
    public Command setSpeedCommand(double speed) {
        return this.run(() -> setSpeed(speed)).finallyDo(() -> stop());
    }
    
    /**
     * Move to target position using PID control.
     * Continuously runs the PID loop; stops the motor when interrupted.
     *
     * @param position Target position in rotations
     * @return A command that holds the target position
     */
    public Command moveToPositionCommand(double position) {
        return this.run(() -> {
            double output = pidController1.calculate(encoder1.getPosition(), position);
            output = Math.max(-1.0, Math.min(1.0, output));
            climbmotor1.set(output);
        }).finallyDo(() -> {
            climbmotor1.set(0);
        });
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        usePID = false;
        climbmotor1.set(0);
    }

}