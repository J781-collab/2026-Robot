package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.AlignmentController.RotationController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

/**
 * Aim and Drive Command - Drives with joystick while automatically aiming at a specified AprilTag.
 * The robot will point at the target while driving or standing still using PID control for smooth rotation.
 */
public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final CommandSwerveDrivetrain swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final int targetAprilTagID;
    private final double maxSpeed;
    private final double maxAngularRate;

    // PID controller for rotation to target
    private final PIDController rotationController;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Creates an AimAndDriveCommand that aims at a specified AprilTag while driving with joystick input.
     *
     * @param swerve The swerve drivetrain subsystem
     * @param forwardInput Supplier for forward/back input (positive forward)
     * @param leftInput Supplier for left/right input (positive left)
     * @param targetAprilTagID The ID of the AprilTag to aim at
     */
    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        int targetAprilTagID
    ) {
        this.swerve = swerve;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.targetAprilTagID = targetAprilTagID;
        this.maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        this.maxAngularRate = RadiansPerSecond.of(Math.PI * 2).in(RadiansPerSecond); // 2 rotations per second max
        
        // Initialize rotation PID controller with values from VisionConstants
        this.rotationController = new PIDController(
            RotationController.P,
            RotationController.I,
            RotationController.D
        );
        // Enable continuous input for angles to handle wraparound (359° to 0°)
        this.rotationController.enableContinuousInput(-180, 180);
        
        addRequirements(swerve);
    }

    /**
     * Alternate constructor with custom rotation PID gains.
     *
     * @param swerve The swerve drivetrain subsystem
     * @param forwardInput Supplier for forward/back input
     * @param leftInput Supplier for left/right input
     * @param targetAprilTagID The ID of the AprilTag to aim at
     * @param rotationP Proportional gain for rotation
     * @param rotationI Integral gain for rotation
     * @param rotationD Derivative gain for rotation
     */
    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        int targetAprilTagID,
        double rotationP,
        double rotationI,
        double rotationD
    ) {
        this(swerve, forwardInput, leftInput, targetAprilTagID);
        this.rotationController.setPID(rotationP, rotationI, rotationD);
    }

    /**
     * Checks if the robot is aimed at the target within tolerance.
     *
     * @return true if robot is aimed at target, false otherwise
     */
    public boolean isAimed() {
        try {
            final Rotation2d targetDirection = getDirectionToTarget();
            final Rotation2d currentHeading = swerve.getPose().getRotation();
            final double angleDifference = Math.abs(currentHeading.minus(targetDirection).getDegrees());
            return angleDifference < kAimTolerance.in(Degrees);
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Calculates the direction from the robot to the target AprilTag.
     *
     * @return Rotation2d pointing toward the target AprilTag
     */
    private Rotation2d getDirectionToTarget() {
        final Translation2d targetPosition = swerve.getTagPose(targetAprilTagID).getTranslation();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        return targetPosition.minus(robotPosition).getAngle();
    }

    /**
     * Applies squared sensitivity to joystick input for finer control at low speeds.
     *
     * @param input The raw joystick input
     * @return The squared input with sign preserved
     */
    private double applySquaredSensitivity(double input) {
        return Math.copySign(input * input, input);
    }

    @Override
    public void execute() {
        try {
            // Get target direction
            final Rotation2d targetDirection = getDirectionToTarget();
            final double targetAngleDegrees = targetDirection.getDegrees();
            final double currentAngleDegrees = swerve.getPose().getRotation().getDegrees();

            // Calculate rotation output using PID
            final double rotationOutput = rotationController.calculate(currentAngleDegrees, targetAngleDegrees);
            
            // Clamp rotation output to max angular rate
            final double clampedRotation = Math.max(-maxAngularRate, Math.min(maxAngularRate, rotationOutput));

            // Get joystick inputs and apply squared sensitivity for finer control
            final double forward = applySquaredSensitivity(forwardInput.getAsDouble());
            final double left = applySquaredSensitivity(leftInput.getAsDouble());

            // Telemetry
            double[] aimOutputs = {targetAngleDegrees, currentAngleDegrees, rotationOutput};
            SmartDashboard.putNumberArray("Aim PID Outputs", aimOutputs);
            SmartDashboard.putBoolean("Is Aimed", isAimed());

            // Set control with field-centric drive and PID-controlled rotation
            swerve.setControl(
                driveRequest
                    .withVelocityX(forward * maxSpeed)
                    .withVelocityY(left * maxSpeed)
                    .withRotationalRate(clampedRotation)
            );
        } catch (Exception e) {
            // If target is not found, just drive without aiming
            final double forward = applySquaredSensitivity(forwardInput.getAsDouble());
            final double left = applySquaredSensitivity(leftInput.getAsDouble());
            
            swerve.setControl(
                driveRequest
                    .withVelocityX(forward * maxSpeed)
                    .withVelocityY(left * maxSpeed)
                    .withRotationalRate(0)
            );
            SmartDashboard.putString("Aim Status", "Target Not Found");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
