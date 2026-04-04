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

public class IntakeRollers extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeMotorConfig;
  private RelativeEncoder encoder;

  // 3:1 gear reduction — encoder is motor-side, divide by ratio to get roller RPM
  private static final double GEAR_RATIO = 3.0;

  // Jam detection thresholds (in roller-side RPM)
  private static final double JAM_VELOCITY_THRESHOLD = 100.0; // RPM — below this while commanding full speed = jammed
  private static final double UNJAM_VELOCITY_THRESHOLD = 200.0; // RPM — above this while reversing = unjammed
  private static final double STARTUP_GRACE_SECONDS = 0.25; // Ignore jam detection for this long after starting

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    intakeMotor = new SparkMax(25, MotorType.kBrushless);
    encoder = intakeMotor.getEncoder();
    
    configureDevices();
  }

  // Set current limits and config motor
  private void configureDevices() {
    try {
      intakeMotorConfig = new SparkMaxConfig();
      intakeMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60);

      intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure Intake Rollers Subsystem", ex.getStackTrace());
    }
  }

  /** Get roller velocity in RPM (output / roller side, accounting for 3:1 gear reduction). */
  public double getVelocity() {
    return encoder.getVelocity() / GEAR_RATIO;
  }

  /** Get roller motor current draw in amps. */
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  /**
   * Set the roller motor speed with duty cycle control.
   * @param speed [-1, 1] speed to set the roller to.
   */
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
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
   * Smart intake command with jam detection.
   * Runs the rollers at the given speed. If a jam is detected (velocity drops
   * below threshold while commanding full speed), reverses the rollers until
   * they spin freely again, then resumes intaking. Repeats as needed.
   *
   * @param speed The normal intake speed (e.g. -1 for full intake)
   * @return A command that handles jam detection and reversal
   */
  public Command smartIntakeCommand(double speed) {
    return Commands.sequence(
      // Phase 1: Run intake normally, end when jam detected
      // Grace period before checking for jams (motor needs time to spin up)
      Commands.runOnce(() -> setSpeed(speed), this),
      Commands.waitSeconds(STARTUP_GRACE_SECONDS),
      this.run(() -> setSpeed(speed))
          .until(() -> Math.abs(getVelocity()) < JAM_VELOCITY_THRESHOLD),
      // Phase 2: Reverse until unjammed
      this.run(() -> setSpeed(-speed))
          .until(() -> Math.abs(getVelocity()) > UNJAM_VELOCITY_THRESHOLD)
    ).repeatedly() // Rinse and repeat
     .finallyDo(() -> stop());
  }

  /**
   * Stop the roller.
   */
  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRollers/Velocity RPM", getVelocity());
    SmartDashboard.putNumber("IntakeRollers/Current Amps", getCurrent());
  }
}