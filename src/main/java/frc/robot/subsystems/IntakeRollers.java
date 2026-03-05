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

public class IntakeRollers extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeMotorConfig;

  /** Creates a new IntakeRolers. */
  public IntakeRollers() {
    intakeMotor = new SparkMax(25, MotorType.kBrushless);
    
    configureDevices();
  }

  // Set current limits and config motor
  private void configureDevices() {
    try {
      intakeMotorConfig = new SparkMaxConfig();
      intakeMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

      intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure Intake Rollers Subsystem", ex.getStackTrace());
    }
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
   * Stop the roller.
   */
  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

