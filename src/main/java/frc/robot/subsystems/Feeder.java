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


public class Feeder extends SubsystemBase {
  private SparkMax feederMotor;
  private SparkMaxConfig feederMotorConfig;

  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor = new SparkMax(24, MotorType.kBrushless);
    
    configureDevices();
  }

  // Set current limits and config motor
  private void configureDevices() {
    try {
      feederMotorConfig = new SparkMaxConfig();
      feederMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

      feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to configure Feeder Subsystem", ex.getStackTrace());
    }
  }

  /**
   * Set the feeder motor speed with duty cycle control.
   * @param speed [-1, 1] speed to set the feeder to.
   */
  public void setSpeed(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Set the feeder speed as a command.
   * @param speed [-1, 1] speed to set the feeder to.
   * @return A command that sets the speed.
   */
  public Command setSpeedCommand(double speed) {
    return this.run(() -> setSpeed(speed)).finallyDo(() -> stop());
  }

  /**
   * Stop the feeder.
   */
  public void stop() {
    feederMotor.set(0);
  }
}