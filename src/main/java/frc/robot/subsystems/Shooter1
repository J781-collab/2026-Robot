package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorOneConstants;
import frc.robot.Constants.MotorTwoConstants;

public class ShooterSubsys extends SubsystemBase {

    private TalonFX motorOne;
    private TalonFX motorTwo;

    public ShooterSubsys() {
        
        motorOne = new TalonFX(0);
        motorOne.getConfigurator().apply(Robot.ctreConfigs.motorOneConfig);
        motorTwo = new TalonFX(11);
        motorTwo.getConfigurator().apply(Robot.ctreConfigs.motorTwoConfig);

    }

    public void setMotorSpeed(double speedSup) {

        motorOne.setVoltage(MotorOneConstants.maxVoltage * speedSup);
        motorTwo.setVoltage(MotorTwoConstants.maxVoltage * speedSup);

    }

    public double getMotorVelocityInRPS() {
        
        return motorOne.getVelocity().getValueAsDouble();

    }

    public double getMotorPosition() {

        return motorOne.getPosition().getValueAsDouble();

    }

    public void brakeMotor() {

        motorOne.setVoltage(0);
        motorTwo.setVoltage(0);

    }

    @Override
    public void periodic() {
    }
}