// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRollers;

public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double AngularRate = Math.PI * 2.5;
    // Set the default command to force the arm to go to 0.
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveAimAtTag = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<String>m_chooser = new SendableChooser<>();
    
   // private final CommandXboxController joystick = new CommandXboxController(0);
    public Joystick driverController = new Joystick(0);
    public Joystick opPanel = new Joystick(1);
    public Joystick testPanel = new Joystick(2);
 // Driver Controller //
    public final Trigger driverA = new Trigger(() -> driverController.getRawButton(1));
    public final Trigger driverB = new Trigger(() -> driverController.getRawButton(2));
    public final Trigger driverX = new Trigger(() -> driverController.getRawButton(3));
    public final Trigger driverY = new Trigger(() -> driverController.getRawButton(4));
    public final Trigger driverStart = new Trigger(() -> driverController.getRawButton(8));

    public final Trigger driverRT = new Trigger(() -> driverController.getRawAxis(3) > 0.5);
    public final Trigger driverLT = new Trigger(() -> driverController.getRawAxis(2) > 0.5);      
    public final Trigger driverLB = new Trigger(() -> driverController.getRawButton(5));

    public final Trigger driverRB = new Trigger(() -> driverController.getRawButton(6));
    public final Trigger driverPadUp = new Trigger(()-> driverController.getPOV(0) == 0);
    public final Trigger driverPadDown = new Trigger(()-> driverController.getPOV(0)==180);
   
 // Operator Panel //
    public final Trigger op1 = new Trigger(() -> opPanel.getRawButton(1));
    public final Trigger op2 = new Trigger(() -> opPanel.getRawButton(2));
    public final Trigger op3 = new Trigger(() -> opPanel.getRawButton(3));
    public final Trigger op4 = new Trigger(() -> opPanel.getRawButton(4));
    public final Trigger op5 = new Trigger(() -> opPanel.getRawButton(5));
    public final Trigger op6 = new Trigger(() -> opPanel.getRawButton(6));
    public final Trigger op7 = new Trigger(() -> opPanel.getRawButton(7));
    public final Trigger op8 = new Trigger(() -> opPanel.getRawButton(8));
    public final Trigger op9 = new Trigger(() -> opPanel.getRawButton(9));
    public final Trigger op10 = new Trigger(() ->opPanel.getRawButton(10));
    public final Trigger op11 = new Trigger(() ->opPanel.getRawButton(11));
    public final Trigger op12 = new Trigger(() ->opPanel.getRawButton(12));
    public final Trigger op13 = new Trigger(() ->opPanel.getRawButton(13));
    public final Trigger op14 = new Trigger(() ->opPanel.getRawButton(14));
    public final Trigger op15 = new Trigger(() ->opPanel.getRawButton(15));
    public final Trigger op16 = new Trigger(() ->opPanel.getRawButton(16));
    public final Trigger op17 = new Trigger(() ->opPanel.getRawButton(17));
    public final Trigger op18 = new Trigger(() ->opPanel.getRawButton(18));
    public final Trigger op19 = new Trigger(() ->opPanel.getRawButton(19));
    public final Trigger op20 = new Trigger(() ->opPanel.getRawButton(20));
    public final Trigger op21 = new Trigger(() ->opPanel.getRawButton(21));
    public final Trigger op22 = new Trigger(() ->opPanel.getRawButton(22));
    public final Trigger op23 = new Trigger(() ->opPanel.getRawButton(23));
    public final Trigger op24 = new Trigger(() ->opPanel.getRawButton(24));

    // Test panel // 
    public final Trigger test1 = new Trigger(() -> testPanel.getRawButton(1));
    public final Trigger test2 = new Trigger(() -> testPanel.getRawButton(2));
    public final Trigger test3 = new Trigger(() -> testPanel.getRawButton(3));
    public final Trigger test4 = new Trigger(() -> testPanel.getRawButton(4));
    public final Trigger test5 = new Trigger(() -> testPanel.getRawButton(5));
    public final Trigger test6 = new Trigger(() -> testPanel.getRawButton(6));
    public final Trigger test7 = new Trigger(() -> testPanel.getRawButton(7));
    public final Trigger test8 = new Trigger(() -> testPanel.getRawButton(8));
    public final Trigger test9 = new Trigger(() -> testPanel.getRawButton(9));
    public final Trigger test10 = new Trigger(() ->testPanel.getRawButton(10));
    public final Trigger test11 = new Trigger(() ->testPanel.getRawButton(11));
    public final Trigger test12 = new Trigger(() ->testPanel.getRawButton(12));
    public final Trigger test13 = new Trigger(() ->testPanel.getRawButton(13));
    public final Trigger test14 = new Trigger(() ->testPanel.getRawButton(14));
    public final Trigger test15 = new Trigger(() ->testPanel.getRawButton(15));
    public final Trigger test16 = new Trigger(() ->testPanel.getRawButton(16));
    public final Trigger test17 = new Trigger(() ->testPanel.getRawButton(17));
    public final Trigger test18 = new Trigger(() ->testPanel.getRawButton(18));
    public final Trigger test19 = new Trigger(() ->testPanel.getRawButton(19));
    public final Trigger test20 = new Trigger(() ->testPanel.getRawButton(20));
    public final Trigger test21 = new Trigger(() ->testPanel.getRawButton(21));
    public final Trigger test22 = new Trigger(() ->testPanel.getRawButton(22));
    public final Trigger test23 = new Trigger(() ->testPanel.getRawButton(23));
    public final Trigger test24 = new Trigger(() ->testPanel.getRawButton(24));

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakePivot pivot = new IntakePivot();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final Feeder feeder = new Feeder();
    public final Shooter shooter = new Shooter();
    public final Conveyer conveyer = new Conveyer();  
    public final IntakeRollers intakeRollers = new IntakeRollers();
    

    public RobotContainer() {
        configureBindings();
    /*public void configureNamedCommands(){
        NamedCommands.registerCommand(name:"",
            new ParallelDeadlineGroup(getAutonomousCommand(), null)
    }
        m_chooser.addOption(name:"driveforward+shoot", object:"driveforward+shoot");
        NamedCommands.registerCommand(name:"",new:"" );*/
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(      ((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0)) //Square joystick values for finer control with small inputs while still keeping full tilt = full speed
                        .withVelocityY(      ((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                        .withRotationalRate( ((driverController.getRawAxis(4)*driverController.getRawAxis(4)) * (driverController.getRawAxis(4)>0 ? -1 : 1)) * AngularRate)
                    )
        );
        // Schedule `setPivotGoal` + `pivotArm` when the Xbox controller's X button is pressed,
        // cancelling on release.
        // Hold X to drive while auto-aiming rotation toward the midpoint between two AprilTags
        // Red alliance: aim between tags 9 & 10, Blue alliance: aim between tags 25 & 26
        driverX.whileTrue(
            drivetrain.applyRequest(() -> {
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                int tag1 = isRed ? 9 : 25;
                int tag2 = isRed ? 10 : 26;
                return driveAimAtTag
                    .withVelocityX(((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                    .withVelocityY(((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                    .withTargetDirection(drivetrain.getRotationRelativeMidpoint(tag1, tag2));
            })
        );
        driverY.whileTrue(pivot.setPivotGoal(15).andThen(pivot.pivotArm()));
        // Schedule `set` when the Xbox controller's B button is pressed,
        // cancelling on release.
<<<<<<< Updated upstream
=======
        driverRT.whileTrue(pivot.setPivotGoal(30).andThen(pivot.pivotArm()));
>>>>>>> Stashed changes
        driverLT.whileTrue(conveyer.setSpeedCommand(0.5));
        // Hold LB to deploy intake (pivot to 90°) and run rollers; release returns to -45° and stops
        driverLB.onTrue(pivot.setPivotGoal(90).andThen(pivot.pivotArm()));
        driverLB.whileTrue(intakeRollers.setSpeedCommand(0.5));
        driverLB.onFalse(pivot.setPivotGoal(-45).andThen(pivot.pivotArm()));

        // Shooter — hold RT to spin up with velocity PID, release to stop
        driverRT.whileTrue(shooter.setVelocityCommand(50.0));
        // Shooter duty cycle on op panel button 1
        op1.whileTrue(shooter.setDutyCycleCommand(0.75));

        // Hold RB to shoot: spin up shooter with interpolated speed based on distance,
        // then run elevator + conveyer to feed balls into shooter
        driverRB.whileTrue(
            getShootCommand()
        );
        op11.whileTrue(intakeRollers.setSpeedCommand(1));
        op12.whileTrue(intakeRollers.setSpeedCommand(-1));
        op13.whileTrue(conveyer.setSpeedCommand(1));
        op14.whileTrue(conveyer.setSpeedCommand(-1));
        op15.whileTrue(elevator.setSpeedCommand(1));
        op16.whileTrue(elevator.setSpeedCommand(-1));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverA.whileTrue(drivetrain.applyRequest(() -> brake));
        driverB.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(driverController.getRawAxis(1), driverController.getRawAxis(0)))
        ));
    
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       
        
        test1.and(test2).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        test3.and(test4).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        test5.and(test6).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        test7.and(test8).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Shooter SysId routines on test panel
        test9.whileTrue(shooter.sysIdDynamic(Direction.kForward));
        test10.whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        test11.whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        test12.whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on start button press.
        driverStart.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    /**
     * Gets the distance to the alliance-specific shooting target.
     * Red: midpoint of tags 9 & 10, Blue: midpoint of tags 25 & 26.
     */
    private double getDistanceToTarget() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return drivetrain.getDistanceToMidpoint(isRed ? 9 : 25, isRed ? 10 : 26);
    }

    /**
     * Creates a command that:
     * 1. Spins up the shooter to an interpolated speed based on distance
     * 2. Waits for the shooter to reach target velocity
     * 3. Then runs the elevator and conveyer to feed balls into the shooter
     * 
     * Everything stops when the button is released.
     */
    public Command getShootCommand() {
        return Commands.parallel(
            // Shooter spins up and stays running the whole time
            shooter.setVelocityDynamicCommand(() -> Shooter.getInterpolatedSpeed(getDistanceToTarget())),
            // Wait for shooter to reach speed, then feed
            Commands.sequence(
                Commands.waitUntil(() -> shooter.atTargetVelocity(
                    Shooter.getInterpolatedSpeed(getDistanceToTarget()), 3.0)),
                Commands.parallel(
                    elevator.setSpeedCommand(0.5),
                    conveyer.setSpeedCommand(0.5)
                )
            )
        );
    }
}
