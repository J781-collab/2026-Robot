// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShootingConstants;
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
    // Configure the heading PID for auto-aim
    {
        driveAimAtTag.HeadingController.setPID(7, 0, 0);
        driveAimAtTag.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private SendableChooser<Command> autoSelector;
    
    // Adjustable shooter duty cycle (op17 = increase, op19 = decrease by 5%)
    private double shooterDutyCycle = 0.0;
    // Adjustable shooter velocity PID (op18 = increase, op20 = decrease by 5 RPS)
    private double shooterTargetRPS = 0.0;
    
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

    public final CommandSwerveDrivetrain drivetrain;
    public final IntakePivot pivot = new IntakePivot();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final Feeder feeder = new Feeder();
    public final Shooter shooter = new Shooter();
    public final Conveyer conveyer = new Conveyer();  
    public final IntakeRollers intakeRollers = new IntakeRollers();
    

    public RobotContainer() {
        // Register named commands BEFORE creating drivetrain (which calls AutoBuilder.configure)
      /*/  NamedCommands.registerCommand("lowerIntake",
           // pivot.setPivotGoal(IntakePivotConstants.intakePosition).andThen(pivot.pivotArm()));
        NamedCommands.registerCommand("raiseIntake",
           // pivot.setPivotGoal(IntakePivotConstants.idlePosition).andThen(pivot.pivotArm()));
        NamedCommands.registerCommand("intakeRollers", intakeRollers.setSpeedCommand(1.0));*/
        NamedCommands.registerCommand("shootSequence", 
            Commands.parallel(
                // Shooter runs the entire time
                shooter.setDutyCycleCommand(1),
                // Wait for shooter to reach speed, then feed with conveyor + elevator
                Commands.sequence(
                    Commands.waitUntil(() -> shooter.atTargetVelocity(50, 2)),
                    Commands.parallel(
                        conveyer.setSpeedCommand(1.0),
                        elevator.setSpeedCommand(1.0)
                    )
                    
                ),
                Commands.sequence(Commands.waitSeconds(10),
                pivot.pivotArm(250))
            )
        );
        NamedCommands.registerCommand("conveyer", conveyer.setSpeedCommand(1.0));
        NamedCommands.registerCommand("elevator", elevator.setSpeedCommand(1.0));
        NamedCommands.registerCommand("climb", climber.setSpeedCommand(1.0));
        // Lower intake pivot to 10 and run intake rollers at full speed
        NamedCommands.registerCommand("intake",
            Commands.parallel(
                pivot.pivotArm(10),
                intakeRollers.setSpeedCommand(-1)
            )
        );

        // Create drivetrain AFTER named commands are registered
        drivetrain = TunerConstants.createDrivetrain();

        // Named command that requires drivetrain — registered after drivetrain is created
        NamedCommands.registerCommand("aimAndShoot",
            Commands.parallel(
                // Auto-aim at the hub (Red: tag 10, Blue: tag 25 — centered tags)
                drivetrain.applyRequest(() -> {
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    int tag = isRed ? 10 : 25;
                    Rotation2d aimAngle = ShootingConstants.ENABLE_AIM_COMPENSATION
                        ? drivetrain.getAimCompensatedRotation(tag, tag)
                        : drivetrain.getRotationRelativeMidpoint(tag, tag);
                    return driveAimAtTag
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withTargetDirection(aimAngle);
                }),
                // Spin up shooter at interpolated speed based on Limelight distance
                shooter.setVelocityDynamicCommand(
                    () -> Shooter.getInterpolatedSpeed(drivetrain.getLimelightAprilTagDistance())),
                // Wait for speed, pause 1s for aim to settle, then feed
                Commands.sequence(
                    Commands.waitUntil(() -> shooter.atTargetVelocity(
                        Shooter.getInterpolatedSpeed(drivetrain.getLimelightAprilTagDistance()), 10.0)),
                    Commands.waitSeconds(2.0),
                    Commands.parallel(
                        conveyer.setSpeedCommand(1.0),
                        elevator.setSpeedCommand(1.0)
                    )
                )
            ).withTimeout(10)
        );

        // Build PathPlanner auto chooser and publish to SmartDashboard
        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selector", autoSelector);
        
        configureBindings();
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(      ( (driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0)) //Square joystick values for finer control with small inputs while still keeping full tilt = full speed
                        .withVelocityY(      ( (driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                        .withRotationalRate( ( (driverController.getRawAxis(4)*driverController.getRawAxis(4)) * (driverController.getRawAxis(4)>0 ? -1 : 1)) * AngularRate)
                    )
        );
        // Schedule `setPivotGoal` + `pivotArm` when the Xbox controller's X button is pressed,
        // cancelling on release.
        // Hold X to drive while auto-aiming rotation toward the centered AprilTag
        // Red alliance: tag 10, Blue alliance: tag 25
        // Uses aim compensation if enabled, otherwise plain aim
        driverX.whileTrue(
            drivetrain.applyRequest(() -> {
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                int tag = isRed ? 10 : 25;
                Rotation2d aimAngle = ShootingConstants.ENABLE_AIM_COMPENSATION
                    ? drivetrain.getAimCompensatedRotation(tag, tag)
                    : drivetrain.getRotationRelativeMidpoint(tag, tag);
                return driveAimAtTag
                    .withVelocityX(((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                    .withVelocityY(((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
                    .withTargetDirection(aimAngle);
            })
        );
        // OLD driverX (no aim compensation):
        // driverX.whileTrue(
        //     drivetrain.applyRequest(() -> {
        //         boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        //         int tag1 = isRed ? 9 : 25;
        //         int tag2 = isRed ? 10 : 26;
        //         return driveAimAtTag
        //             .withVelocityX(((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
        //             .withVelocityY(((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0))
        //             .withTargetDirection(drivetrain.getRotationRelativeMidpoint(tag1, tag2));
        //     })
        // );
      //  driverY.whileTrue(pivot.setPivotGoal(15).andThen(pivot.pivotArm()));
        // Schedule `set` when the Xbox controller's B button is pressed,
        // cancelling on release.
        driverRT.whileTrue(conveyer.setSpeedCommand(1));
        driverRT.whileTrue(elevator.setSpeedCommand(1));
        // Hold LB to deploy intake (pivot to 90°) and run rollers; release returns to -45° and stops
        driverLB.onTrue(pivot.pivotArm(10));
        driverLB.whileTrue(intakeRollers.setSpeedCommand(-1));
      //  driverLB.onFalse(/*pivot.setPivotGoal(0).andthen*/pivot.pivotArm(250));

        // Shooter — hold RT to spin up with velocity PID, release to stop
    //    driverRT.whileTrue(shooter.setVelocityCommand(
     //       -50.0));
        // Hold RB to auto-aim at AprilTags AND spin shooter at interpolated speed
        // Hold driverRB to shake/agitate (rotation oscillation) while aiming at goal and shooting
        driverRB.whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(() -> {
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    int tag = isRed ? 10 : 25;
                    Rotation2d aimAngle = ShootingConstants.ENABLE_AIM_COMPENSATION
                        ? drivetrain.getAimCompensatedRotation(tag, tag)
                        : drivetrain.getRotationRelativeMidpoint(tag, tag);
                    // Joystick driving
                    double driveX = ((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0);
                    double driveY = ((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * (driverLB.getAsBoolean() ? 0.3 : 1.0);
                    // Agitate: oscillate rotation ±7° around the aim angle (~12 Hz)
                    double shakeRadians = Math.toRadians(7.0) * Math.sin(Timer.getFPGATimestamp() * 12.0 * 2.0 * Math.PI);
                    Rotation2d agitatedAngle = aimAngle.rotateBy(Rotation2d.fromRadians(shakeRadians));
                    return driveAimAtTag
                        .withVelocityX(driveX)
                        .withVelocityY(driveY)
                        .withTargetDirection(agitatedAngle);
                }),
                shooter.setVelocityDynamicCommand(
                    () -> Shooter.getInterpolatedSpeed(drivetrain.getLimelightAprilTagDistance()))
            )
        );
        // Shooter duty cycle oon op panel button 1
      //  op1.whileTrue(shooter.setDutyCycleCommand(0.75));
        // Hold op2 to shake robot (destuck balls) while aiming at goal and shooting
        op2.whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(() -> {
                    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                    int tag = isRed ? 10 : 25;
                    Rotation2d aimAngle = ShootingConstants.ENABLE_AIM_COMPENSATION
                        ? drivetrain.getAimCompensatedRotation(tag, tag)
                        : drivetrain.getRotationRelativeMidpoint(tag, tag);
                    // Gentle shake: oscillate X velocity with a sine wave (~6 Hz, 0.5 m/s amplitude)
                    double shake = 0.5 * Math.sin(Timer.getFPGATimestamp() * 6.0 * 2.0 * Math.PI);
                    return driveAimAtTag
                        .withVelocityX(shake)
                        .withVelocityY(0)
                        .withTargetDirection(aimAngle);
                }),
                shooter.setVelocityDynamicCommand(
                    () -> Shooter.getInterpolatedSpeed(drivetrain.getLimelightAprilTagDistance()))
            )
        );
        op3.whileTrue(intakeRollers.setSpeedCommand(1));
       op4.whileTrue(shooter.setVelocityDynamicCommand(
            () -> Shooter.getInterpolatedSpeed(drivetrain.getLimelightAprilTagDistance())));
        // Hold RB to shoot: auto-aim + spin up shooter + feed when ready
        // Option 1 (fallback): limits drive speed while shooting
        // Option 2: uses aim compensation for accurate shots while driving
  ///      driverRB.whileTrue(
    //        getShootCommand()
     //   ); 
        //op5.whileTrue(shooter.setVelocityCommand(5));    
        op5.whileTrue(intakeRollers.setSpeedCommand(1));
        op5.whileTrue(conveyer.setSpeedCommand(-1));
        op5.whileTrue(elevator.setSpeedCommand(-1));
        op6.whileTrue(pivot.pivotArm(10));
        op1.whileTrue(pivot.pivotArm(-300));
        // Hold op8 to oscillate intake pivot between 10 and -250 (agitate/destuck)
        op8.whileTrue(
            Commands.repeatingSequence(
                pivot.pivotArm(-50).withTimeout(0.3),
                pivot.pivotArm(-350).withTimeout(0.3)
            )
        );   
       // op6.onTrue(pivot.setPivotGoal(-450).andThen(pivot.pivotArm()));
        op11.whileTrue(intakeRollers.setSpeedCommand(-1));
        op12.whileTrue(intakeRollers.setSpeedCommand(-1));
        op13.whileTrue(conveyer.setSpeedCommand(1));
        op14.whileTrue(conveyer.setSpeedCommand(-1));
        op15.whileTrue(elevator.setSpeedCommand(1));
        op16.whileTrue(elevator.setSpeedCommand(-1));

        // Adjustable shooter duty cycle: op17 = +5% and run, op19 = -5% and run
        op17.onTrue(Commands.runOnce(() -> {
            shooterDutyCycle = MathUtil.clamp(shooterDutyCycle +0.1, -1.0, 1.0);
            SmartDashboard.putNumber("Shooter Target Duty Cycle", shooterDutyCycle);
            shooter.setDutyCycle(shooterDutyCycle);
        }, shooter));
        op19.onTrue(Commands.runOnce(() -> {
            shooterDutyCycle = MathUtil.clamp(shooterDutyCycle - 0.1, -1.0, 1.0);
            SmartDashboard.putNumber("Shooter Target Duty Cycle", shooterDutyCycle);
            shooter.setDutyCycle(shooterDutyCycle);
        }, shooter));
        // Adjustable shooter velocity PID: op18= +5 RPS and run, op20 = -5 RPS and run
        op18.onTrue(Commands.runOnce(() -> {
            shooterTargetRPS = MathUtil.clamp(shooterTargetRPS + 5.0, -100, 100.0);
            SmartDashboard.putNumber("Shooter Target RPS", shooterTargetRPS);
            shooter.setVelocityCommand(shooterTargetRPS);
        }, shooter));
        op20.onTrue(Commands.runOnce(() -> {
            shooterTargetRPS = MathUtil.clamp(shooterTargetRPS - 5.0, -100, 100.0);
            SmartDashboard.putNumber("Shooter Target RPS", shooterTargetRPS);
            shooter.setVelocityCommand(shooterTargetRPS);
        }, shooter));
        // op21 = stop shooter and reset both duty cycle and velocity targets
        op21.onTrue(Commands.runOnce(() -> {
            shooterDutyCycle = 0.0;
            shooterTargetRPS = 0.0;
            SmartDashboard.putNumber("Shooter Target Duty Cycle", shooterDutyCycle);
            SmartDashboard.putNumber("Shooter Target RPS", shooterTargetRPS);
            shooter.stop();
        }, shooter));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // Set intake pivot to coast when disabled, brake when enabled
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> pivot.setCoastMode()).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onFalse(
            Commands.runOnce(() -> pivot.setBrakeMode())
        );

        driverA.whileTrue(drivetrain.applyRequest(() -> brake));
        driverB.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(driverController.getRawAxis(1), driverController.getRawAxis(0)))
        ));
    
        // SignalLogger stop for SysId — press after tests are done
        test12.onTrue(Commands.runOnce(SignalLogger::stop));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       
        
        test1.and(test2).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // test3.and(test4).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      //  test5.and(test6).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        test7.and(test8).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Shooter SysId routines on test panel
        test4.whileTrue(shooter.sysIdDynamic(Direction.kForward));
        test5.whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        test9.whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        test10.whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on start button press.
        driverStart.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    /**
     * Gets the distance to the alliance-specific shooting target.
     * Red: tag 10, Blue: tag 25 (centered tags).
     */
    private double getDistanceToTarget() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        int tag = isRed ? 10 : 25;
        return drivetrain.getDistanceToMidpoint(tag, tag);
    }

    /**
     * Creates a command that:
     * 1. Auto-aims at the alliance target (with velocity compensation if enabled)
     * 2. Spins up the shooter to an interpolated speed based on distance
     * 3. Waits for the shooter to reach target velocity
     * 4. Then runs the elevator and conveyer to feed balls into the shooter
     * 
     * If ENABLE_AIM_COMPENSATION is false (option 1 fallback):
     *   - Drive speed is limited by SHOOT_SPEED_MULTIPLIER while shooting
     *   - Aim uses plain rotation toward target
     * If ENABLE_AIM_COMPENSATION is true (option 2):
     *   - Full drive speed allowed
     *   - Aim leads the target based on robot velocity
     * 
     * Everything stops when the button is released.
     */

    public Command getShootCommand() {
        return Commands.parallel(
            // Auto-aim while shooting — uses drivetrain subsystem
            drivetrain.applyRequest(() -> {
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                int tag = isRed ? 10 : 25;

                Rotation2d aimAngle = ShootingConstants.ENABLE_AIM_COMPENSATION
                    ? drivetrain.getAimCompensatedRotation(tag, tag)
                    : drivetrain.getRotationRelativeMidpoint(tag, tag);

                // Option 1 fallback: limit speed. Option 2: full speed.
                double speedScale = ShootingConstants.ENABLE_AIM_COMPENSATION
                    ? 1.0
                    : ShootingConstants.SHOOT_SPEED_MULTIPLIER;

                return driveAimAtTag
                    .withVelocityX(((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed * speedScale)
                    .withVelocityY(((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed * speedScale)
                    .withTargetDirection(aimAngle);
            }),
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

    // OLD getShootCommand (no auto-aim, no speed limiting):
    // public Command getShootCommand() {
    //     return Commands.parallel(
    //         // Shooter spins up and stays running the whole time
    //         shooter.setVelocityDynamicCommand(() -> Shooter.getInterpolatedSpeed(getDistanceToTarget())),
    //         // Wait for shooter to reach speed, then feed
    //         Commands.sequence(
    //             Commands.waitUntil(() -> shooter.atTargetVelocity(
    //                 Shooter.getInterpolatedSpeed(getDistanceToTarget()), 3.0)),
    //             Commands.parallel(
    //                 elevator.setSpeedCommand(0.5),
    //                 conveyer.setSpeedCommand(0.5)
    //             )
    //         )
    //     );
    // }
}
