# Understanding Subsystems

If you are looking at the code in this folder, you are looking at the code that directly controls the physical parts of the robot!

## What is a Subsystem?

In the WPILib Command-Based framework, a **Subsystem** represents a major physical mechanism on the robot. Every file in this folder usually represents one specific mechanism.

The job of a Subsystem is to:
1.  **Initialize the Hardware:** Set up the motor controllers (like SparkMax or TalonFX), sensors, and encoders, using the correct ID numbers.
2.  **Provide Methods to Move:** Create functions (methods) like `setSpeed()`, `stop()`, or `moveToPosition()` that other parts of the code can call.
3.  **Report Data (Telemetry):** Send information (like how fast a motor is spinning or the current position of an arm) to the SmartDashboard so drivers and programmers can see it. This is usually done in the `periodic()` method, which runs continuously, roughly 50 times a second.

## The Subsystems in this Robot

Here is a breakdown of what each subsystem file in this folder does:

*   **`CommandSwerveDrivetrain.java`**: This is the most complex subsystem. It controls the swerve drive base, allowing the robot to drive in any direction and rotate at the same time. It also handles reading data from the cameras (Limelights) to figure out exactly where the robot is on the field using AprilTags.
*   **`Climber.java`**: Controls the motor that pulls the robot up onto the chain at the end of the match. It uses a PID controller to go to specific heights based on encoder readings.
*   **`IntakeRollers.java`**: Controls the spinning wheels that grab the game piece (Notes) from the floor. It includes a "smart intake" feature that detects if a note is jammed and automatically reverses to unjam it.
*   **`IntakePivot.java`**: Controls the arm/wrist that moves the intake rollers up and down. It uses a PID controller to aim the intake at specific angles and has a "smart agitate" feature to shake the arm if notes are stuck in a pile.
*   **`Elevator.java`**: Controls a mechanism that moves up and down (likely to lift notes to the shooter). It also has jam detection.
*   **`Conveyer.java`**: Controls the belts that move the game piece from the intake to the shooter. It has jam detection similar to the intake and elevator.
*   **`Feeder.java`**: A small wheel mechanism right before the shooter that pushes the game piece into the spinning shooter wheels when it's time to fire.
*   **`Shooter.java`**: Controls the two high-speed flywheels that launch the game piece. It has logic to automatically adjust its wheel speed based on how far away the robot is from the target (using the distance calculated by the Drivetrain/cameras).

## Reading a Subsystem File

When you open one of these files (like `Feeder.java`), you'll typically see a similar pattern:
1.  **Variables at the top:** Defining the motors and configuration objects.
2.  **The Constructor (e.g., `public Feeder()`):** This runs once when the robot turns on. It creates the motor objects using specific CAN IDs and sets their limits (like max current so they don't burn out).
3.  **Control Methods (e.g., `setSpeed(double speed)`):** The actual code that tells the motor to spin.
4.  **Command Factories (e.g., `setSpeedCommand(...)`):** These return a WPILib `Command` object that can be tied to a button on the controller.
5.  **`periodic()`:** A block of code that runs constantly, updating numbers on the SmartDashboard.