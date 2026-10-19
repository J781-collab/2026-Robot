# FRC Robot Code: A Beginner's Guide

Welcome to the robot code! If you are new to programming or new to FIRST Robotics Competition (FRC), this guide will help you understand how this project is organized and how the code makes the robot move.

This project uses **Java** and a library called **WPILib** (specifically the Command-Based Framework). Don't worry if it looks overwhelming at first; it's broken down into logical pieces.

## The Big Picture: Command-Based Programming

Think of the robot like a crew on a ship.
1.  **Subsystems (The Hardware):** These are the physical parts of the robot, like the drive base (wheels), the shooter, or the elevator. In the code, a subsystem manages the motors and sensors for that specific part. It knows *how* to move, but doesn't decide *when* to move.
2.  **Commands (The Actions):** A command is a specific action you want the robot to perform, like "Shoot a ball," "Drive forward for 2 seconds," or "Raise the elevator." Commands tell the subsystems what to do.
3.  **RobotContainer (The Captain):** This is where you tie everything together. You create the subsystems, create the controllers (joysticks/Xbox controllers), and say, "When the driver presses the 'A' button, run the 'Shoot' command."

## Where is everything?

The most important code lives inside `src/main/java/frc/robot/`.

*   `Robot.java`: The main loop of the robot. It handles switching between different modes (Autonomous, Teleop/Driver-controlled, Disabled). You usually don't need to change much here.
*   `RobotContainer.java`: The central hub. This is where controllers are defined and buttons are mapped to commands. (See the README in this folder for more details).
*   `Constants.java`: A place to store all the "magic numbers" (like motor IDs on the CAN bus, speeds, and dimensions) so they are in one easy-to-find spot.
*   `subsystems/`: A folder containing all the code for the physical parts of the robot. (See the README in this folder for more details).
*   `commands/`: A folder where complex, standalone actions would go (though often, simple commands are written directly in `RobotContainer`).

## Glossary of Common Terms

*   **Teleop:** Short for Teleoperated. This is the part of the match where drivers control the robot using controllers.
*   **Autonomous (Auto):** The first 15 seconds of the match where the robot must move purely based on pre-programmed instructions.
*   **SmartDashboard / NetworkTables:** A way for the robot to send data (like motor speeds, sensor readings, or camera feeds) back to the laptop so you can see what's happening.
*   **CAN Bus:** The communication wire that connects all the smart motor controllers (like SparkMax or TalonFX) to the main robot brain (the roboRIO). Every device on the CAN bus has a unique ID number.
*   **PID Controller:** A math equation used in the code to make a mechanism reach a target smoothly and accurately (e.g., aiming an arm exactly at 45 degrees without overshooting).
*   **SparkMax / TalonFX:** Brands of motor controllers. They are the "middlemen" that take signals from the roboRIO and send power to the actual motors.

## How to Build and Deploy

This project uses a tool called **GradleRIO** to turn your Java code into a program the robot can understand.

*   **Build:** In VS Code, click the WPILib icon (the little 'W' in the top right corner) and select "Build Robot Code". This checks for errors.
*   **Deploy:** Connect your laptop to the robot's radio (via Wi-Fi or USB tether). Click the WPILib icon and select "Deploy Robot Code". This sends the compiled code to the robot.
