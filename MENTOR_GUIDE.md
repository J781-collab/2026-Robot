# FRC Programming Mentor Guide (For Web Devs)

Welcome to FRC mentoring! It’s a blast, but the environment is definitely different from standard web development. Since your goal is to guide students without writing the code for them, here is a "cheat sheet" to help you translate your web-dev debugging skills into FRC terms.

## 1. The Build/Deploy Loop (The "Refresh" Button)
In web dev, you hit save and refresh the browser. In FRC:
*   The student hits save in VS Code.
*   They must run **Build Robot Code** (checks for Java compile errors).
*   They must run **Deploy Robot Code** (sends the compiled `.jar` over the network to the roboRIO).
*   *Mentor Tip:* If a student says "My code change didn't work," always ask: "Did you successfully build and deploy?" Often, a build fails silently in the background, or the laptop disconnects from the robot Wi-Fi.

## 2. The Driver Station (The "Console/Terminal")
The FRC Driver Station software (running on the Windows laptop connected to the controllers) is your best friend.
*   **The Console Window:** Click the gear icon on the Driver Station to open the console. This is the equivalent of your browser's DevTools console or Node.js terminal.
*   **Reading Stack Traces:** If the robot code crashes, the red stack trace will appear here. Teach the student to scroll up to the *top* of the red text to find the actual line of code in their files (`frc.robot...`) that caused the NullPointerException, rather than staring at the deep WPILib internal errors.
*   *Mentor Tip:* "Let's look at the Driver Station log. What is the first line in the red text that mentions our project files?"

## 3. SmartDashboard / NetworkTables (The "Inspector / Redux DevTools")
NetworkTables is a real-time key-value store shared between the robot and the laptop. The SmartDashboard (or Shuffleboard / AdvantageScope) is the UI to view it.
*   If a motor isn't moving, you don't just guess. You look at the telemetry.
*   *Mentor Tip:* If the Intake Pivot isn't going to the right angle, ask the student: "Are we sending the encoder position and the PID target to SmartDashboard? Let's graph them and see if the motor is actually trying to move."
*   *Debugging Flow:* Code -> SmartDashboard -> Motor Controller LEDs (they blink green for forward, red for reverse) -> Actual Motor. Follow the data pipeline.

## 4. The Command-Based Architecture (Event-Driven / React-ish)
The WPILib Command framework is heavily event-driven.
*   **Subsystems** are like your state/stores. They hold the hardware instances and current sensor values.
*   **Commands** are like actions/reducers. They encapsulate a specific behavior over time.
*   **RobotContainer** is like your main Router or App component, binding user input (DOM events) to Commands.
*   *Mentor Tip:* If a button press does the wrong thing, guide them to `RobotContainer.java`. Find the button mapping. Ask, "What command is this button running? Let's go look inside that command or subsystem method to see what logic it executes."

## 5. Dealing with Hardware (The "Backend" you can't control)
Unlike web dev, sometimes the code is perfect but the robot still fails.
*   *Is it plugged in?* Literally. Check CAN bus wires, power wires, and ethernet cables.
*   *Are the IDs right?* Every motor controller (SparkMax/TalonFX) has a CAN ID (a number 1-63). If the code says `new SparkMax(25)` but the physical motor controller is ID 26, it will throw a timeout error in the console and not move.
*   *Is there physical friction?* "The PID isn't reaching the target, let's turn the robot off and move the arm by hand. Is it physically jammed?"

## How to ask guiding questions:
When the rookie coder gets stuck, avoid the urge to take the keyboard. Use these prompts:
*   "What behavior were you expecting, and what actually happened?"
*   "Where in the code do you think the robot decides to spin that motor?"
*   "How can we prove the code is reaching this `if` statement? Can we print something to the console or SmartDashboard?"
*   "Let's isolate the problem. Can we create a simple command that *only* spins that one motor, ignoring the sensors, just to prove the motor works?"