# Adding Level 3 Logging to the WPILib DifferentialDrive Java Template

## What are the logging levels?

Team 6328 Mechanical Advantage defines the levels of FRC logging as:
1. Driver Station logging: voltage, current draw, network diagnostics, and console prints.
1. Onboard logging: Recording to USB drive, network logging. This includes WPILib's native logging features and other vendor-specific tools.
1. Log replay: "Log everything," full internal state of the robot is accessible using a log file.

## What is this repo?

I started with the DifferentialDrive Java template using WPILib> Create New Project and implemented level 3 logging into the base example with clear and individual steps. Most steps only edit or create a single file.

## How do I use this repo?

You should click on the `# Commits` button in the top right of the Github file explorer and go to the second commit from the bottom titled: __Install AdvantageKit into existing project, with additional best practices__

Start a project with the DifferentialDrive Java template and follow along coding with each commit __without copying and pasting__. Each commit has a very thorough description of what changes were made and why. You will learn the fundamentals of refactoring a subsytem to do level 3 logging.

## AI Summary of each commit

1. We started with WPILib's basic differential drive example. This gives us working code that can drive a robot forward/backward and turn, but it doesn't have any advanced features like logging or simulation.
1. We add AdvantageKit, which is a powerful logging tool. In build.gradle we tell our project to use it, and in Robot.java we set it up to record data about our robot. This will help us debug problems by recording everything that happens during matches. We also add Constants.java so we can switch between real robot, simulation, and replay modes.
1. We create DriveSideIO.java as an "interface" - think of it as a blueprint that says "any code controlling drive motors must include these features." It defines what data we want to track (like motor speed and voltage) and what actions we need (like setting motor speed). This helps us organize our code and makes it easier to switch between real and simulated hardware.
1. Similar to step 3, we create GyroIO.java as a blueprint for gyroscope sensors. A gyro tells us which way the robot is pointing, so we define what data we need from it (rotation angle and speed) and how we'll get that data.
1. Now we implement our first "real" hardware code with DriveSideIOSparkMax.java. This takes our DriveSideIO blueprint and fills it in with actual code to control SparkMax motors. It handles all the details of setting voltages, reading encoder values, and controlling motor speed.
1. We create a simulator version (DriveSideIOSim.java) that pretends to be real motors. Instead of sending commands to actual hardware, it uses physics equations to calculate how the motors would behave. This lets us test our code without needing the real robot.
1. We implement GyroIOAnalog.java to work with a real gyro sensor. Like the motor code, this takes our GyroIO blueprint and fills it in with code that can actually read from an analog gyro sensor.
1. We rewrite the main Drivetrain code to use our new motor and gyro interfaces. Now instead of talking directly to hardware, it uses our blueprints. This means the same code works whether we're using real hardware or simulation.
1. We fix a small bug where we needed to change some special data types (Time[], Angle[]) to simple number arrays (double[]) because AdvantageKit couldn't handle the special types.