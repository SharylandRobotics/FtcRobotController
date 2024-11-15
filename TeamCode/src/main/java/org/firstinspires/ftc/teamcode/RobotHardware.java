package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armDrive = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    private IMU imu = null; // Universal IMU interface

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */
    public ElapsedTime runtime = new ElapsedTime();

    public double headingError;
    public double targetHeading;

    public double drivePower;
    public double strafePower;
    public double turnPower;

    public double leftFrontPower;
    public double leftBackPower;
    public double rightFrontPower;
    public double rightBackPower;

    public int leftFrontTarget;
    public int leftBackTarget;
    public int rightFrontTarget;
    public int rightBackTarget;

    public double DRIVE_SPEED;
    public double STRAFE_SPEED;
    public double TURN_SPEED;

    public double COUNTS_PER_MOTOR_REV;
    public double DRIVE_GEAR_REDUCTION;
    public double WHEEL_DIAMETER_INCHES;
    public double COUNTS_PER_INCH;

    public double ARM_TICKS_PER_DEGREE;

    public double ARM_START;
    public double ARM_VERTICAL;
    public double ARM_HORIZONTAL;

    public double armPosition;

    public double P_DRIVE_GAIN;
    public double P_STRAFE_GAIN;
    public double P_TURN_GAIN;
    public double HEADING_THRESHOLD;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robots' hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map and initialized.
     */
    public void init(){

        // Define and initialize ALL installed motors (note: need to use reference to the actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_drive");

        DRIVE_SPEED = 0.6; // Maximum autonomous driving speed for better distance accuracy.
        STRAFE_SPEED = 0.6; // Maximum autonomous strafing speed for better distance accuracy.
        TURN_SPEED = 0.4; // Maximum autonomous turning speed for better rotational accuracy.

        /*
        Calculate the COUNTS_PER_INCH for your specific drive train. Go to your motor vendor website to determine your
        motor's COUNT_PER_MOTOR_REV. For external drive gearing set DRIVE_GEAR_REDUCTION as needed. For example, use a
        value of 2.0 for a 12-tooth spur driving a 24-tooth spur gear. This is gearing DOWN for less speed and more
        torque. For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of the wheel
        rotation.
         */
        COUNTS_PER_MOTOR_REV = 1120; // AndyMark NeveRest
        DRIVE_GEAR_REDUCTION =  40.0 / 80.0; // 40 tooth input gear, 80 tooth output gear.
        WHEEL_DIAMETER_INCHES = 3.5; // Tetrix
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        /*
        Define the proportional control coefficient (or GAIN) for the "heading control". We define one value, when
        turning (larger errors), and the other is used when driving straight (smaller errors). Increase these numbers  if
        the heading does not correct strongly enough (e.g., a heavy robot or using tracks). Decrease these numbers if the
        heading does not settle on the correct value (e.g., a very agile robot with omni wheels).
         */
        P_DRIVE_GAIN = 0.02;
        P_STRAFE_GAIN = 0.01;
        P_TURN_GAIN = 0.02;

        /*
        How close the heading must get to the target before moving to the next step. Requiring more accuracy (a smaller
        number) will often make the turn take longer to get into the final position.
        */
        HEADING_THRESHOLD = 1.0;

        /*
        28: (number of encoder ticks per rotation of the bare motor)
        60: (exact gear ratio of the 60:1 NeveRest gearbox)
        120.0 / 40.0: (external gear reduction, a 40T that drives a 120 T gear)
        1 / 360.0 (ticks per degree, not per rotation)
        */
        ARM_TICKS_PER_DEGREE = 28 * 60 * 120.0 / 40.0 * 1 / 360.0;

        /*
        These constants hold the position that the arm is commanded to run to and are relative to where the arm was
        located when you start the OpMode. In these variables you will see a number of degrees, multiplied by the ticks
        per degree of the arm. This results in the number of encoders ticks the arm needs to move to achieve the ideal
        set position of the arm.
        */
        ARM_START = 0 * ARM_TICKS_PER_DEGREE;
        ARM_VERTICAL = 75 * ARM_TICKS_PER_DEGREE;
        ARM_HORIZONTAL = 170 * ARM_TICKS_PER_DEGREE;
        armPosition = (int)ARM_START;

        // Define and initialize ALL installed sensors (note: need to use reference to the actual OpMode).
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);

        /*
         Most robots need the motors on one side to be reversed to drive forward. The motor reversals shown here are
         for a "direct drive" robot (the wheels turn in the same direction as the motor shaft). If your robot has
         additional gear reductions or uses a right-angled drive, it is important to ensure that your motors are turning
         in the correct direction. So, start out with the reversals here, BUT when you first test your robot, push the
         left joystick forward and observe the wheels turn. Reverse the direction (flip FORWARD <-> REVERSE) of any
         wheel that runs backward. Keep testing until ALL the wheels move the robot forward when you push the left
         joystick forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary. Reset the encoders and set the motors to BREAK mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the TargetPostiion to 0. Then we'll set the RunMode to RUN_TO_POSITION, and we'll ask it to stop and reset.
        armDrive.setTargetPosition(0);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

        // Wait for the game to start (Display Gyro value while waiting)
        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("\nStarting at", "%7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("\nOrientation", "Robot Heading = %4.0f", getHeading());
            myOpMode.telemetry.update();
        }
    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        leftFrontPower = (drive + strafe + turn) / denominator;
        leftBackPower = (drive - strafe + turn) / denominator;
        rightFrontPower = (drive - strafe - turn) / denominator;
        rightBackPower = (drive + strafe - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {

        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(driveRotation) + Math.abs(strafeRotation) + Math.abs(turn), 1);
        leftFrontPower = (driveRotation + strafeRotation + turn) / denominator;
        leftBackPower = (driveRotation - strafeRotation + turn) / denominator;
        rightFrontPower = (driveRotation - strafeRotation - turn) / denominator;
        rightBackPower = (driveRotation +  strafeRotation - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Drive based on encoder counts. Move will stop if either of the following conditions occurs: 1) Move gets to the
     * desired position. 2) Move runs out of time. 3) Driver stops the OpMode running.
     *
     * @param speed                 magnitude of the movement
     * @param leftFrontInches       desired distance traveled of the leftFrontDrive
     * @param leftBackInches        desired distance traveled of the leftBackDrive
     * @param rightFrontInches      desired distance traveled of the rightFrontDrive
     * @param rightBackInches       desired distance traveled of the rightBackDrive
     * @param timeoutS              how many seconds the move will last
     */
    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches,  double rightFrontInches,
                             double rightBackInches, double timeoutS) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            setArmPosition();

            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            // Turn on RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion
            runtime.reset();
            setDrivePower(Math.abs(speed),Math.abs(speed),Math.abs(speed),Math.abs(speed));

            /*
            Keep looping while we are still active, and there is time left, and all motors are running. Note, We use
            (isBusy() && isBusy() && isBusy()) in the loop test, which means that when ANY motor hits its target
            position, the motion will stop. This is "safer" in the event that the robot will alwyas end the motion as
            soon as possible. However, if you require that ALL motors have finished their moves before the robot
            continues onto the next step, use (isBusy() || isBusy() || isBusy() || isBusy()) in the loop test.
             */
            while (myOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to ", "%7d :%7d :%7d :%7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Currently at ", "%7d :%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motions
            setDrivePower(0,0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250); // optional pause after each move.
        }
    }

    /**
     * Drive in a straight line, on a fixed heading (angle), based on encoder counts. Move will stop if either of these
     * conditions occurs: 1) Move gets to the desired position or 2) Driver stops the Mode running.
     *
     * @param maxDriveSpeed     Maximum speed for the forward/reverse motion (0.0 to 1.0)
     * @param distance          Distance (in inches) to move from current position (-1.0 to 1.0) +ve is forward
     * @param heading           Absolute heading angle (in degrees) relative to the last gyro reset. (-1.0 to 1.0) + ve
     *                          is clockwise. If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveGyro(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine the new target position, and pass the value to the motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed (must be positive for RUN_TO_POSITION). Start driving straight, and then
            // enter the control loop.
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveRobotCentric(maxDriveSpeed, 0, 0);

            // Keep looping while we are still active and ALL motors are running.
            while (myOpMode.opModeIsActive() && (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                    rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnPower = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // If driving in reverse, the motor correction also needs to be reversed.
                if (distance < 0) {
                    turnPower *= -1.0;
                }

                // Apply the turning correction to the current driving speed.
                driveRobotCentric(drivePower, 0, turnPower);

                // Display the drive status for the driver.
                myOpMode.telemetry.addData("Motion", "Driving");
                myOpMode.telemetry.addData("Target Pos LFD : LBD : RFD : RBD", "%7d : %7d : %7d : %7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Actual Pos LFD : LBD : RFD : RBD", "%7d : %7d : %7d : %7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            }

            // Stop all motions
            setDrivePower(0,0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250); // optional pause after each move.
        }
    }

    /**
     * Strafe in a straight line, on a fixed heading (angle), based on encoder counts. Move will stop if either of these
     * conditions occurs: 1) Move gets to the desired position or 2) Driver stops the Mode running.
     *
     * @param maxStrafeSpeed    Maximum speed for the right/left motion (0.0 to 1.0)
     * @param distance          Distance (in inches) to move from current position (-1.0 to 1.0) +ve is forward
     * @param heading           Absolute heading angle (in degrees) relative to the last gyro reset. (-1.0 to 1.0) + ve
     *                          is clockwise. If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void strafeGyro(double maxStrafeSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine the new target position, and pass the value to the motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed (must be positive for RUN_TO_POSITION). Start driving straight, and then
            // enter the control loop.
            maxStrafeSpeed = Math.abs(maxStrafeSpeed);
            driveRobotCentric(0, maxStrafeSpeed, 0);

            // Keep looping while we are still active and ALL motors are running.
            while (myOpMode.opModeIsActive() && (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                    rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnPower = getSteeringCorrection(heading, P_STRAFE_GAIN);

                // If driving in reverse, the motor correction also needs to be reversed.
                if (distance < 0) {
                    turnPower *= -1.0;
                }

                // Apply the turning correction to the current driving speed.
                driveRobotCentric(0, strafePower, turnPower);

                // Display the drive status for the driver.
                myOpMode.telemetry.addData("Motion", "Strafing");
                myOpMode.telemetry.addData("Target Pos LFD : LBD : RFD : RBD", "%7d : %7d : %7d : %7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Actual Pos LFD : LBD : RFD : RBD", "%7d : %7d : %7d : %7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            }

            // Stop all motions
            setDrivePower(0,0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250); // optional pause after each move.
        }
    }

    /**
     * Spin on the central axis to point in a new direction. Move will stop if either of these conditions occurs: 1)
     * Turn gets to the desired position or 2) Driver stops to Mode running
     * @param maxTurnSpeed      Maximum speed for the clockwise/counterclockwise motion (0.0 to 1.0)
     * @param heading           Absolute heading angle (in degrees) relative to the last gyro reset. (-1.0 to 1.0) + ve
     *                          is clockwise. If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void turnGyro(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // Keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine the required steering to keep on heading.
            turnPower = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnPower = Range.clip(turnPower, -maxTurnSpeed, -maxTurnSpeed);

            // Piviot in place by applying the turning correction
            driveRobotCentric(0,0, turnPower);

            // Display drive status for the driver.
            myOpMode.telemetry.addData("Motion", "Turning");
            myOpMode.telemetry.addData("Heading - Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
            myOpMode.telemetry.addData("Error : Steer Pwr", "%5.1f : %5.1f", headingError, turnPower);
            myOpMode.telemetry.addData("Wheel Power LFD : LBD : RFD : RBD", "%5.2f : %5.2f : %5.2f : %5.2f",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            myOpMode.telemetry.update();

            // Stop all motions
            setDrivePower(0,0,0,0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250); // optional pause after each move.
        }
    }

    /**
     * Use a Proportional Controller to deteming how much steering correction is required
     *
     * @param desiredHeading   The desired heading (relative to the last heading reset
     * @param proportionalGain The gain factors applied to heading error to get the turning power
     * @return Turning power needed to get to the required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        // Save targetHeading for telemetry
        targetHeading = desiredHeading;

        // Determine the current heading error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) {
            headingError -= 360;
        }
        while (headingError <= 180) {
            headingError += 360;
        }

        // Multiply the error by the gain to determine the required steering correction. Limit the result to +/- 1.0.
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel,
                              double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * set the target position of our arm to match the variables that was selected by the driver. We also set the target
     * velocity (speed) the motor runs at, and use setMode to run it.
     */
    public void setArmPosition() {
        armDrive.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armDrive).setVelocity(2500);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}