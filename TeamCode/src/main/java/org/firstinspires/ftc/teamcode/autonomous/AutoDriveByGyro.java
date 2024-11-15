package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Gyroscopic Autonomous", group = "Robot"
)
public class AutoDriveByGyro extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Set through each let of the path.
        // Note: Each tile is ~24 inches by ~ 24 inches
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Strafe left 45 inches at a 0 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, 45, 0.0);
        // Pushes a preloaded sample into the net zone

        // Strafe right 20 inches at a 0 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, 20, 0);

        // Dive forward 40 inches at a 0 degree heading
        robot.driveGyro(robot.DRIVE_SPEED,40, 0);

        // Strafe left 8 inches at a 0 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, -8, 0);

        // Turn clockwise 1 degree
        robot.turnGyro(robot.TURN_SPEED, 1);

        // Dive backward 38 inches at a 1 degree heading
        robot.driveGyro(robot.DRIVE_SPEED, -38, 1);
        // pushes the inside specimen into the net zone

        // Dive forward 38 inches at a 1 degree heading
        robot.driveGyro(robot.DRIVE_SPEED, 38, 1);

        // Strafe left 8 inches at a 1 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, -8, 1);

        // Dive backward 38 inches at a 1 degree heading
        robot.driveGyro(robot.DRIVE_SPEED, -38, 1);
        // pushes the middle specimen into the net zone

        // Dive forward 38 inches at a 1 degree heading
        robot.driveGyro(robot.DRIVE_SPEED, 38, 1);

        // Strafe left 8 inches at a 1 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, -8, 1);

        // Dive backward 38 inches at a 1 degree heading
        robot.driveGyro(robot.DRIVE_SPEED, -38, 1);
        // pushes the outside specimen into the net zone

        // Turn counterclockwise 1 degree
        robot.turnGyro(robot.TURN_SPEED, -1);

        // Strafe right 132 inches at a 0 degree heading
        robot.strafeGyro(robot.STRAFE_SPEED, 132, 0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pauses to display thr final telemetry message.
    }
}
