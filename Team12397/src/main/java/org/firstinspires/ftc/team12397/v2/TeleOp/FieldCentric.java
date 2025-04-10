package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.team12397.v2.RobotHardware;

@TeleOp(name="FieldCentric", group="Robot")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        double slideRotation = 0;

        Gamepad luisL = gamepad1;
        Gamepad alexH = gamepad2;

        robot.init();
        waitForStart();

        while (opModeIsActive()) {

            drive = -luisL.left_stick_y;
            strafe = luisL.left_stick_x;
            turn = luisL.right_stick_x;

            robot.driveFieldCentric(drive, strafe, turn);


            if(alexH.y){
                slideRotation = robot.ROTATION_90;
            }else if (alexH.a){
                slideRotation = robot.ROTATION_START;
            }

            robot.RotateSlides(slideRotation);

        }
    }

}
