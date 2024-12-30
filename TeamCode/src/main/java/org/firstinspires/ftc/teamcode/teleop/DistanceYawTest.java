/*package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

//test
@TeleOp(name = "DistanceYawTest", group = "Sensor")

public class DistanceYawTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        robot.imu.resetYaw();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("Yaw: %.1f\n",
                    robot.getCurrentYaw()));
            telemetry.update();
            double horizontal = gamepad1.left_stick_x * -1; // -0.7
            double vertical = -gamepad1.left_stick_y * 1; // 0.7
            double turn = gamepad1.right_stick_x * 0.7;

            robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);
            telemetry.addLine(String.format("Yaw: %.1f\n", robot.getCurrentYaw()));
            telemetry.update();
        }
    }
}
*/