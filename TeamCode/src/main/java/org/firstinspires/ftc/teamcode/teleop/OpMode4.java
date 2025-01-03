package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware2;

//test
@TeleOp(name = "OpMode4")

public class OpMode4 extends LinearOpMode {
    /*
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    private double lastError = 0;

    PIDController control = new PIDController();
*/
    double rotatePosition = 0.5;
    double horizontalPosition = 0;

    ElapsedTime timer = new ElapsedTime();


    RobotHardware2 robot = new RobotHardware2();

    private int sleepMs1 = 0;
//

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init2(hardwareMap);

        // robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.liftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // robot.liftHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {

            double horizontal = gamepad1.left_stick_x * 0.7; // -0.7
            double vertical = -gamepad1.left_stick_y * 0.7; // 0.7
            double turn = gamepad1.right_stick_x * 0.7;

            robot.setDrivePower(vertical + turn + horizontal, vertical - turn - horizontal, vertical + turn - horizontal, vertical - turn + horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));

           /* double Power = pidControl(100, robot.pivotMotor.getVelocity());


            while (gamepad2.left_stick_y > 0.7) {
                robot.pivotMotor.setPower(Power);
                sleep(15);
            }

            while (gamepad2.left_stick_y < -0.7) {
                robot.pivotMotor.setPower(-Power);
                sleep(15);
            }

*/
            if (gamepad2.y) {
                rotatePosition += 0.1;

                if (rotatePosition >= 1.0) {
                    rotatePosition = 1.0;
                }

                robot.rotationServo.setPosition(rotatePosition);

            }

            if (gamepad2.a) {
                rotatePosition -= 0.1;

                if (rotatePosition >= 0.0) {
                    rotatePosition = 0.0;
                }

                robot.rotationServo.setPosition(rotatePosition);

            }


            while (gamepad2.right_trigger > 0.7) {
                horizontalPosition += 0.02;

                if (horizontalPosition >= 1.0) {
                    horizontalPosition = 1.0;
                }

                robot.extentionServo.setPosition(horizontalPosition);
                sleep(30);

            }

            while (gamepad2.left_trigger > 0.7) {
                horizontalPosition -= 0.02;

                if (horizontalPosition >= 0.0) {
                    horizontalPosition = 0.0;
                }

                robot.extentionServo.setPosition(horizontalPosition);
                sleep(30);

            }



                if (gamepad2.right_bumper) {
                    robot.grabServo.setPosition(1);
                }

                if (gamepad2.left_bumper) {
                    robot.grabServo.setPosition(0);
                }

/*
                while (gamepad1.right_trigger > 0.7) {
                    robot.rightLiftMotor.setPower(-0.7);
                    robot.leftLiftMotor.setPower(0.7);
                }

                while (gamepad1.left_trigger > 0.7) {
                    robot.rightLiftMotor.setPower(0.7);
                    robot.leftLiftMotor.setPower(-0.7);
                }

                robot.rightLiftMotor.setPower(0);
                robot.leftLiftMotor.setPower(0);

*/


                if (gamepad2.right_stick_y > 0.7) {
                    robot.actuatorMotor.setPower(-1);
                } else if (gamepad2.right_stick_y < -0.7) {
                    robot.actuatorMotor.setPower(1);
                } else {
                    robot.actuatorMotor.setPower(0);
                }


                if (gamepad2.left_stick_y > 0.7) {
                    robot.pivotMotor.setPower(-0.4);
                } else if (gamepad2.left_stick_y < -0.7) {
                    robot.pivotMotor.setPower(0.4);
                } else {
                    robot.pivotMotor.setPower(0.08);
                }

                // 0.7      1                    -0.7

//make sure one of the directions is correct/reversed


                if (gamepad2.b){
                    robot.tiltServo.setPosition(0.7);
                }

                if (gamepad2.x){
                    robot.tiltServo.setPosition(0);
                }

                //spin

                //tilt bucket





/*           //lift arm start
            if (gamepad2.a) { //if button a pressed
                robot.intakeServo.setPosition(1);
            }

            else if (gamepad2.y) {
                robot.intakeServo.setPosition(0);
            }

            else{
                robot.intakeServo.setPosition(0.5);
            }

*/
/*
            if (gamepad2.y) { //if button a pressed
                // Extend liftArm
                robot.liftArm.setPower(0.8);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }
            if (gamepad2.a) { //if button a pressed
                // Retract liftArm
                robot.liftArm.setPower(-1.0);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }

/*
            if(gamepad1.right_trigger > 0.7){
                robot.airplaneLauncher.setPosition(1.0);
            }






        //emergency releases
    }

    void liftHexArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();

        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftHex.setTargetPosition(ticks);
        robot.liftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftHex.setPower(power);
        while(opModeIsActive()
                && robot.liftHex.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();
        }

    }

    private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);
        }
*/
            }

        }


    }