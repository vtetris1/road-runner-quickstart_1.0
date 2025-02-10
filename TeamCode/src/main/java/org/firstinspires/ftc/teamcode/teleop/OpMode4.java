package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    int pivotTicks = 0;

    double pivotArmSpeed = 0.6;


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
        //robot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.liftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // robot.liftHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here.
        waitForStart();
        double controller1Speed = 0;
        while (opModeIsActive()) {

            double horizontal = gamepad1.left_stick_x * 0.7 * controller1Speed; // -0.7
            double vertical = -gamepad1.left_stick_y * 0.7 * controller1Speed; // 0.7
            double turn = gamepad1.right_stick_x * 0.7 * controller1Speed;

            robot.setDrivePower(vertical + turn + horizontal, vertical - turn - horizontal, vertical + turn - horizontal, vertical - turn + horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));

            if (gamepad1.right_bumper) {
                controller1Speed = 0.3;
            } else {
                controller1Speed = 1;
            }
           /* while(gamepad2.right_stick_y > 0.7){
                turnPivotMotor(0.4, pivotTicks);
                sleep(100);
                pivotTicks += 50;
            }

            while(gamepad2.right_stick_y < -0.7){
                turnPivotMotor(0.4, pivotTicks);
                sleep(100);
                pivotTicks += 50;
            }

*/

            if (gamepad2.right_stick_y > 0.7){
                turnPivotMotor(1, (pivotTicks + 2000));

                telemetry.addLine(String.format("pivot ticks: %d",
                        pivotTicks
                ));

            }

            else if (gamepad2.right_stick_y < -0.7){
                turnPivotMotor(1, -(pivotTicks + 2000));

                telemetry.addLine(String.format("pivot ticks: %d",
                        pivotTicks
                ));

            }

            else{
                turnPivotMotor(0,0);
            }



            if (gamepad2.dpad_down) {
                robot.rotationServo.setPosition(1);
            }
            if (gamepad2.dpad_right) {
                robot.rotationServo.setPosition(0.7);
            }
            if (gamepad2.dpad_up) {
                robot.rotationServo.setPosition(0.45);
            }
            if (gamepad2.dpad_left) {
                robot.rotationServo.setPosition(0.3);
            }


            if (gamepad2.left_trigger > 0.7) {
                robot.extensionServo.setPower(0.5);
            }

            else if (gamepad2.right_trigger > 0.7) {
                robot.extensionServo.setPower(-0.5);
            }
            else{
                robot.extensionServo.setPower(0);
            }


            if (gamepad2.right_bumper) {
                robot.grabServo.setPosition(0.6); //close
                }

            if (gamepad2.left_bumper) {
                robot.grabServo.setPosition(0.3);
                }




            if (gamepad1.left_trigger > 0.7) {
                robot.actuatorMotor.setPower(-1);
            }
            else if (gamepad1.right_trigger > 0.7) {
                robot.actuatorMotor.setPower(1);
            }
            else {
                robot.actuatorMotor.setPower(0);
            }



            if (gamepad2.left_stick_y > 0.7) {
                robot.actuatorMotor.setPower(-1);
            }
            else if (gamepad2.left_stick_y < -0.7) {
                robot.actuatorMotor.setPower(1);
            }
            else {
                robot.actuatorMotor.setPower(0);
            }



                // 0.7      1                    -0.7

//make sure one of the directions is correct/reversed

            if(gamepad2.a){
                robot.tiltServo.setPosition(0.5);
            }

            if (gamepad2.b){
                robot.tiltServo.setPosition(1);
            }

            if (gamepad2.x){
                robot.tiltServo.setPosition(0.3); //0
            }

            if (gamepad2.y){
                robot.tiltServo.setPosition(0); //0
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

    private void turnPivotMotor(double power, int pivotTarget){
        robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.pivotMotor.setTargetPosition(pivotTarget);

        robot.pivotMotor.setPower(power);
        robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    }