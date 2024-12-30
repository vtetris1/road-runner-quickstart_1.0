/*package org.firstinspires.ftc.teamcode.teleop;
//test

@TeleOp(name = "OpMode2")

public class OpMode extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.5;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.5;

            robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);


            //lift arm start
            if(gamepad1.a) { //if button a pressed
                //tilt
                mRunTime.reset(); //timer reset

                robot.liftHex.setPower(-0.5); //set motor power

                if (mRunTime.time() > 0.3 ){ //if time over 0.3 sec
                    robot.liftHex.setPower(-0.2); //set motor power
                }

                mRunTime.reset();

                robot.liftArm.setPower(0.5);

                if (mRunTime.time() > 0.3 ){
                    robot.liftArm.setPower(0);
                }

            }

            if (gamepad1.b) {
                mRunTime.reset();

                robot.liftArm.setPower(-0.5);
                robot.liftHex.setPower(0.1);


                if (mRunTime.time() > 0.3) {
                    robot.liftArm.setPower(0);
                    robot.liftHex.setPower(0);
                }

            }


/*            if (-gamepad2.left_stick_y > 0.1){
                robot.setArmPower(0.45);
            }
            else if (-gamepad2.left_stick_y < -0.1){
                robot.setArmPower(-0.45);
            }
            else{
                robot.setArmPower(0);
            }

            //servos

//grabber
            if (gamepad2.left_trigger > 0.5) {
                robot.grabServo.setPosition(0.4); // open
            }
            else if (gamepad2.right_trigger>0.5){
                robot.grabServo.setPosition(1); // close
            }

//airplane launcher
            if (gamepad2.a) {
                //robot.airplane.setPower(1); //going up
                Thread.sleep(2000);

                mRunTime.reset();

                robot.liftHex.setPower(0.1);

                if (mRunTime.time() > 0.2){
                    // robot.airplaneLauncher.setPosition(0.5);
                }
                //the reset
                Thread.sleep(500);

                robot.liftHex.setPower(0);
                //robot.airplane.setPower(0);
            }


//tilt servo
            double initTilt = 0.0;

            if (gamepad2.right_stick_y > 0.7) {
                initTilt += 0.1;

                if (initTilt > 1.0) {
                    initTilt = 1.0;
                    robot.tiltServo.setPosition(-initTilt);
                }

                else robot.tiltServo.setPosition(-initTilt);
            }

            if (gamepad2.right_stick_y < -0.7) {
                initTilt -= 0.1;

                if (initTilt < -1.0) {
                    initTilt = -1.0;
                    robot.tiltServo.setPosition(initTilt);
                }

                else robot.tiltServo.setPosition(initTilt);


            }


 /*
            telemetry.update();


        }
    }

}
*/
