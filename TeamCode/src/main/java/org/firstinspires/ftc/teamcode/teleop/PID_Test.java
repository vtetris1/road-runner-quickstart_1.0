package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware2;

//test
//@Config
@TeleOp(name = "PID_Test")

public  class PID_Test extends LinearOpMode {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double reference = 0;
    static boolean angleWrap = false;

    public static double integralSum = 0;

    public static double Kf = 0;

    public double degrees = 0;
    public final double degreesToRadians = degrees * Math.PI/180;


    //public static double lastReference = reference;
    //change public static double maxIntegralSum = 0;

    // change public static double a = 0; //between 0 and 1

    public static double lastError = 0;
    public static double target_position = 300;

    static ElapsedTime timer = new ElapsedTime();




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
            boolean setPointIsNotReached = true;
            while(setPointIsNotReached) {

                double Power = pidControl(target_position, robot.pivotMotor.getVelocity());
                robot.pivotMotor.setPower(Power);

                while (gamepad1.right_trigger > 0.7) {
                    robot.pivotMotor.setPower(Power);
                }

                telemetry.addLine(String.format(" pivotMotor: %d ",
                        robot.pivotMotor.getCurrentPosition()
                ));

                telemetry.addLine(String.format(" Power: %.2f ",
                        Power
                ));

                telemetry.addLine(String.format(" error: %.2f ",
                        (target_position - robot.pivotMotor.getVelocity())
                ));
                telemetry.update();

                telemetry.update();
            }

        }

    }

    public void pidController(double Kp, double Ki, double Kd, boolean angleWrap){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    public static double pidControl(double reference,double state) {
        double error  = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        if (angleWrap) {
            error = angleWrap(reference - state);
        } else {
            error = reference - state;
        }

        double output = ((error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf));

        timer.reset();
        lastError = error;

        return output;
    }

    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
