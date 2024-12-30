/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    /* Declare OpMode members. */
    HardwareMap hwMap =  null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;
    public DcMotor motorbl = null;

    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;



    //public DcMotor launcher = null;

    //public DcMotor linearSlider = null;


    //public DistanceSensor distanceR = null;
    //public DistanceSensor distanceL = null;


    public Servo bucketTilt = null;


    public CRServo intakeServo = null;
    //public Servo tiltServoLeft = null;
    //public Servo grabServoLeft = null;


    //public Servo airplaneLauncher = null;

    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // save reference to hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motorfl = hwMap.get(DcMotor.class, "front_left");
        motorfr = hwMap.get(DcMotor.class, "front_right");
        motorbl = hwMap.get(DcMotor.class, "back_left");
        motorbr = hwMap.get(DcMotor.class, "back_right");

        leftLiftMotor = hwMap.get(DcMotor.class, "lift_left");
        rightLiftMotor = hwMap.get(DcMotor.class, "lift_right");



        // liftHex = hwMap.get(DcMotor.class, "liftHex");
        // set Brake zero power behavior
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        bucketTilt = hwMap.get(Servo.class, "bucket_servo");
        intakeServo = hwMap.get(CRServo.class, "intake_servo");


        //intakeTilt = hwMap.get(Servo.class, "intake_servo");
        //tiltServoLeft = hwMap.get(Servo.class, "tiltServoL");
        //grabServoLeft = hwMap.get(Servo.class, "grabServoL");
        //autoPixel.setPosition(0.5);
        //boardPixel.setPosition(0);
        //grabServo.setPosition(0.4);

        // Get distance sensors
        //distanceR = hwMap.get(DistanceSensor.class, "distanceR");
        //distanceL = hwMap.get(DistanceSensor.class, "distanceL");

        // Set airplane launcher default servo position
        //airplaneLauncher.setPosition(0);

        // reverse motor directions
      //  motorfl.setDirection(DcMotorSimple.Direction.REVERSE);
      //  motorfr.setDirection(DcMotorSimple.Direction.REVERSE);
      //  motorbr.setDirection(DcMotorSimple.Direction.REVERSE);
      //  motorbl.setDirection(DcMotorSimple.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Retrieve the very initial Rotational Angles and Velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    public void setAutoDriveMotorMode() {
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }
    public void setArmsMode(DcMotor.RunMode mode) {
        //linearSlider.setMode(mode);
    }
    /*
    public void setDriveForward(double fl, double fr, double bl, double br){
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }
*/

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl); //back left
        motorfr.setPower(-1 * fr);
        motorbl.setPower(bl);
        motorbr.setPower(-1 * br); //backright

    }

    public void setLiftPower(double leftm, double rightm) {
        if (leftm > 1.0)
            leftm = 1.0;
        else if (leftm < -1.0)
            leftm = -1.0;

        if (rightm > 1.0)
            rightm = 1.0;
        else if (rightm < -1.0)
            rightm = -1.0;

        leftLiftMotor.setPower(leftm);
        rightLiftMotor.setPower(rightm);
    }
    public void setAllDrivePower(double p){ setDrivePower(p,p,p,p);}
    //public void setArmPower(double armPower){
     //   linearSlider.setPower(armPower);
    //}






}
/*
   port 1 motorfl
   port 2 motorbl
   port 3 motorap
   extension hub port 0 motorfr
   extension hub port 1 motorbr
   extension hub port 2 liftArm
   extension hub port 3 liftHex

   Servos
   port 0 boardPixel
   port 1 grabServo
   port 2 tiltServo

   extension hub port 0 autoPixel
   extension hub port 1 feeder


   //Controls//
   Gamepad 1:

     */