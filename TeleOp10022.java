package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class TeleOp10022 extends OpMode {

    //Declare Hardware

    //Drivetrain
    DcMotor frontleft, frontright, backleft, backright;

    //Lift
    DcMotor liftMotor;

    //Sensors
    ColorSensor goldSensor;

    //Linear Slide
    DcMotor horizontalSlide, verticalSlide;

    //Intake & Outtake
    DcMotor intakeTubing;
    Servo intakeRotateOne, intakeRotateTwo, outtakeRotate;

    //Variables
    int toggleOne = 0;
    int toggleTwo = 0;

    //REV IMU
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void init () {

        //Initialization

        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        //Drivetrain
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        //Lift
        liftMotor = hardwareMap.dcMotor.get("liftmotor");

        //Linear Slide
        horizontalSlide = hardwareMap.dcMotor.get("horizontalslide");
        verticalSlide = hardwareMap.dcMotor.get("verticalslide");

        //Intake & Outtake
        intakeTubing = hardwareMap.dcMotor.get("intaketubing");
        intakeRotateOne = hardwareMap.servo.get("intakerotateone");
        intakeRotateTwo = hardwareMap.servo.get("intakerotatetwo");
        outtakeRotate = hardwareMap.servo.get("outtakerotate");

        //Sensors
        goldSensor = hardwareMap.colorSensor.get("goldsensor");

    }

    @Override
    public void loop() {

        //Drivetrain

        /*
        Gamepad 1:
        Left Analog Y - Forward, Backward
        Left Analog X - Strafing
        Right Analog X - Rotation
         */

        float lefty = -gamepad1.left_stick_y;
        float leftx = -gamepad1.left_stick_x;
        float rightx = gamepad1.right_stick_x;

        frontleft.setPower(.8*(lefty - leftx + rightx));
        backleft.setPower(.8*(lefty + leftx + rightx));
        frontright.setPower(.8*(lefty + leftx - rightx));
        backright.setPower(.8*(lefty - leftx - rightx));

        //Sensor
        telemetry.addData("R: ", goldSensor.red());
        telemetry.addData("G: ", goldSensor.blue());
        telemetry.addData("B: ", goldSensor.green());
        telemetry.update();

        //IMU Orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();

        //Lift

        /*
        Gamepad 1:
        Right Bumper - Lift Up
        Left Bumper - Lift Down
         */

        if (gamepad1.right_bumper == true) {

            liftMotor.setPower(1.0);

        }

        else if (gamepad1.left_bumper == true) {

            liftMotor.setPower(-1.0);

        }

        else {

            liftMotor.setPower(0.0);

        }

        //Linear Slide (Horizontal)

        /*
        Gamepad 2:
        Dpad Right - Horizontal Slide Extend
        Dpad Left - Horizontal Slide Retract
         */

        if (gamepad2.dpad_left == true) {

            horizontalSlide.setPower(1.0);

        }

        else if (gamepad2.dpad_right == true) {

            horizontalSlide.setPower(-1.0);

        }

        else {

            horizontalSlide.setPower(0.0);

        }


        //Linear Slide (Vertical)

        /*
        Gamepad 1:
        Dpad Up - Vertical Slide Extend
        Dpad Down - Vertical Slide Retract
         */

        if (gamepad1.dpad_up == true && outtakeRotate.getPosition() != 1) {

            verticalSlide.setPower(-1.0);

        }

        else if (gamepad1.dpad_down == true && outtakeRotate.getPosition() != 1) {

            verticalSlide.setPower(1.0);

        }

        else {

            verticalSlide.setPower(0.0);

        }

        //Intake & Outtake

        //Intake Tubing

        /*
        Gamepad 2:
        Right Bumper - Intake Tubing Collect
        Left Bumper - Intake Tubing Deposit
         */

        if (gamepad2.right_bumper == true) {

            intakeTubing.setPower(-1);

        }

        else if (gamepad2.left_bumper == true) {

            intakeTubing.setPower(1);

        }

        else {

            intakeTubing.setPower(0);

        }

        //Intake Rotate

        /*
        Gamepad 2:
        A - Lower Intake Box
        B - Raise Intake Box
         */

        /*if (gamepad2.a == true) {

            intakeRotateOne.setPosition(.925);

        }

        else if (gamepad2.b == true) {

            intakeRotateOne.setPosition(0.4);

        }*/

        if (gamepad2.a == true) {

            intakeRotateTwo.setPosition(0.6);

        }

        else if (gamepad2.b == true) {

            intakeRotateTwo.setPosition(.15);

        }

        //Outtake Rotate

        /*
        Gamepad 1:
        A - Raise Outtake Box
        B - Reposition Outtake Box
        X - Lower Outtake Box
         */

        if (gamepad1.a == true) {

            outtakeRotate.setPosition(1);

        }

        else if (gamepad1.x == true) {

            outtakeRotate.setPosition(.4);

        }

        else if (gamepad1.b == true) {

            outtakeRotate.setPosition(0.75);

        }

        /*

        Gamepad 1:

        Left Analog Y - Forward, Backward
        Left Analog X - Strafing
        Right Analog X - Rotation

        Right Bumper - Lift Up
        Left Bumper - Lift Down

        Dpad Up - Vertical Slide Extend
        Dpad Down - Vertical Slide Retract

        A - Raise Outtake Box
        B - Reposition Outtake Box
        X - Lower Outtake Box

        Gamepad 2:

        Dpad Right - Horizontal Slide Extend
        Dpad Left - Horizontal Slide Retract

        Right Bumper - Intake Tubing Collect
        Left Bumper - Intake Tubing Deposit

        A - Lower Intake Box
        B - Raise Intake Box

         */

    }

}
