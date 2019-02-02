package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

//Runtime and Pulses

public abstract class Auto10022 extends LinearOpMode {

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
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    int colorTest = 0;
    int positionTest = 0;

    //Counter
    private ElapsedTime Runtime = new ElapsedTime();

    //REV IMU
    BNO055IMU imu;
    Orientation angles;

    public void initialize() {

        //Slide Initialization
        horizontalSlide = hardwareMap.dcMotor.get("horizontalslide");

        //Lift Initializtion
        liftMotor = hardwareMap.dcMotor.get("liftmotor");

        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        //Drivetrain Initialization
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");

        //Reverse Motors and Servos
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        //Encoders
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sensor Initialization
        goldSensor = hardwareMap.colorSensor.get("goldsensor");

        //Servo Initialization
        intakeRotateOne = hardwareMap.servo.get("intakerotateone");
        intakeRotateTwo = hardwareMap.servo.get("intakerotatetwo");
        intakeTubing = hardwareMap.dcMotor.get("intaketubing");

    }

    public void resetEncoders() {

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void stopDrive() {

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

    }

    public void getGyro() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();

    }

    public void driveForward(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", backleft.getTargetPosition(), backright.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(-power);
                backright.setPower(-power);
                frontleft.setPower(-power);
                frontright.setPower(-power);

            }

            Runtime.reset();

            //Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            //Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void driveBackward(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newbackleftTarget, newbackrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", frontleft.getCurrentPosition(), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

            }

            Runtime.reset();

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void strafeRight(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", backleft.getTargetPosition(), backright.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

            }

            Runtime.reset();

            //Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            //Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void strafeLeft(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", backleft.getTargetPosition(), backright.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

            }

            Runtime.reset();

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void rotateRight(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", backleft.getTargetPosition(), backright.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

            }

            Runtime.reset();

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void rotateLeft(double distance, double power) {

        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        if (opModeIsActive()) {

            newfrontleftTarget = frontleft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backright.isBusy() && frontright.isBusy() && frontleft.isBusy() && backleft.isBusy()) {

                telemetry.addData("Path1",  "Running to %7d :%7d", backleft.getTargetPosition(), backright.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();

                backleft.setPower(power);
                backright.setPower(power);
                frontleft.setPower(power);
                frontright.setPower(power);

            }

            Runtime.reset();

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);


            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void gyroTurn(double angle, double power) {

        //Update angular orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(angles.firstAngle < angle) {

            //Turns Right if set angle is rightwards
            frontleft.setPower(-power);
            frontright.setPower(-power);
            backleft.setPower(power);
            backright.setPower(power);

        }

        //Update angular orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(angles.firstAngle > angle) {

            //Turns Left if set angle is Leftwards
            frontleft.setPower(power);
            frontright.setPower(power);
            backleft.setPower(-power);
            backright.setPower(-power);

        }

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

    }

    public void landRobot(double ticks, double power) {

        int newliftmotorTarget;

        if (opModeIsActive()) {

            newliftmotorTarget = liftMotor.getCurrentPosition() + (int)(ticks*100);

            liftMotor.setTargetPosition(newliftmotorTarget);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && liftMotor.isBusy()) {

                telemetry.addData("Path1", liftMotor.getTargetPosition());
                telemetry.addData("Path2", liftMotor.getCurrentPosition());
                telemetry.update();

                liftMotor.setPower(-power);

            }

            Runtime.reset();

            //Stop all motion;
            liftMotor.setPower(0);

            //Turn off RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void lowerLift(int time, double power) {

        liftMotor.setPower(power);
        sleep(time);
        liftMotor.setPower(0);

    }

    //Position 0: Middle Sample
    //Position 1: Right Sample
    //Position 2: Left Sample

    public void detectColor() {

        if (positionTest == 0) {

            if (goldSensor.green() < 60 || goldSensor.red() > 100 && goldSensor.green() < 80) {

                //Push Gold Cube
                driveForward(7.5, 0.3);
                driveBackward(8.5, 0.3);

            }

            else {

                driveBackward(3, 0.3);
                positionTest = 1;

            }

        }

        if (positionTest == 1) {

            //Move to right sample
            strafeRight(23, .3);

            driveForward(2.75, .3);

            sleep(600);

            if (goldSensor.green() < 60 || goldSensor.red() > 100 && goldSensor.green() < 80) {

                //Push Gold Cube
                driveForward(7.5, 0.3);
                driveBackward(8.5, 0.3);

            }

            else {

                driveBackward(6, .3);
                positionTest = 2;

            }

        }

        if (positionTest == 2) {

            //Move to left sample
            strafeLeft(43.5, .3);

            driveForward(4.5, .3);

            //Push Gold Cube
            driveForward(7.5, 0.3);
            driveBackward(8.5, 0.3);

        }

    }

    public void repositionRobotDepot() {

        if (positionTest == 0) {

            driveBackward(35.0, 0.6);
            rotateRight(11.2, 0.3); //45 Degrees

        }

        if (positionTest == 1) {

            driveBackward(20.5, 0.6);
            rotateRight(11.2, 0.3); //45 Degrees

        }

        if (positionTest == 2) {

            driveBackward(49.5, 0.6);
            rotateRight(11.2, 0.3); //45 Degrees

        }

    }

    public void repositionRobotCrater() {

        if (positionTest == 0) {

            driveForward(35.0, 0.6);
            rotateLeft(11.2, 0.3); //45 Degrees

        }

        if (positionTest == 1) {

            driveForward(49.5, 0.6);
            rotateLeft(11.2, 0.3); //45 Degrees

        }

        if (positionTest == 2) {

            driveForward(20.5, 0.6);
            rotateLeft(11.2, 0.3); //45 Degrees

        }

    }

    public void outtakeMarker() {

        intakeRotateTwo.setPosition(0.6);
        sleep(1000);
        intakeTubing.setPower(1);
        sleep(800);
        intakeTubing.setPower(0)

    }

}
