package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Auto10022Depot extends Auto10022 {

    public void runOpMode() throws InterruptedException {

        initialize();

        while(!(isStarted()  || isStopRequested())) {

            idle();

        }

        waitForStart();

        lowerLift(9850, 1); //Duration required to lower the robot at roughly 12.57v

        driveBackward(1.5, .3); //Resets robot position after landing

        sleep(500);

        strafeRight(6, .4); //Unhooks the robot

        driveForward(15.6, .3);

        strafeLeft(6, .4); //Front of center mineral - Position 0

        sleep(500);

        detectColor(); //Pushes gold block

        rotateLeft(22.5, .3); //Face reversed robot towards wall between depot and crater, 90 Degrees

        repositionRobotDepot(); //Repositions robot to face depot

        driveForward(40, .8); //To the depot

        outtakeMarker(); //Drops team marker into depot

        rotateLeft(45, .7); //Face robot towards crater, 180 Degrees

        driveForward(55, 1); //Towards crater

        horizontalSlide.setPower(1); //Extend arms outward for parking

        sleep(1750);

        horizontalSlide.setPower(0); //Stop arms from extending

        idle();

    }

}
