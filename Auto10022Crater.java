package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Auto10022Crater extends Auto10022 {

    public void runOpMode() throws InterruptedException {

        initialize();

        while(!(isStarted()  || isStopRequested())) {

            idle();

        }

        waitForStart();

        landRobot(50000.0, 1.0);

        sleep(500); //wait

        driveForward(17, 0.3); //3" in front of middle sample

        sleep(500); //wait

        detectColor(); //pushes gold cube

        sleep(500); //wait

        repositionRobotCrater(); //moves robot to the side

        sleep(500); //wait

        driveForward(44.5, 0.5); //to depot

        sleep(500); //wait

        outtakeMarker(); //outtakes team marker into depot

        sleep(500); //wait

        rotateRight(120, 0.5) //facing crater

        sleep(500); //wait

        driveForward(44.5, 0.5); //into crater

        sleep(500); //wait

        intakeMinerals(5000); //extend horizontal slides over crater and runs intake

        sleep(500); //wait

        idle();

    }

}