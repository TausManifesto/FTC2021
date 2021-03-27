//for testing autonomous methods

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "test run", group = "Tau")
public class Test extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Starting RunOpMode");
        //initializing robot
        initializeRobot();
        //int numRings = findNumRings(bmp);
        //bmp.recycle();
        //telemetry.addData("rings", numRings);
        telemetry.update();
        runWithEncoders();

        //strafeLeft(.3, 48);
        //forward(.5, 2, 0);
        //strafeRight(.5, 48);
        backward(.5, 1, 0);

        //robot.backLeftMotor.setTargetPosition(1000);
        //robot.backRightMotor.setTargetPosition(-1000);
        //robot.frontLeftMotor.setTargetPosition(-1000);
        //robot.frontRightMotor.setTargetPosition(1000);

        //runToPosition();

        sleep(10000);



    }

}
