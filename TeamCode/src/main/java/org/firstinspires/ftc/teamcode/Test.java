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
        controlIndexServo(1);
        int numRings = findNumRings(bmp);
        bmp.recycle();
        telemetry.addData("rings", numRings);
        telemetry.update();
        runWithEncoders();

        //strafeLeft(.5, 24);
        //forward(1, 4, 0);
        //strafeRight(.5, 24);
        //backward(.5, 1, 0);

        //runToPosition();
        //shoot(0, shooterPower);

        sleep(30000);



    }

}
