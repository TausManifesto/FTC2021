//for testing autonomous methods

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "test run", group = "Tau")
public class Test extends AutonomousMethods {

    public static double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addLine("Starting RunOpMode");
        //initializing robot
        initializeRobot();
        int numRings = findNumRings(bmp);
        bmp.recycle();
        controlIndexServo(1);
        dashboardTelemetry.addData("rings", numRings);
        dashboardTelemetry.update();
        telemetry.addData("rings", numRings);
        telemetry.update();
        runWithEncoders();
        int a = 0;
        controlBlocker(.4);

        while (true){
            //sleep(5000);
            //a++;
            //takePic();
            //numRings = findNumRings(bmp);
            //bmp.recycle();
            //dashboardTelemetry.addData("rings "+a, numRings);
            //dashboardTelemetry.update();
            //forward(.5, 1,0);
            //strafeLeft(.5,1,0);
            //backward(.5, 1,0);
            //strafeRight(.5,1,0);
        }


    }

}
