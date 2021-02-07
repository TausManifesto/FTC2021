//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedLeft", group = "Tau")
public class RedLeft extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        int numberOfRings = 0;
        double a = 24;//inches/square

        //initializing robot
        initializeRobot();
        //detect number of rings
        numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//move up

        //shoot
        setIntakePower(1);
        setShooterPower(.51);
        forward(.75, a*2.5);
        shoot(-27, .51, true);
        toAngle(.2, 0);

        switch (numberOfRings){
            case 0:
                //1st Wobble goal
                //move to square
                forward(.75,  a*.6);
                strafeRight(.5, a*.75);//move right half a square
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(500);
                controlArmServo(0);//move up

                //2nd Wobble goal
                //move back
                strafeLeft(.5, a*.4);
                backward(.5, a*2.6);
                //arm down
                controlArmServo(1);//move down
                sleep(1000);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//move up
                //forward
                forward(.75, a*2.5);
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(1000);
                controlArmServo(0);//move up
                sleep(1000);
                break;
            case 1:
                //1st Wobble Goal
                //move to square
                forward(.75, a*1.5);
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(500);
                controlArmServo(0);//move up

                //2nd Wobble goal
                //move back
                strafeRight(.5, a*.35);
                backward(.5, a*3.6);
                //arm down
                controlArmServo(1);//move down
                sleep(1000);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//move up
                //forward
                forward(.75, a*3.5);
                strafeLeft(.5, a*.35);
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(1000);
                controlArmServo(0);//move up
                sleep(1000);

                //park
                backward(.5,1);
                break;
            case 4:
                //1st Wobble Goal
                //move to square
                forward(.75, a*2.6);
                strafeRight(.5, a*.75);//move right half a square
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(500);
                controlArmServo(0);//move up

                //2nd Wobble goal
                //move back
                strafeLeft(.5, a*.4);
                backward(.5, a*4.6);
                //arm down
                controlArmServo(1);//move down
                sleep(1000);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//move up
                //forward
                forward(.75, a*4.5);
                //drop wobble goal
                controlArmServo(1);//move down
                controlClawServo(.7);//open
                sleep(1000);
                controlArmServo(0);//move up
                sleep(1000);

                //park
                backward(.5,2);
                break;
        }

    }
}
