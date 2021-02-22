//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedLeft", group = "Taus")
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

        stopAndResetEncoders();

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.35);//up

        //shoot
        setIntakePower(1);
        setShooterPower(.585);
        forward(.5, a*(2.5)+3);
        setIntakePower(0);
        shoot(32.5, .585);
        setShooterPower(0);
        toAngle(0);

        switch (numberOfRings){
            case 0:
                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  (a*.5)-8);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(.5, 12);//left
                backward(.5, (a*2.5)-4);//back
                //arm down
                controlArmServo(1);//down
                sleep(500);
                strafeRight(.5, 6);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//up

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, (a*2.5)+4);//forward
                strafeRight(.5, 6);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //park
                sleep(1000);
                break;
            case 1:
                //Dropping 1st Wobble Goal
                //move to square
                forward(.5, (a*1.5)-8);
                //drop wobble goal
                controlArmServo(1);//move down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//move up

                //Picking up 2nd Wobble goal
                //move back
                backward(.5, (a*3.5)-4);
                //arm down
                controlArmServo(1);//move down
                strafeRight(.5, .5*a);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//move up

                //Dropping 2nd Wobble goal
                //forward
                strafeLeft(.5, .75*a);
                forward(.75, (a*3.5)+4);
                //drop wobble goal
                controlArmServo(1);//move down
                sleep(1000);
                controlClawServo(.7);//open
                controlArmServo(0);//move up
                sleep(500);

                //park
                backward(1,a*1);
                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                forward(.75,  (a*2.5)-8);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(.5, 18);//left
                toAngle(0);
                backward(.75, (a*4.5)-4);//back
                toAngle(0);
                //arm down
                controlArmServo(1);//down
                sleep(500);
                strafeRight(.5, 12);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//up

                //Dropping 2nd Wobble goal
                //forward
                toAngle(0);
                strafeLeft(1, 12);
                forward(1, (a*4.5)+4);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(1000);
                controlClawServo(.7);//open
                controlArmServo(0);//up
                sleep(500);

                //park
                backward(1,a*2);//back
                break;
        }

    }
}
