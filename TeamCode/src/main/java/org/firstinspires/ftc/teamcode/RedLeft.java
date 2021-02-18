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

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.35);//up

        //shoot
        setIntakePower(1);
        setShooterPower(.585);
        forward(.5, a*2.5);
        setIntakePower(0);
        shoot(30, .585);
        setShooterPower(0);
        toAngle(0);

        switch (numberOfRings){
            case 0:
                //Dropping 1st Wobble goal
                //move to square
                forward(1,  (a*.5)-4);//forward
                strafeRight(1, a*.75);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(1, a*.25);//left
                backward(1, (a*2.5)-6);//back
                //arm down
                controlArmServo(1);//down
                sleep(1000);
                strafeRight(1, 6);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//up

                //Dropping 2nd Wobble goal
                //forward
                forward(1, (a*2.5)+4);//forward
                strafeRight(1, 6);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //park
                strafeLeft(1, 6);//left
                break;
            case 1:
                //Dropping 1st Wobble Goal
                //move to square
                forward(1, (a*1.5)-4);
                //drop wobble goal
                controlArmServo(1);//move down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//move up

                //Picking up 2nd Wobble goal
                //move back
                backward(1, (a*3.5)-4);
                //arm down
                controlArmServo(1);//move down
                strafeRight(1, a*.5);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//move up

                //Dropping 2nd Wobble goal
                //forward
                strafeLeft(1, a*.5);
                forward(1, (a*3.5)+4);
                //drop wobble goal
                controlArmServo(1);//move down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//move up

                //park
                backward(.5,1);
                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                forward(1, (a*2.5)-4);//forward
                strafeRight(1, a*.75);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(1, a*.25);//left
                backward(1, (a*4.5)-4);//back
                //arm down
                controlArmServo(1);//down
                sleep(1000);
                strafeRight(1, 6);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//up

                //Dropping 2nd Wobble goal
                //forward
                forward(1, (a*4.5)+2);//forward
                strafeRight(1, 6);//right
                //drop wobble goal
                controlArmServo(1);//down
                sleep(500);
                controlClawServo(.7);//open
                controlArmServo(0);//up

                //park
                strafeLeft(1, 6);//left
                backward(1,a*2);//back
                break;
        }

    }
}
