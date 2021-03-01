//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedRight", group = "Taus")
public class RedRight extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();
        magic8();

        waitForStart();

        stopAndResetEncoders();

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.35);//up

        //shoot
        setIntakePower(.5);
        setShooterPower(shooterPower);
        forward(1, 2.5,startOffset);
        setIntakePower(0);
        shoot(30, shooterPower);
        setShooterPower(0);
        toAngle(0, .3);

        switch (numberOfRings){
            case 0:
                //Dropping 1st Wobble goal
                //move to square
                forward(1,  0, 4);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(.5, 12);//left
                backward(1, 2,8);//back
                //arm down
                pickUpWobbleGoal(6);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 2,16);//forward
                strafeRight(.5, 6);//right
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 1:
                //Dropping 1st Wobble Goal
                //move to square
                forward(1, 1,4);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                backward(1, 3,8);
                //arm down
                pickUpWobbleGoal(12);

                //Dropping 2nd Wobble goal
                //forward
                strafeLeft(.5, 18);
                forward(1, 3,16);
                //drop wobble goal
                dropWobbleGoal();

                //park
                backward(1,1, 0);
                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                forward(1,  2,4);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeLeft(.5, 18);//left
                toAngle(0, .3);
                backward(1, 4,8);//back
                toAngle(0, .3);
                //arm down
                controlArmServo(1);//down
                sleep(500);
                strafeRight(.5, 12);
                controlClawServo(.25);//close
                sleep(500);
                controlArmServo(0);//up

                //Dropping 2nd Wobble goal
                //forward
                toAngle(0, .3);
                strafeLeft(1, 12);
                forward(1, 4,16);//forward
                strafeRight(.5, 18);//right
                //drop wobble goal
                dropWobbleGoal();

                //park
                backward(1,2,0);//back
                break;
        }

    }
}
