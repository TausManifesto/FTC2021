
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedLeft", group = "Taus")
public class RedLeft extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.75);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        forward(.5, 2.5, startOffset);//3
        powerShot(-5.5, -1, 3.5, powerShotPower, shooterPower);
        toAngle(0, .5);

        switch (numberOfRings){
            case 0:
                setShooterPower(0);

                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  .5, 10);//forward +10
                strafeLeft(.5, 0, 16);//left 15
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.3, 0,14);//right 15
                toAngle(0, .1);
                controlArmServo(1);//down
                backward(.5, 2.5,6);//back 5
                //arm down
                pickUpWobbleGoal(6);//4

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 2,10);//forward 6
                strafeLeft(.5, 0,4);//1
                //drop wobble goal
                dropWobbleGoal();

                //park
                //strafeRight(.5, 48+4);
                break;
            case 1:
                //Dropping 1st Wobble Goal
                //move to square
                forward(1, 1.5,10);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up rings
                //move back
                backward(1, .5, 10);
                strafeLeft(.5, 0,14);
                toAngle(0, .2);
                controlArmServo(1);//down
                setIntakePower(1);
                backward(1, 2,2);

                //Picking up 2nd wobble goal
                toAngle(-50, .5);
                strafeLeft(.5, 0,4);
                //strafeRight(.5, 6);
                //backward(1, .5,8);
                //strafeLeft(.5, 6);
                //arm down
                pickUpWobbleGoal(0);
                toAngle(0, .5);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 1.5, 0);
                shoot(-5, shooterPower);
                forward(1, .5,0);
                toAngle(90, 1);
                //drop wobble goal
                dropWobbleGoal();

                //park
                strafeRight(1, 0,2);
                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                forward(1,  2.5,0);//forward
                strafeLeft(.5, 0,14);//left
                //drop wobble goal
                dropWobbleGoal();

                //Picking up rings
                setShooterPower((2100*28)/60.0);
                setIntakePower(.75);
                toAngle(0 ,.5);
                controlArmServo(1);//down
                backward(.5, 3.5, 0);

                //shooting 2nd time
                shoot(-10, (2100*28)/60.0);
                setShooterPower((2150*28)/60.0);
                toAngle(0, 5);
                setIntakePower(1);

                //picking up rings
                backward(.5, 0, 2);

                //shooting 3rd time
                shoot(-10, (2150*28)/60.0);

                //Picking up second wobble goal
                toAngle(-50, .5);
                strafeLeft(.5, 0,4);
                pickUpWobbleGoal(0);
                toAngle(0, .5);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 3.5, 2);
                strafeLeft(1, 0,6);
                dropWobbleGoal();

                //park
                backward(1,2,0);//back
                break;
        }

    }
}
