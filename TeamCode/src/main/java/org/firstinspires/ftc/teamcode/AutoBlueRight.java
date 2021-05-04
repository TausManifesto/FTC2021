
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Taus")
public class AutoBlueRight extends AutonomousMethods {

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
        controlBlocker(0);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.75);//down


        forward(.5, 2.5, 3);
        if(numberOfRings==0) {
            sleep(3000);
            shoot(-22, shooterPower);
            //powerShot(-5.5, -1, 3.5, powerShotPower, shooterPower);
            toAngle(0, .5);
            setShooterPower(0);
        }

        switch (numberOfRings){
            case 0:
                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  .5, 8);//forward 22
                strafeLeft(.3, .5, 6);//left
                //drop wobble goal
                dropWobbleGoal();
                toAngle(0, .5);

                //Picking up 2nd Wobble goal
                //move back
                //strafeRight(.3, .5, 6);//right
                //toAngle(0, .1);
                backward(.5, 2,0);//back
                controlArmServo(1);//down
                //arm down
                toAngle(-60, .5);
                pickUpWobbleGoal(6);
                sleep(500);
                toAngle(0, .5);

                //Dropping 2nd Wobble goal
                //forward
                strafeRight(.5, 0,4);
                forward(.5, 1,20);//forward
                dropWobbleGoal();
                strafeRight(.5, 1,0);//left
                //drop wobble goal


                //park
                break;
            case 1:
                toAngle(-20, 1);
                setIntakePower(1);
                sleep(2000);
                shootRings(2);
                setIntakePower(0);
                toAngle(0, 1);

                //Dropping 1st Wobble Goal
                //move to square
                forward(.5, 1,18);
                toAngle(10, 1);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up rings
                //move back
                setIntakePower(1);
                backward(.5, 3, 3);

                //Picking up 2nd wobble goal
                controlArmServo(1);//down
                toAngle(-50, 1);
                pickUpWobbleGoal(6);


                //Dropping 2nd Wobble goal
                //forward
                toAngle(0, 1);
                forward(.5, 1, 11);
                toAngle(-7.5, 1);
                shootRings(2);
                toAngle(0, 1);
                forward(.5, 0,18);
                toAngle(120, 1);
                //drop wobble goal
                dropWobbleGoal();

                //park
                toAngle(0, 1);


                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                controlArmServo(.5);//down
                strafeLeft(.5, 0,24);//left
                shoot(0, shooterPower);
                //toAngle(0, 1);
                forward(.6,  2,10);//forward
                //drop wobble goal
                //dropWobbleGoal();

                //Picking up rings
                setShooterPower((2150*28)/60.0);
                setIntakePower(1);
                toAngle(-10 ,1);
                controlClawServo(.7);//open
                controlArmServo(1);//down
                backward(.5, 3, 0);
                backward(.5, 0, 4);
                forward(.5, 0, 2);
                backward(.5, 0, 4);
                //forward(.5, 0, 2);
                //backward(.5, 0, 10);

                //shooting 2nd time
                toAngle(-12, 1);
                shootRings(2);
                setIntakePower(1);
                setShooterPower((2250*28)/60.0);
                toAngle(-8, 1);

                //picking up rings
                backward(.5, 0, 14);

                //Picking up second wobble goal
                toAngle(-45, 1);
                setIntakePower(1);
                pickUpWobbleGoal(6);

                //shooting 3rd time

                toAngle(-10,1);
                shootRings(2);
                toAngle(0, 1);

                //Dropping 2nd Wobble goal
                //forward
                forward(.7, 4, 0);
                controlClawServo(.7);//open
                //controlArmServo(0);//up
                sleep(250);
                //park
                //backward(.7,1,18);//back
                break;
        }

    }
}
