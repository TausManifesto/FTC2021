
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
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(shooterPower);

        forward(.5, 2.5, 3);
        shoot(-20, shooterPower);
        //powerShot(-5.5, -1, 3.5, powerShotPower, shooterPower);
        toAngle(0, .5);
        sleep(100);

        switch (numberOfRings){
            case 0:
                setShooterPower(0);

                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  .5, 8);//forward
                strafeLeft(.5, .5, 6);//left
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.3, .5, 6);//right
                toAngle(0, .1);
                controlArmServo(1);//down
                backward(.5, 2.5,5);//back
                //arm down
                pickUpWobbleGoal(12);

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 2,10);//forward
                strafeLeft(.5, 0,6);//left
                //drop wobble goal
                dropWobbleGoal();

                //park
                strafeRight(.5, 0,4);//right
                break;
            case 1:
                //Dropping 1st Wobble Goal
                //move to square
                forward(.5, 1,22);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up rings
                //move back
                backward(.5, 1, 8);
                toAngle(15, .2);
                controlArmServo(1);//down
                setIntakePower(1);
                backward(.5, 1,20);

                //Picking up 2nd wobble goal
                toAngle(-50, .5);
                strafeLeft(.5, 0,4);
                //arm down
                pickUpWobbleGoal(0);
                toAngle(0, .5);

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 1, 9);
                shoot(-5, shooterPower);
                forward(.5, 0,20);
                toAngle(120, 1);
                //drop wobble goal
                dropWobbleGoal();

                //park
                strafeRight(1, 0,2);
                break;
            case 4:
                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  2.5,0);//forward
                strafeLeft(.5, .5,3);//left
                //drop wobble goal
                dropWobbleGoal();
                strafeRight(.5, 0,4);

                //Picking up rings
                setShooterPower((2200*28)/60.0);
                setIntakePower(.65);
                toAngle(0 ,.5);
                controlArmServo(1);//down
                backward(.5, 3, 0);
                backward(.2, 0, 10);

                //shooting 2nd time
                shoot(-10, (2225*28)/60.0);
                setShooterPower((2225*28)/60.0);
                toAngle(0, .5);
                setIntakePower(1);

                //picking up rings
                backward(.5, 0, 5);

                //shooting 3rd time
                shoot(-10, (2225*28)/60.0);

                //Picking up second wobble goal
                backward(.5, 0, 5);
                toAngle(-40, .5);
                strafeLeft(.5, 0,2);
                pickUpWobbleGoal(0);
                toAngle(0, .5);

                //Dropping 2nd Wobble goal
                //forward
                //forward(1, 3, 22);
                //dropWobbleGoal();

                //park
                //backward(1,2,0);//back
                break;
        }

    }
}
