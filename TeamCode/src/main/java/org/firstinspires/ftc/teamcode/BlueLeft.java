
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto", group = "Taus")
public class BlueLeft extends AutonomousMethods {

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

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.75);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(shooterPower);
        forward(.5, 2.5, startOffset);
        shoot(-18, shooterPower);
        toAngle(0, .5);

        switch (numberOfRings){
            case 0:
                setShooterPower(0);

                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  .5, 6);//forward +6
                strafeLeft(1, 24);//left
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.5, 12);//right
                controlArmServo(1);//down
                backward(.5, 2.5,7);//back
                //arm down
                pickUpWobbleGoal(3);

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 2,11);//forward
                strafeLeft(.5, 6);
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 1:
                //picking up rings
                setIntakePower(1);
                toAngle(25, .5);
                backward(.5, 1, 0);
                forward(.5, 1, 0);
                setIntakePower(0);

                //shooting 2nd time
                toAngle(-18, .5);
                shootRings(1);
                setShooterPower(0);
                toAngle(0, .5);

                //Dropping 1st Wobble Goal
                //move to square
                forward(.5, 1.5,6);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                controlArmServo(1);//down
                backward(.5, 3.5,7);
                //arm down
                pickUpWobbleGoal(9);

                //Dropping 2nd Wobble goal
                //forward
                strafeRight(.5, 6);//right
                forward(.5, 2.5,0);
                toAngle(90, 1);
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 4:
                //picking up rings
                setIntakePower(1);
                toAngle(25, .5);
                backward(.3, 1, 4);
                forward(.5, 1, 4);
                setIntakePower(0);

                //shooting 2nd time
                shoot(-18, shooterPower);
                setShooterPower(0);
                toAngle(0, .5);

                //Dropping 1st Wobble goal
                //move to square
                forward(1,  2.5,0);//forward
                strafeLeft(.5, 24);//left
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.5, 18);//right
                controlArmServo(1);//down
                setIntakePower(1);
                backward(1, 4.5,0);//back
                setIntakePower(0);
                pickUpWobbleGoal(3);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 4,2);//forward
                //strafeLeft(.5, 9);//left
                //drop wobble goal
                dropWobbleGoal();

                //park
                backward(1,2,0);//back
                break;
        }

    }
}
