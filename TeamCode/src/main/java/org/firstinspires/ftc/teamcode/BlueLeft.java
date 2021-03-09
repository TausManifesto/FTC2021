
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueLeft", group = "Taus")
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
        controlArmServo(1);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(shooterPower);
        forward(.5, 2.5, startOffset);
        shoot(-18, shooterPower);
        setShooterPower(0);
        toAngle(0, .5);

        switch (numberOfRings){
            case 0:
                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  0, 10);//forward
                strafeLeft(.5, 24);//right
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.5, 18);//left
                controlArmServo(1);//down
                backward(.5, 2,10);//back
                //arm down
                pickUpWobbleGoal(6);

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 2,20);//forward
                strafeLeft(.5, 12);//right
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 1:
                setShooterPower(0);
                //Dropping 1st Wobble Goal
                //move to square
                forward(1, 1,10);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                controlArmServo(1);//down
                backward(1, 3,10);
                //arm down
                pickUpWobbleGoal(12);

                //Dropping 2nd Wobble goal
                //forward
                strafeRight(.5, 18);
                forward(1, 3,22);
                //drop wobble goal
                dropWobbleGoal();

                //park
                backward(1,1, 0);
                break;
            case 4:
                //picking up rings
                toAngle(45, .5);
                backward(.5, .5, 0);
                forward(.5, .5, 0);

                //shooting 2nd time
                shoot(-18, shooterPower);
                setShooterPower(0);
                toAngle(0, .5);

                //Dropping 1st Wobble goal
                //move to square
                forward(1,  2,10);//forward
                strafeLeft(.5, 24);//right
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.5, 24);//left
                controlArmServo(1);//down
                backward(1, 4,10);//back
                pickUpWobbleGoal(12);

                //Dropping 2nd Wobble goal
                //forward
                strafeRight(.5, 12);
                forward(1, 2,6);//forward
                strafeLeft(.5, 12);//right 24
                //drop wobble goal
                dropWobbleGoal();

                //park
                backward(1,2,0);//back
                break;
        }

    }
}
