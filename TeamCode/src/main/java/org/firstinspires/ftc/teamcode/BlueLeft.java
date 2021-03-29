
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
        controlIndexServo(1);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.75);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(shooterPower);

        forward(.5, 2.5, startOffset);
        if(numberOfRings!=1) {
            shoot(-18, shooterPower);
        }
        else{
            setIntakePower(1);
            toAngle(-18, .5);
            shootRings(1);
            setIntakePower(0);
        }

        switch (numberOfRings){
            case 0:
                toAngle(0, .5);
                setShooterPower(0);

                //Dropping 1st Wobble goal
                //move to square
                forward(.5,  .5, 6);//forward +6
                strafeLeft(.3, 16);//left
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                strafeRight(.3, 14);//right
                toAngle(0, .1);
                controlArmServo(1);//down
                backward(.5, 2.5,2);//back
                //arm down
                pickUpWobbleGoal(6);

                //Dropping 2nd Wobble goal
                //forward
                forward(.5, 2,6);//forward
                strafeLeft(.3, 4);
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 1:
                //picking up rings
//                setIntakePower(1);
//                toAngle(25, .5);
//                backward(.5, 1, 0);
//                forward(.5, 1, 0);

//                //shooting 2nd time
//                shoot(-18, shooterPower);
                toAngle(0, .5);

                //Dropping 1st Wobble Goal
                //move to square
                forward(.5, 1.5,6);
                //drop wobble goal
                dropWobbleGoal();

                //Picking up 2nd Wobble goal
                //move back
                backward(.5, .5, 6);
                strafeLeft(.5, 14);
                toAngle(0, .2);//-2.5
                controlArmServo(1);//down
                setIntakePower(1);//N
                backward(.75, 2,6);
                //toAngle(-2, .2);
                strafeRight(.5, 5);
                backward(.5, .5,2);
                strafeLeft(.75, 5);
                //arm down
                //controlArmServo(1);//down N
                //sleep(1000);//N
                pickUpWobbleGoal(0);
                toAngle(0, .2);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 2, 0);
                shoot(-10, shooterPower);
                //toAngle(0, .5);
                forward(1, .5,0);//2.5
                toAngle(90, 1);
                //drop wobble goal
                dropWobbleGoal();

                //park
                sleep(1000);
                break;
            case 4:
                //picking up rings
                setShooterPower(shooterPower);
                setIntakePower(.75);
                //toAngle(25, .5);
                //backward(.5, 1, 0);
                //forward(.5, 1, 0);
                toAngle(0 ,.5);
                strafeLeft(.5, 10);
                backward(.15, .5, 8);
                forward(.5, .5, 8);

                //shooting 2nd time
                shoot(-10, shooterPower);

                toAngle(0, .5);

                //Dropping 1st Wobble goal
                //move to square
                //forward(1,  2.5,0);//forward
                //strafeLeft(.5, 24);//left
                //drop wobble goal

                setIntakePower(1);

                backward(1, 1.5,0);//back//4.5

                //Picking up 2nd Wobble goal
                //move back
                //strafeRight(.5, 12);//right
                //toAngle(0, .1);
                //controlArmServo(1);//down


                //arm down
                //controlArmServo(1);//down N
                //sleep(750);//N
                //pickUpWobbleGoal(0);

                //Dropping 2nd Wobble goal
                //forward
                forward(1, 1.5, 0);//2
                toAngle(-10, .5);
                shootRings(2);
                toAngle(0, .5);
                setIntakePower(0);
                forward(1, 2.5, 4);
                strafeLeft(1, 3);
                dropWobbleGoal();
                //backward(.5, 2.5, 0);
                //forward(.75, 2,2);//forward
                //strafeLeft(.5, 6);
                //strafeLeft(.5, 9);//left
                //drop wobble goal
                //dropWobbleGoal();

                //park
                //backward(1,2,0);//back
                break;
        }

    }
}
