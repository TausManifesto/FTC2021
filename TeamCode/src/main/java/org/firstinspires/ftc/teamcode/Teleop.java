//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Tele-Op", group = "Taus")

public class Teleop extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = false;
    boolean isBPressed = false;
    boolean clawClosed = false;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean stopperDown = true;
    boolean dpadPressed = false;
    boolean leftStick = false;

    double shooterPower = .585;
    double rpm;
    double oldRotations = 0;
    double multiplier = 1;

    @Override
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(magic8());
        telemetry.update();
        telemetry.update();
        waitForStart();
        method.controlLaunchServo(1);
        method.setShooterPower(shooterPower);

        while (opModeIsActive()) {
            drive();

            shooter();
            intake();
            claw();
            stopper();

            shoot();
            powerShot();

            goToPosition();
            updatePosition();
            resetAngle();

            telemetry.addData("angle", method.getHeading());
            telemetry.addData("shooter", shooterPower);
            telemetry.addData("rpm", rpm/1);
            telemetry.addData("position", "[" +method.currentXPosition + ", " + method.currentYPosition + "]");
            telemetry.update();
            telemetry.clear();
        }

        method.setAllMotorsTo(0);
    }

    public void drive(){
        method.runWithEncoders();
        if (gamepad1.left_stick_button&&!leftStick){
            if (multiplier==1){
                multiplier=2;
            }
            else {
                multiplier=1;
            }
            leftStick = true;
        }
        if(!gamepad1.left_stick_button){
            leftStick = false;
        }
        double scaleFactor = 1;
        double rotationValue = gamepad1.right_stick_x;
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        //Robot Centric
        //gyroAngle = Math.PI / 2;

        //inverse tangent of game-pad stick y/ game-pad stick x = angle of joystick
        double joystickAngle = Math.atan2(stickY, stickX);
        double theta =  joystickAngle+gyroAngle;

        //changing from a [+] with -- being y and | being x to an [X] with \ being y and / being x (left is forward)
        double calculationAngle = theta - ((3*Math.PI) / 4);

        //magnitude of movement using pythagorean theorem
        double magnitude = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
        double xComponent = magnitude * (Math.cos(calculationAngle));
        double yComponent = magnitude * (Math.sin(calculationAngle));

        //creates scaleFactor to make sure movement+turning doesn't exceed power 1
        if (yComponent - rotationValue > 1) {
            scaleFactor = Math.abs(yComponent - rotationValue);
        }
        if (yComponent + rotationValue > 1 && yComponent + rotationValue > scaleFactor) {
            scaleFactor = Math.abs(yComponent + rotationValue);
        }
        method.robot.frontLeftMotor.setPower(((xComponent + rotationValue) / scaleFactor)/multiplier);
        method.robot.backLeftMotor.setPower(((yComponent + rotationValue) / scaleFactor)/multiplier);//y
        method.robot.backRightMotor.setPower(((xComponent - rotationValue) / scaleFactor)/multiplier);//x
        method.robot.frontRightMotor.setPower(((yComponent - rotationValue) / scaleFactor)/multiplier);
    }

    public void shooter(){
        method.robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(method.runtime2.seconds()>1){
            double newRotations = method.robot.shooter.getCurrentPosition()/28.0;
            double rotations = newRotations - oldRotations;
            double min = method.runtime2.seconds()/60;
            rpm = rotations/min;
            method.runtime2.reset();
            oldRotations = newRotations;
        }

        if(gamepad2.x && !isXPressed){
            isXPressed = true;
            if (!shooterOn) {
              method.setShooterPower(shooterPower);
              shooterOn = true;
            }
            else{
             method.setShooterPower(0);
             shooterOn = false;
            }
        }
        if(gamepad2.dpad_up && !dpadPressed){
            shooterPower +=.01;
            method.setShooterPower(shooterPower);
            dpadPressed=true;
        }
        else if(gamepad2.dpad_down && !dpadPressed){
            shooterPower -=.01;
            method.setShooterPower(shooterPower);
            dpadPressed=true;
        }
        else if(gamepad2.dpad_right && !dpadPressed){
            shooterPower = .585;
            method.setShooterPower(shooterPower);
            dpadPressed=true;
        }

        if(!gamepad2.x){
            isXPressed = false;
        }
        if(!gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right){
            dpadPressed = false;
        }
    }
    public void shoot(){
        if(gamepad2.right_trigger>.1) {
            telemetry.addLine(magic8());
            telemetry.update();
            telemetry.update();
            method.shoot(30, shooterPower);
        }
    }
    public void powerShot(){
        if(gamepad2.left_trigger>.1) {
            telemetry.addLine(magic8());
            telemetry.update();
            method.powerShot(10, 16, 23, .46, shooterPower);
        }
    }

    public void stopper(){
        if(gamepad2.a && !isAPressed){
            isAPressed = true;
            if (!stopperDown) {
                method.controlLaunchServo(1);
                stopperDown = true;
            }
            else{
                method.controlLaunchServo(0);
                stopperDown = false;
            }
        }
        if(!gamepad2.a){
            isAPressed = false;
        }
    }
    public void intake(){
        method.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        method.setIntakePower(-gamepad2.right_stick_y);
    }
    public void claw(){
        if((gamepad2.b && !isBPressed)){
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw
                isRunning = true;
                method.runtime.reset();

            }
            else{
                method.controlArmServo(1);//moving arm down
                isRunning = true;
                method.runtime.reset();
            }
        }
        if(!gamepad2.b){
            isBPressed = false;
        }
        if (isRunning){
            if (!clawClosed){
                if (method.runtime.seconds() > .5) {
                    method.controlArmServo(0);//move arm up
                    clawClosed = true;
                    isRunning = false;
                }

            }
            else{
                if (method.runtime.seconds() > .5) {
                    method.controlClawServo(.7);//opening claw
                    clawClosed = false;
                    isRunning = false;
                }
            }
        }
    }

    public void resetAngle() {
        if (gamepad1.right_trigger>.1) {
            method.resetAngle = method.getHeading() + method.resetAngle;
        }
    }
    public void goToPosition(){

    }
    public void updatePosition(){
        if (gamepad1.left_trigger>.1){
            method.stopAndResetEncoders();
        }

        double rotation = (method.robot.backLeftMotor.getCurrentPosition()-method.robot.frontRightMotor.getCurrentPosition())/2.0;
        telemetry.addData("rot", rotation);
        double DistY = (method.wheelCircumference) * ((method.robot.backLeftMotor.getCurrentPosition()-rotation)/method.countsPerRotation);
        double DistX = (method.wheelCircumference) * ((method.robot.backRightMotor.getCurrentPosition()+rotation)/method.countsPerRotation);
        method.currentXPosition = (DistX-DistY)/Math.sqrt(2);
        method.currentYPosition = (DistY+DistX)/Math.sqrt(2);
    }

    public String magic8() {
        telemetry.addLine("  ._-------_. ");
        telemetry.addLine(" /     _     | ");
        telemetry.addLine(" |    (8)    | ");
        telemetry.addLine(" |     ^     / ");
        telemetry.addLine("  '-.......-'");
        int magic8 = (int)(Math.random()*(19)+1);
        switch(magic8){
            case 1:
                return "As I see it, yes.";
            case 2:
                return "Ask again later.";
            case 3:
                return "Better not tell you now";
            case 4:
                return "Cannot predict now.";
            case 5:
                return "Concentrate and ask again.";
            case 6:
                return "Don’t count on it.";
            case 7:
                return "It is certain.";
            case 8:
                return "It is decidedly so.";
            case 9:
                return "Most likely.";
            case 10:
                return "My reply is no.";
            case 11:
                return "My sources say no.";
            case 12:
                return "Outlook not so good.";
            case 13:
                return "Outlook good.";
            case 14:
                return "Reply hazy, try again.";
            case 15:
                return "Signs point to yes.";
            case 16:
                return "Very doubtful.";
            case 17:
                return "Without a doubt.";
            case 18:
                return "Yes.";
            case 19:
                return "You may rely on it.";
            case 20:
                return "Yes – definitely.";
        }
        return "error";
    }
}
