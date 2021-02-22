//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "Tele-Op2", group = "Taus")

public class Teleop2 extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = true;
    boolean isBPressed = false;
    boolean clawClosed = false;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean stopperDown = true;
    boolean dpadPressed = false;
    boolean leftStick = false;

    double shooterRpm = 2750;
    double powerShotRpm = 2500;
    double shooterPower = (shooterRpm*28)/60.0;
    double powerShotPower = (powerShotRpm*28)/60.0;
    double rpm;
    double oldRotations = 0;
    double multiplier = 1;
    double previousY = 0;
    double previousX = 0;

    double p = 5;
    double i = 0;
    double d = 0;
    double f = 0;

    //PIDFCoefficients pid = method.robot.shooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);


    @Override
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(method.magic8());
        telemetry.update();
        method.robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
        waitForStart();
        method.controlLaunchServo(1);
        method.robot.shooter.setVelocity(shooterPower);

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

            telemetry.addData("angle", (int)method.getHeading());
            telemetry.addData("target", (int)shooterRpm);
            telemetry.addData("current", (int)rpm);
            telemetry.addData("position", "[" +(int)method.currentXPosition + ", " + (int)method.currentYPosition + "]");
            telemetry.update();
            telemetry.clear();
        }

        method.setAllMotorsTo(0);
    }

    public void drive(){
        method.runWithEncoders();
        if (gamepad1.right_stick_button&&!leftStick){
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
        //method.robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(method.runtime2.seconds()>5){
            double newRotations = method.robot.shooter.getCurrentPosition()/28.0;
            double rotations = newRotations - oldRotations;
            double min = method.runtime2.seconds()/60;
            rpm = rotations/min;
            method.runtime2.reset();
            oldRotations = newRotations;
        }

        if(gamepad1.x && !isXPressed){
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
        if(gamepad1.dpad_up && !dpadPressed){
            shooterPower +=(50*28)/60.0;
            shooterRpm +=50;
            method.robot.shooter.setVelocity(shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_down && !dpadPressed){
            shooterPower -=(50*28)/60.0;
            shooterRpm-=50;
            method.robot.shooter.setVelocity(shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_right && !dpadPressed){
            shooterPower = (2750*28)/60.0;
            shooterRpm=2750;
            method.robot.shooter.setVelocity(shooterPower);
            dpadPressed=true;
        }

        if(!gamepad1.x){
            isXPressed = false;
        }
        if(!gamepad1.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right){
            dpadPressed = false;
        }
    }
    public void shoot(){
        if(gamepad1.right_trigger>.1) {
            telemetry.addLine(method.magic8());
            telemetry.update();
            method.shoot(32.5, shooterPower);
        }
    }
    public void powerShot(){
        if(gamepad1.left_trigger>.1) {
            telemetry.addLine(method.magic8());
            telemetry.update();
            sleep(3000);
            method.powerShot(0, -5, -10, powerShotPower, shooterPower);
        }
    }

    public void stopper(){
        if(gamepad1.a && !isAPressed){
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
        if(!gamepad1.a){
            isAPressed = false;
        }
    }
    public void intake(){
        method.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(gamepad1.right_bumper){
            method.setIntakePower(1);
        }
        else if(gamepad1.left_bumper){
            method.setIntakePower(-1);
        }
        else{
            method.setIntakePower(0);
        }
    }
    public void claw(){
        if((gamepad1.b && !isBPressed)){
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw

            }
            else{
                method.controlArmServo(1);//moving arm down
            }
            isRunning = true;
            method.runtime.reset();
        }
        if(!gamepad1.b){
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
        if (gamepad1.y) {
            method.resetAngle = method.getHeading() + method.resetAngle;
        }
    }
    public void goToPosition(){
        if(gamepad1.left_stick_button){
            method.goToPosition(32.5, method.currentXPosition, method.currentYPosition, 100, 100);
        }
    }
    public void updatePosition(){
        if (gamepad1.left_trigger>.1){
            method.stopAndResetEncoders();
        }

        double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

        double currentY = method.robot.backLeftMotor.getCurrentPosition()-rotation;
        double deltaY1 = previousY-currentY;

        double currentX = method.robot.backRightMotor.getCurrentPosition()+rotation;
        double deltaX1 = previousX-currentX;

        double theta = Math.atan2(deltaY1,deltaX1);
        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngle =  rotatedTheta+gyroAngle;

        double deltaY2 = Math.sin(calculationAngle) * deltaY1 + Math.sin(calculationAngle) * deltaX1;
        double deltaX2 = Math.cos(calculationAngle) * deltaY1;
        if((Math.cos(calculationAngle) * deltaY1>0&&Math.cos(calculationAngle) * deltaX1<0)||(Math.cos(calculationAngle) * deltaY1<0&&Math.cos(calculationAngle) * deltaX1>0)) {
            deltaY2 = Math.sin(calculationAngle) * deltaY1;
            deltaX2 = Math.cos(calculationAngle) * deltaY1 + Math.cos(calculationAngle) * deltaX1;
        }

        double DistY = (method.wheelCircumference) * ((deltaY2) / method.countsPerRotation);
        double DistX = (method.wheelCircumference) * ((deltaX2) / method.countsPerRotation);
        if(Math.abs(deltaY2)>30) {
            method.currentXPosition = DistX;
            previousY = deltaY1;
        }
        if(Math.abs(deltaX2)>30) {
            method.currentYPosition = DistY;
            previousX = deltaX1;
        }
    }
}
