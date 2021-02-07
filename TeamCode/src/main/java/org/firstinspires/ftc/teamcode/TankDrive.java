//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TankDrive", group = "Taus")

public class TankDrive extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = false;
    boolean isBPressed = false;
    boolean clawClosed = false;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean stopperDown = true;
    boolean dpadPressed = false;

    double shooterpower = .51;
    double targetrpm = 2000;
    double rpm;

    @Override
    public void runOpMode() {

        method.robot.initializeHardware(hardwareMap);
        method.runWithouthEncoders();
        //Opposite direction?
        method.robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        method.robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        method.robot.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        method.robot.backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("most definitely");
        telemetry.update();
        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();
        method.controlLaunchServo(1);
        method.setShooterPower(shooterpower);

        while (opModeIsActive()) {
            drive1();
            shooter();
            intake();
            claw();
            stopper();
            resetAngle();
            shoot();
            powerShot();
            //adjustShooter();
            //goToPosition();
            //updatePosition();

            telemetry.addLine("owen is cool");

            telemetry.addData("angle", method.getHeading());
            telemetry.addData("shooter", shooterpower);
            telemetry.addData("rpm", rpm);
            telemetry.update();
            telemetry.clear();
        }

        method.setAllMotorsTo(0);
    }

    public void drive1(){
        double scaleFactor = 1;
        double rotationValue = gamepad1.right_stick_x;
        double stickX = gamepad1.left_stick_x;
        double stickY = gamepad1.left_stick_y;
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle > Math.PI/2) {
            gyroAngle = gyroAngle + (5*Math.PI / 2);
        }
        else{
            gyroAngle = gyroAngle+Math.PI/2;
        }
        //Robot Centric
        //gyroAngle = Math.PI / 2;
        //Linear directions in case you want to do straight lines.
        if (gamepad1.dpad_right) {
            stickX = .3;
        }
        else if (gamepad1.dpad_left) {
            stickX = -.3;
        }
        if (gamepad1.dpad_up) {
            stickY = .3;
        }
        else if (gamepad1.dpad_down) {
            stickY = -.3;
        }
        //MOVEMENT for rotation
        //inverse tangent of gamepad stick y/ gamepad stick x = angle of joystick
        double joystickAngle = Math.atan2(stickY, stickX);
        double theta =  joystickAngle-gyroAngle; //fix?
        //theta + pi/4 because wheels apply power at 45 degree angle
        double calculationAngle = theta - (Math.PI / 4);//fix?
        //magnatude of movement using pythagorean theorem
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
        method.robot.frontLeftMotor.setPower((yComponent + rotationValue) / scaleFactor);
        method.robot.backLeftMotor.setPower((xComponent + rotationValue) / scaleFactor);
        method.robot.backRightMotor.setPower((yComponent - rotationValue) / scaleFactor);
        method.robot.frontRightMotor.setPower((xComponent - rotationValue) / scaleFactor);
    }
    public void shooter(){
        if(method.runtime.seconds()>1){
            double rotations = method.robot.shooter.getCurrentPosition()/28.0;
            double min = method.runtime.seconds()/60;
            rpm = rotations/min;
        }
        method.runtime.reset();

        if(gamepad2.a && !isAPressed){
            isAPressed = true;
            if (!shooterOn) {
              method.setShooterPower(shooterpower);
              shooterOn = true;
            }
            else{
             method.setShooterPower(0);
             shooterOn = false;
            }
        }
        if(gamepad2.dpad_up && !dpadPressed){
            shooterpower +=.01;
            method.setShooterPower(.45);
            dpadPressed=true;
        }
        else if(gamepad2.dpad_down && !dpadPressed){
            shooterpower -=.01;
            method.setShooterPower(.45);
            dpadPressed=true;
        }
        else if(gamepad2.dpad_right && !dpadPressed){
            shooterpower = .55;
            method.setShooterPower(.45);
            dpadPressed=true;
        }

        if(!gamepad2.a){
            isAPressed = false;
        }
        if(!gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right){
            dpadPressed = false;
        }
    }
    public void adjustShooter(){
        if(Math.abs(shooterpower-method.robot.shooter.getPower())>.005) {
            shooterpower += method.shooterPowerIncrement(rpm, targetrpm, 100, .1);
            method.setShooterPower(shooterpower);
        }
    }
    public void stopper(){
        if(gamepad2.a && !isXPressed){
            isXPressed = true;
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
            isXPressed = false;
        }
    }
    public void intake(){
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
    public void shoot(){
        if(gamepad2.right_trigger>.1) {
            telemetry.addLine("actually three");
            telemetry.update();
            method.shoot(-23, shooterpower, false);
        }
    }
    public void powerShot(){
        if(gamepad2.left_trigger>.1) {
            telemetry.addLine("actually three");
            telemetry.update();
            method.powerShot(-28, -25, -18, .5, .5, .475, shooterpower);
        }
    }
    public void resetAngle() {
        if (gamepad1.right_trigger>.1) {
            method.resetAngle = method.getHeading() + method.resetAngle;
        }
    }
    public void goToPosition(){
        if (gamepad1.left_trigger>.1){
            method.goToPosition(1, 100, 100, 90);
        }
    }
    public void updatePosition(){
    }
}








