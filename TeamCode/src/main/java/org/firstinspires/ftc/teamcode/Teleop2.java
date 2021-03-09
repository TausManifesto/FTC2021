//Tank Drive


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Single driver", group = "Taus")
@Config
public class Teleop2 extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = true;
    boolean isBPressed = false;
    boolean clawClosed = false;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean dpadPressed = false;
    boolean leftStick = false;

    double multiplier = 1;
    double previousY = 0;
    double previousX = 0;


    @Override
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(method.magic8());
        telemetry.update();
        waitForStart();
        method.robot.shooter.setVelocityPIDFCoefficients(method.p,method.i,method.d,method.f);
        method.setShooterPower(method.shooterPower);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addLine("starting");
        dashboardTelemetry.update();


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
            telemetry.addData("target", (int)method.shooterRpm);
            telemetry.addData("current", (int)(method.robot.shooter.getVelocity()/28.0)*60);
           telemetry.addData("position", "[" +(int)method.currentXPosition + ", " + (int)method.currentYPosition + "]");

            dashboardTelemetry.addData("current", (int)(method.robot.shooter.getVelocity()/28.0)*60);
            dashboardTelemetry.addData("target", (int)method.shooterRpm);
            dashboardTelemetry.addData("0", 0);
            dashboardTelemetry.update();

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
        double rotationValue = 0;
        double stickX = 0;
        double stickY = 0;
        if(Math.abs(gamepad1.right_stick_x)>.05) {
            rotationValue = gamepad1.right_stick_x;
        }
        if(Math.abs(gamepad1.left_stick_x)>.05) {
            stickX = gamepad1.left_stick_x;
        }
        if(Math.abs(gamepad1.left_stick_y)>.05) {
            stickY = -gamepad1.left_stick_y;
        }
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

        if(gamepad1.x && !isXPressed){
            isXPressed = true;
            if (!shooterOn) {
                method.setShooterPower(method.shooterPower);
              shooterOn = true;
            }
            else{
             method.setShooterPower(0);
             shooterOn = false;
            }
        }
        if(gamepad1.dpad_up && !dpadPressed){
            method.shooterRpm +=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_down && !dpadPressed){
            method.shooterRpm-=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_right && !dpadPressed){
            method.shooterRpm=method.staticShooterRpm;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }

        if(!gamepad1.x){
            isXPressed = false;
        }
        if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right){
            dpadPressed = false;
        }
    }
    public void shoot(){
        if(gamepad1.right_trigger>.1) {
            telemetry.addLine(method.magic8());
            telemetry.update();
            method.shoot(0, method.shooterPower);
        }
    }
    public void powerShot(){
        if(gamepad1.left_trigger>.1) {
            telemetry.addLine(method.magic8());
            telemetry.update();
            method.powerShot(10, 16, 20, method.powerShotPower, method.shooterPower);
        }
    }

    public void stopper(){
        if(gamepad1.a && !isAPressed){
            isAPressed = true;
            method.shootRings(1);
        }
        if(!gamepad1.a){
            isAPressed = false;
        }
    }
    public void intake(){
        method.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        method.robot.intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (gamepad1.dpad_left){
            method.stopAndResetEncoders();
            method.currentXPosition = 0;
            method.currentYPosition = 135;
            method.resetAngle = method.getHeading() + method.resetAngle;
        }

        double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

        double currentY = method.robot.backLeftMotor.getCurrentPosition()-rotation;
        double deltaY1 = currentY-previousY;
        //telemetry.addData("Delta y1", deltaY1);

        double currentX = method.robot.backRightMotor.getCurrentPosition()+rotation;
        double deltaX1 = currentX-previousX;
        //telemetry.addData("Delta x1", deltaX1);

        double theta = Math.atan2(deltaY1,deltaX1);
        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngle =  rotatedTheta+gyroAngle;

        double deltaY2 = Math.sin(calculationAngle) * Math.abs(deltaY1);// + Math.sin(calculationAngle) * Math.abs(deltaX1);
        double deltaX2 = Math.cos(calculationAngle) * Math.abs(deltaY1);

        if((Math.cos(calculationAngle) * Math.abs(deltaY1)>0&&Math.cos(calculationAngle) * Math.abs(deltaX1)<0)||(Math.cos(calculationAngle) * Math.abs(deltaY1)<0&&Math.cos(calculationAngle) * Math.abs(deltaX1)>0)) {
            //deltaY2 = Math.sin(calculationAngle) * Math.abs(deltaY1);
            deltaX2 = Math.cos(calculationAngle) * Math.abs(deltaY1) + Math.cos(calculationAngle) * Math.abs(deltaX1);
        }
        //telemetry.addData("Delta y2", deltaY2);

        //telemetry.addData("Delta y2", deltaY2);

        double DistY = (method.wheelCircumference) * ((deltaY2) / method.countsPerRotation);
        //telemetry.addData("Delta y", DistY);

        double DistX = (method.wheelCircumference) * ((deltaX2) / method.countsPerRotation);
        //telemetry.addData("Delta x", DistX);

        if(Math.abs(deltaY2)>30 || Math.abs(deltaX2)>30) {
            method.currentYPosition += DistY;
            previousY = currentY;

            if(method.currentYPosition<0){
                method.currentYPosition = 0;
            }

            else if (method.currentYPosition>135){
                method.currentYPosition = 135;
            }

            method.currentXPosition += DistX;
            previousX = currentX;

            if(method.currentXPosition<0){
                method.currentXPosition = 0;
            }

            else if (method.currentXPosition>88){
                method.currentXPosition = 88;
            }
        }
    }
}
