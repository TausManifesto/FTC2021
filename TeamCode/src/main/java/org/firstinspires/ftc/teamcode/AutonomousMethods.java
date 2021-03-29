//Autonomous methods to be used in run programs

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

//for reading gyro


public class AutonomousMethods extends LinearOpMode {


    private static final String TAG = "Webcam: ";
    //How long we are to wait to be granted permission to use the camera before giving up. Here,we wait indefinitely
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    // State regarding our interaction with the camera
    public CameraManager cameraManager;
    public WebcamName cameraName;
    public Camera camera;
    public CameraCaptureSession cameraCaptureSession;
    //The queue into which all frames from the camera are placed as they become available. Frames which are not processed by the OpMode are automatically discarded. */
    public EvictingBlockingQueue<Bitmap> frameQueue;
    //State regarding where and how to save frames when the 'A' button is pressed.
    public int captureCounter = 0;
    public File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    //A utility object that indicates where the asynchronous callbacks from the camera infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}if you're curious): no knowledge of multi-threading is needed here.
    private Handler callbackHandler;
    public Bitmap bmp;

    public Hardware robot = new Hardware(true);
    private final double gearRatio = 1;
    private final double wheelDiameter = 3.78;
    public final double wheelCircumference = wheelDiameter*Math.PI;
    public final double encoderCounts = 383.6; //counts per one rotation of output shaft
    double countsPerRotation = encoderCounts*gearRatio;
    double distPerSquare = 24;//inches/square
    double robotLength = 18;
    double startOffset = 1.5;

    public double staticShooterRpm = 2025;
    public double shooterRpm = 2025;
    public double powerShotRpm = 1850;
    public double powerShotRpm2 = 1875;
    public double powerShotRpm3 = 1875;
    public double shooterPower = (shooterRpm*28)/60.0;
    public double powerShotPower = (powerShotRpm*28)/60.0;
    public double powerShotPower2 = (powerShotRpm2*28)/60.0;
    public double powerShotPower3 = (powerShotRpm3*28)/60.0;
    public static double p = 100;
    public static double i = 0;
    public static double d = 0;
    public static double f = 14.3;

    public double kFactor = 1.15;//1.09

    public double resetAngle = 0;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtime2 = new ElapsedTime();
    public ElapsedTime runtime3 = new ElapsedTime();
    public double currentXPosition = robotLength/2;
    public double currentYPosition = robotLength/2;
    double max;

    public void initializeRobot() {
        robot.initializeHardware(hardwareMap);
        resetAngle=getHeading();

        //initializing camera
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        if (!OpenCVLoader.initDebug()) {
            error("Error: Cannot load OpenCV Library");
        } else {
            telemetry.addLine("Loaded OpenCV");
            telemetry.update();
        }

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;
            telemetry.addLine(magic8());
            telemetry.update();
            waitForStart();
            bmp = frameQueue.poll();
            while(frameQueue.isEmpty()){
                idle();
            }



        }
        finally {
            closeCamera();
        }
    }

    //Basic
    //moving forward distance (inch) with power [0, 1]
    public void forward(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 48;
        runWithEncoders();
        int counts = (int) ((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = distance/8;
        while (robot.frontLeftMotor.getCurrentPosition()<counts) {
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);
            if (distanceGoneFl<rampUpScale){
                setAllMotorsTo(errorToPower(distanceGoneFl, rampUpScale, power, .1));
            }
            else {
                setAllMotorsTo(errorToPower(distance-distanceGoneFl, max, power, .1));
            }
            //if(distanceGoneBl>=distance){
            //    robot.backLeftMotor.setPower(0);
            //}
            //if(distanceGoneBr>=distance){
            //    robot.backRightMotor.setPower(0);
            //}
            //if(distanceGoneFl>=distance){
            //    robot.frontLeftMotor.setPower(0);
            //}
            //if(distanceGoneFr>=distance){
            //    robot.frontRightMotor.setPower(0);
            //}
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        sleep(250);
    }
    //moving backward distance (inch) with power [0, 1]
    public void backward(double power, double squares, double inches) {
        stopAndResetEncoders();
        double distance = squares*distPerSquare+inches;
        max = power* 48;
        runWithEncoders();
        int counts = (int) -((distance / wheelCircumference) * countsPerRotation);
        double rampUpScale = distance/8;
        while (robot.frontLeftMotor.getCurrentPosition()>counts) {
            double distanceGoneBl = wheelCircumference * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneBr = wheelCircumference * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFl = wheelCircumference * (robot.frontLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneFr = wheelCircumference * (robot.frontRightMotor.getCurrentPosition()/countsPerRotation);
            if (-distanceGoneFl<rampUpScale){
                setAllMotorsTo(-errorToPower(-distanceGoneFl, rampUpScale, power, .1));
            }
            else {
                setAllMotorsTo(-errorToPower(distance -(-distanceGoneFl), max, power, .1));
            }
            //if(distanceGoneBl<=-distance){
            //    robot.backLeftMotor.setPower(0);
            //}
            //if(distanceGoneBr<=-distance){
            //    robot.backRightMotor.setPower(0);
            //}
            //if(distanceGoneFl<=-distance){
            //    robot.frontLeftMotor.setPower(0);
            //}
            //if(distanceGoneFr<=-distance){
            //    robot.frontRightMotor.setPower(0);
            //}
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        sleep(250);
    }
    //strafing left distance (inch) with power [0, 1]
    public void strafeLeft(double power, double distance) {
        stopAndResetEncoders();
        max = power*24;
        runWithEncoders();
        int counts = (int) (((distance / wheelCircumference) * countsPerRotation)*kFactor);

        while (robot.frontLeftMotor.getCurrentPosition()>-counts) {
            double distanceGoneBl = wheelCircumference * ((robot.backLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneBr = wheelCircumference * ((robot.backRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFr = wheelCircumference * ((robot.frontRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);

            double powerRaw = errorToPower(distance - (-distanceGoneFl), max, power, .15);
            setPowerOfMotorsTo(powerRaw,-powerRaw, -powerRaw, powerRaw);

            //if(distanceGoneBl>=distance){
            //    robot.backLeftMotor.setPower(0);
            //}
            //if(distanceGoneBr<=-distance){
            //    robot.backRightMotor.setPower(0);
            //}
            //if(distanceGoneFl<=-distance){
            //robot.frontLeftMotor.setPower(0);
            //}
            //if(distanceGoneFr>=distance){
            //    robot.frontRightMotor.setPower(0);
            //}
        }
        setAllMotorsTo(0);
        //stopAndResetEncoders();
        sleep(250);
    }
    //strafing right distance (inch) with power [0, 1]
    public void strafeRight(double power, double distance) {
        stopAndResetEncoders();
        max = power*24;
        runWithEncoders();
        int counts = (int) (((distance / wheelCircumference) * countsPerRotation)*kFactor);

        while (robot.frontLeftMotor.getCurrentPosition()<counts) {
            double distanceGoneBl = wheelCircumference * ((robot.backLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneBr = wheelCircumference * ((robot.backRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFl = wheelCircumference * ((robot.frontLeftMotor.getCurrentPosition()/kFactor)/countsPerRotation);
            double distanceGoneFr = wheelCircumference * ((robot.frontRightMotor.getCurrentPosition()/kFactor)/countsPerRotation);

            double powerRaw = errorToPower(distance - distanceGoneFl, max, power, .15);
            setPowerOfMotorsTo(-powerRaw,powerRaw, powerRaw, -powerRaw);

            //if(distanceGoneBl<=-distance){
            //    robot.backLeftMotor.setPower(0);
            //}
            //if(distanceGoneBr>=distance){
            //    robot.backRightMotor.setPower(0);
            //}
            //if(distanceGoneFl>=distance){
            //    robot.frontLeftMotor.setPower(0);
            //}
            //if(distanceGoneFr<=-distance){
            //    robot.frontRightMotor.setPower(0);
            //}
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        sleep(250);
    }
    //going to any angle
    public void toAngle(double angle, double speed){
        runWithEncoders();
        double scale = 90*speed;
        double calcAngle = getHeading()-angle;
        boolean right = calcAngle<0;
        double power;
        if (right){
            while (calcAngle<0){
                power = errorToPower(-calcAngle, scale, speed, .05);
                setPowerOfMotorsTo(power, power , -power , -power );
                calcAngle = adjust(getHeading()-angle);
            }
        }
        else {
            while (calcAngle>0){
                power = errorToPower(Math.abs(calcAngle), scale, speed, .05);
                setPowerOfMotorsTo(-power, -power , power , power );
                calcAngle = adjust(getHeading()-angle);
            }
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
        sleep(500);
    }
    //goes to a position and angle on the field
    public void goToPosition(double angle, double currentX, double currentY, double x, double y) {
        runWithEncoders();
        double deltaY1 = y-currentY;

        double deltaX1 = x-currentX;

        double thetaX = -Math.PI/4;
        double thetaY = Math.PI/4;

        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        //double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngleX =  thetaX-gyroAngle;
        double calculationAngleY =  thetaY-gyroAngle;

        double deltaY2 = Math.sin(calculationAngleY) * deltaY1 + Math.sin(calculationAngleX)*deltaX1;
        double deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX)*deltaX1;

        deltaY2+=deltaX2;
        deltaX2+=deltaX2;

        // setting scale factor to the biggest value
        // dividing by this when setting powers will make sure that speeds are in the format 0-1
        // will also insure speeds are proportional to distance needed to travel in each direction so robot can move at angle theta
        double scaleFactor = deltaX2;
        if (deltaY2 > deltaX2) {
            scaleFactor = deltaY2;
        }

        int countsX = (int) ((deltaX2/wheelCircumference)*countsPerRotation);
        int countsY = (int) ((deltaY2/wheelCircumference)*countsPerRotation);
        setTargetPosition(robot.backLeftMotor.getCurrentPosition()+countsY, robot.frontLeftMotor.getCurrentPosition()+countsX, robot.backRightMotor.getCurrentPosition()+countsX, robot.frontRightMotor.getCurrentPosition()+countsY);
        runToPosition();
        while(robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()){
            double distanceGoneX = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneY = (wheelCircumference) * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double powerX = errorToPower(deltaX2-distanceGoneX, 24, deltaX2 / scaleFactor, .05);
            double powerY = errorToPower(deltaY2-distanceGoneY, 24, deltaY2 / scaleFactor, .05);
            setPowerOfMotorsTo(powerY, powerX, powerX, powerY);
            if (deltaX2-distanceGoneX<=.25){
                break;
            }
        }

        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        toAngle(angle, 1);
        currentYPosition=y;
        currentXPosition=x;
    }
    //goes in any direction
    public void translate(double angle, double x, double y){
        goToPosition(angle, 0, 0, x, y);
    }

    //Tools
    //sets the power of motors to inputted powers
    public void setPowerOfMotorsTo(double bl, double fl, double br, double fr) {
        //set diagonal wheels to prevent jerking
        robot.backLeftMotor.setPower(bl);
        robot.frontRightMotor.setPower(fr);
        robot.backRightMotor.setPower(br);
        robot.frontLeftMotor.setPower(fl);
    }
    //sets all motors to the same power
    public void setAllMotorsTo(double power) {
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);
    }
    //resets all wheel encoders to 0
    public void stopAndResetEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //set mode to run with encoders
    public void runWithEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //set mode to run without encoders
    public void runWithoutEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //set mode to run to Position
    public void runToPosition() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //set target position
    public void setTargetPosition(int bl, int fl, int br, int fr) {
        robot.backLeftMotor.setTargetPosition(bl);
        robot.backRightMotor.setTargetPosition(br);
        robot.frontRightMotor.setTargetPosition(fr);
        robot.frontLeftMotor.setTargetPosition(fl);
    }
    //gets the angle in degrees
    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle-resetAngle; // left [0,-180] right[0,180]
    }
    //adjusting heading on a scale of [-180,180]
    public double adjust(double angle) {
        if (angle>180){
            angle-=360;
        }
        if (angle<-180){
            angle+=360;
        }
        return angle;
    }
    //takes error and finds appropriate speed
    public double errorToPower(double error, double scale, double maxValue, double minValue){
        //proportional
        if(error<0){
            error=0;
        }
        double speed = ((error/scale)*(maxValue-minValue))+minValue;
        if (speed>maxValue){
            speed = maxValue;
        }
        return speed;
    }

    //ultimate goal specific
    //sets power of intake
    public void setIntakePower(double power){
        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setPower(power);
        robot.intake2.setPower(power);
    }
    //set indexer position
    public void controlIndexServo(double position){
        robot.theIndexerServoThatGoesOnTheFrontOfTheRobotAndKicksRingsIntoTheShooterSoWeCanScoreRingsAndBeVeryHappy.setPosition(position);
//        if(position>=robot.theIndexerServoThatGoesOnTheFrontOfTheRobotAndKicksRingsIntoTheShooterSoWeCanScoreRingsAndBeVeryHappy.getPosition()) {
//            while (robot.theIndexerServoThatGoesOnTheFrontOfTheRobotAndKicksRingsIntoTheShooterSoWeCanScoreRingsAndBeVeryHappy.getPosition() < position) {
//                idle();
//            }
//        }
//        else{
//            while (robot.theIndexerServoThatGoesOnTheFrontOfTheRobotAndKicksRingsIntoTheShooterSoWeCanScoreRingsAndBeVeryHappy.getPosition() > position) {
//                idle();
//            }
//        }
        sleep(100);

    }
    //Indexer
    public void shootRings(double x){
        for (i=0; i<x; i++) {
            controlIndexServo(.575);
            controlIndexServo(1);
            sleep(400);
        }
    }
    //set claw position
    public void controlClawServo(double position){
        robot.clawServo.setPosition(position);
    }
    //arm position
    public void controlArmServo(double position){
        robot.armServo.setPosition(position);
    }
    //flywheel
    public void setShooterPower(double velocity){
        robot.shooter.setVelocity(velocity);
    }
    //shoots all three rings at the same angle
    public void shoot(double a, double power){
        setShooterPower(power);
        toAngle(a, 1);
        setIntakePower(1);
        shootRings(3);
        setIntakePower(0);
    }
    //shoots all three rings at 3 different angles
    public void powerShot(double a1, double a2, double a3, double power, double powerE){
        setIntakePower(1);
        setShooterPower(power);
        toAngle(a1,.3);
        shootRings(1);

        setShooterPower(powerShotPower2);
        toAngle(a2, .3);
        shootRings(1);

        setShooterPower(powerShotPower3);
        toAngle(a3, .3);
        shootRings(1);

        setShooterPower(powerE);
        setIntakePower(0);
    }
    //dropping wobble goal
    public void dropWobbleGoal(){
        controlArmServo(1);//drop
        sleep(250);
        controlClawServo(.7);//open
        controlArmServo(0);//up
        sleep(500);
    }
    //picking up wobble goal
    public void pickUpWobbleGoal(double distanceLeft){
        strafeLeft(.5, distanceLeft);
        controlClawServo(.25);//close
        sleep(500);
        controlArmServo(.7);//up
    }
    //magic stuff
    public String magic8() {
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

    //Vision
    private void initializeFrameQueue(int capacity) {
        // The frame queue will automatically throw away bitmap frames if they are not processed quickly by the OpMode. This avoids a buildup of frames in memory
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }
    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            //error("camera not found or permission to use not granted: %s", cameraName);
        }
    }
    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        //YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only* image format supported by a camera
        final int imageFormat = ImageFormat.YUY2;

        //Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            //error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        //Some of the logic below runs asynchronously on other threads. Use of the synchronizer here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            //Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(CameraCaptureSession session) {
                    try {
                        //The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame( CameraCaptureSession session, CameraCaptureRequest request, CameraFrame cameraFrame) {
                                        //A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it for the duration of the callback. So we copy here manually.
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        //error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            //error("exception starting camera");
            synchronizer.finish(null);
        }

        //Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        //Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }
    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }
    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }
    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
    public int findNumRings(Bitmap bitmap) {

        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));

        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                //telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            //error("exception saving %s", file.getName());
        }
        Mat input = Imgcodecs.imread("/sdcard/FIRST/data/webcam-frame-0.jpg");

        int rings = 0;
        int h = input.height();
        int w = input.width();
        Imgproc.rectangle(input, new Point(0,0), new Point(640,375), new Scalar(255,0,0), -1);
        Imgproc.cvtColor(input,input,Imgproc.COLOR_BGR2HSV);
        Core.inRange(input, new Scalar(5, 100, 0),new Scalar(35, 360, 360),input);//new Scalar(0, 100, 0),new Scalar(35, 360, 360),input)

        int pixels = Core.countNonZero(input);
        telemetry.addData("pixels", pixels);
        //telemetry.addData("width=",w);
        //telemetry.addData("height=",h);
        if(pixels > 3500){
            rings = 4;

        }else if (pixels > 1000){
            rings = 1;
        }else{
            rings = 0;
        }
        return rings;
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}

