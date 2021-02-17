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
    public double currentXPosition = 0;
    public double currentYPosition = 0;

    public double resetAngle = 0;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtime2 = new ElapsedTime();

    public void initializeRobot() {
        robot.initializeHardware(hardwareMap);

        //initializing camera
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        if (!OpenCVLoader.initDebug()) {
            error("Error: Cannot load OpenCV Library");
        } else {
            telemetry.addData(">", "Loaded OpenCV");
            telemetry.update();
        }

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();


            bmp = frameQueue.poll();

        }
        finally {
            closeCamera();
        }
    }
    //moving forward distance (inch) with power [0, 1]
    public void forward(double power, double distance) {
        runWithEncoders();
        int counts = (int) ((distance / (wheelCircumference)) * (countsPerRotation));
        setTargetPosition(counts, counts, counts, counts);
        runToPosition();
        while (robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()) {
            double distanceGone = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            setAllMotorsTo(errorToPower((distance-distanceGone), 24, power, .2));
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
    }
    //moving backward distance (inch) with power [0, 1]
    public void backward(double power, double distance) {
        runWithEncoders();
        int counts = (int) -((distance / (wheelCircumference)) * (countsPerRotation));
        setTargetPosition(counts, counts, counts, counts);
        runToPosition();
        while (robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()) {
            double distanceGone = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            setAllMotorsTo(-errorToPower((distance-distanceGone), 24, power, .2));
        }
        setAllMotorsTo(0);
        stopAndResetEncoders();
    }
    //strafing left distance (inch) with power [0, 1]
    public void strafeLeft(double power, double distance) {
        runWithEncoders();
        int counts = (int) (((distance / (wheelCircumference)) * (countsPerRotation))*Math.sqrt(2));//divide by root 2
        setTargetPosition(counts, -counts, -counts, counts);
        runToPosition();
        while (robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()) {
            double distanceGone = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double power2 = errorToPower((distance-distanceGone), 24, power, .2);
            setPowerOfMotorsTo(power2, -power2 , -power2 , power2 );
        }
        stopAndResetEncoders();
    }
    //strafing right distance (inch) with power [0, 1]
    public void strafeRight(double power, double distance) {
        runWithEncoders();
        int counts = (int) (((distance / (wheelCircumference)) * (countsPerRotation))*Math.sqrt(2));//divide by root 2
        setTargetPosition(-counts, counts, counts, -counts);
        runToPosition();

        while (robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()) {
            double distanceGone = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double power2 = errorToPower((distance-distanceGone), 24, power, .2);
            setPowerOfMotorsTo(-power2, power2 , power2 , -power2 );
        }
        stopAndResetEncoders();
    }
    //going to any angle
    public void toAngle(double angle){
        runWithEncoders();
        double calcAngle = getHeading()-angle;
        boolean right = calcAngle<0;
        double power;
        if (right){
            while (calcAngle<0){
                power = errorToPower(angle-getHeading(), 90, 1, .1);
                setPowerOfMotorsTo(power, power , -power , -power );
                calcAngle = adjust(getHeading()-angle);
            }
        }
        else {
            while (calcAngle>0){
                power = errorToPower(Math.abs(angle-getHeading()), 90, 1, .1);
                setPowerOfMotorsTo(-power, -power , power , power );
                calcAngle = adjust(getHeading()-angle);
            }
        }
        setAllMotorsTo(0);
    }
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
    //adjusting heading on a scale of [0, 2pi]
    public double adjust2(double angle) {
        if (angle > Math.PI/2) {
            angle = angle + (5*Math.PI / 2);
        }
        else{
            angle = angle+Math.PI/2;
        }
        return angle;
    }
    //goes to a position and angle on the field
    public void goToPosition(double power, double angle, double currentX, double currentY, double x, double y) {
        runWithEncoders();

        double deltaX =  x-currentX; //required change in x position from current to desired
        double deltaY = y-currentY; //required change in y position from current to desired
        double gyroAngle = adjust2(getHeading() * Math.PI / 180); //Converts gyroAngle into radians
        double angleOfTravel = Math.atan2(deltaY, deltaX);
        double theta =  angleOfTravel-gyroAngle;

        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        double calculationAngle = theta - (Math.PI / 4);

        //magnitude of movement using pythagorean theorem
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double xDist = Math.cos(calculationAngle) * distance;
        double yDist = Math.sin(calculationAngle) * distance;

        // setting scale factor to the biggest value
        // dividing by this when setting powers will make sure that speeds are in the format 0-1
        // will also insure speeds are proportional to distance needed to travel in each direction so robot can move at angle theta
        double scaleFactor = xDist;
        if (yDist > xDist) {
            scaleFactor = yDist;
        }

        int countsX = (int) ((xDist / (wheelCircumference)) * (countsPerRotation));
        int countsY = (int) ((yDist / (wheelCircumference)) * (countsPerRotation));
        setTargetPosition(robot.backLeftMotor.getCurrentPosition()+countsX, robot.frontLeftMotor.getCurrentPosition()+countsY, robot.backRightMotor.getCurrentPosition()+countsY, robot.frontRightMotor.getCurrentPosition()+countsX);
        runToPosition();
        while(robot.backLeftMotor.isBusy()||robot.backRightMotor.isBusy()||robot.frontLeftMotor.isBusy()||robot.frontRightMotor.isBusy()){
            double distanceGoneX = (wheelCircumference) * (robot.backLeftMotor.getCurrentPosition()/countsPerRotation);
            double distanceGoneY = (wheelCircumference) * (robot.backRightMotor.getCurrentPosition()/countsPerRotation);
            double powerX = errorToPower(xDist-distanceGoneX, 24, xDist / scaleFactor, .2);
            double powerY = errorToPower(yDist-distanceGoneY, 24, yDist / scaleFactor, .2);
            setPowerOfMotorsTo(powerX, powerY, powerY, powerX);
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        toAngle(angle);
        currentYPosition=y;
        currentXPosition=x;
    }
    //takes error and finds appropriate speed
    private double errorToPower(double error, double scale, double maxValue, double minValue){
        //proportional
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
        robot.intake.setPower(power);
    }
    //set servo position
    public void controlLaunchServo(double position){
        robot.barrierServo.setPosition(position);
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
    public void setShooterPower(double power){
        robot.shooter.setPower(power);
    }
    //shoots all three rings at the same angle
    public void shoot(double a, double power){
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(power);
        toAngle(a);
        controlLaunchServo(0);
        setIntakePower(.5);
        sleep(3000);
        controlLaunchServo(1);
        setIntakePower(0);
    }
    //shoots all three rings at 3 different angles
    public void powerShot(double a1, double a2, double a3, double power, double powerE){
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(power);
        sleep(1000);
        setIntakePower(1);
        toAngle(a1);
        setIntakePower(0);
        controlLaunchServo(0);
        sleep(500);
        setIntakePower(.2);
        controlLaunchServo(1);
        sleep(500);
        setIntakePower(1);
        sleep(1000);

        toAngle(a2);
        setIntakePower(0);
        controlLaunchServo(0);
        sleep(500);
        setIntakePower(.2);
        controlLaunchServo(1);
        sleep(500);
        setIntakePower(1);
        sleep(1000);

        toAngle(a3);
        setIntakePower(0);
        controlLaunchServo(0);
        sleep(500);
        setIntakePower(.5);
        sleep(1000);

        setShooterPower(powerE);
        controlLaunchServo(1);
        setIntakePower(0);
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
        Imgproc.rectangle(input, new Point(0,0), new Point(640,350), new Scalar(255,0,0), -1);
        Imgproc.cvtColor(input,input,Imgproc.COLOR_BGR2HSV);
        Core.inRange(input, new Scalar(0, 100, 0),new Scalar(35, 360, 360),input);

        int pixels = Core.countNonZero(input);
        //telemetry.addData("pixels", pixels);
        //telemetry.addData("width=",w);
        //telemetry.addData("height=",h);
        if(pixels > 4000){
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

