package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Camera implements Component {

    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private FirstVisionProcessor visionProcessor;
    private AprilTagProcessor aprilTag;
    public String deviceName;
    public HardwareMap hardwareMap;
    public boolean isRunning = false;
    public double centerBluePercent;
    public double centerRedPercent;
    public double leftBluePercent;
    public  double leftRedPercent;
    public double rightBluePercent;
    public double rightRedPercent;
    private volatile ParkingPosition position = ParkingPosition.RIGHT;
    private final Scalar BLACK = new Scalar(255,255,255);
    private static Scalar
            lowerRedBounds = new Scalar(175, 0, 0, 255),
            upperRedBounds = new Scalar(255, 175, 175, 255),
            lowerBlueBounds = new Scalar(0, 0, 175, 255),
            upperBlueBounds = new Scalar(175, 175, 255, 255);


    public Rect rectLeft = new Rect(110, 42, 40, 40);
    public Rect rectMiddle = new Rect(160, 42, 40, 40);
    public Rect rectRight = new Rect(210, 42, 40, 40);
    public Rect rect = new Rect(20, 20, 50, 50);
    boolean isBlue;
    public boolean isInit;

    public Mat rightBlueMat = new Mat(),
            rightRedMat = new Mat(),
            kernel = new Mat(),
            leftBlurredMat = new Mat(),
            leftBlueMat = new Mat(),
            centerBlueMat = new Mat(),
            leftRedMat = new Mat(),
            centerRedMat = new Mat(),
            centerBlurredMat = new Mat(),
            rightBlurredMat = new Mat();




    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }


    public Camera(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.hardwareMap = hardwareMap;

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionProcessor = new FirstVisionProcessor();


        // Create the vision portal the easy way.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
        android.util.Size cameraResolution = new android.util.Size(1280, 720);
//        builder.setCameraResolution(cameraResolution);
//
//
//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, deviceName), aprilTag,visionProcessor);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, deviceName))
                .addProcessors(visionProcessor, aprilTag)
                .setCameraResolution(cameraResolution)
                .build();

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        telemetryAprilTag();



        // Push telemetry to the Driver Station.
        telemetry.update();
    }

    @Override
    public String getTelemetry() {
        return null;
    }
    public void setIsBlue(boolean isBlue){
        this.isBlue = isBlue;
    }


    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.addData("Position",position);
        telemetry.addData("centerBluePercent",centerBluePercent);
        telemetry.addData("centerRedPercent",centerRedPercent);
        telemetry.addData("rightBluePercent",rightBluePercent);
        telemetry.addData("rightRedPercent",rightRedPercent);
        telemetry.addData("leftBluePercent",leftBluePercent);
        telemetry.addData("leftRedPercent",leftRedPercent);
        telemetry.addData("isBlue?",isBlue);


    }
    public ParkingPosition getPosition(){
        return position;
    }



    // maybe later
//    public void driveToTag(int wantedAprilTag){
//        ArrayList<AprilTagDetection> list = aprilTag.getDetections();
//        for(int i = 0; i < list.size(); i++){
//            if(list.get(i).id == wantedAprilTag){
//
//            }
//        }
//    }
    class FirstVisionProcessor implements VisionProcessor {

        //TODO find out rectangle values
        public Rect leftRect = new Rect(70, 470, 200, 200);
        public Rect rightRect = new Rect(1020, 470, 200, 200);
        public Rect centerRect = new Rect(540, 420, 200, 200);

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            processFrame(frame);
            telemetry.addLine("processFrame numero dos is working and in a loop");
            telemetry.update();
            return null;

        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.BLACK);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
            Paint selectedPaint2 = new Paint();
            selectedPaint2.setColor(Color.BLACK);
            selectedPaint2.setStyle(Paint.Style.STROKE);
            selectedPaint2.setStrokeWidth(scaleCanvasDensity * 4);
            Paint selectedPaint3 = new Paint();
            selectedPaint3.setColor(Color.BLACK);
            selectedPaint3.setStyle(Paint.Style.STROKE);
            selectedPaint3.setStrokeWidth(scaleCanvasDensity * 4);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(leftRect, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(centerRect,scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleRight = makeGraphicsRect(rightRect, scaleBmpPxToCanvasPx);
            canvas.drawRect(drawRectangleLeft, selectedPaint);
            canvas.drawRect(drawRectangleMiddle, selectedPaint2);
            canvas.drawRect(drawRectangleRight, selectedPaint3);

        }


        public Mat processFrame(Mat input) {
            //TODO fix submat
            telemetry.addLine("processFrame numero uno is working in a loop");

            if (isInit) {

                Imgproc.blur(input, leftBlurredMat, new Size(5, 5));
                Imgproc.blur(input, centerBlurredMat, new Size(5, 5));
                Imgproc.blur(input, rightBlurredMat, new Size(5, 5));

                leftBlurredMat = leftBlurredMat.submat(leftRect);
                rightBlurredMat = rightBlurredMat.submat(rightRect);
                centerBlurredMat = centerBlurredMat.submat(centerRect);


                Core.inRange(rightBlurredMat, lowerBlueBounds, upperBlueBounds, rightBlueMat);
                Core.inRange(leftBlurredMat, lowerBlueBounds, upperBlueBounds, leftBlueMat);
                Core.inRange(centerBlurredMat, lowerBlueBounds, upperBlueBounds, centerBlueMat);


                Core.inRange(rightBlurredMat, lowerRedBounds, upperRedBounds, rightRedMat);
                Core.inRange(leftBlurredMat, lowerRedBounds, upperRedBounds, leftRedMat);
                Core.inRange(centerBlurredMat, lowerRedBounds, upperRedBounds, centerRedMat);

                centerBluePercent = Core.countNonZero(centerBlueMat);
                centerRedPercent = Core.countNonZero(centerRedMat);
                leftBluePercent = Core.countNonZero(leftBlueMat);
                leftRedPercent = Core.countNonZero(leftRedMat);
                rightBluePercent = Core.countNonZero(rightBlueMat);
                rightRedPercent = Core.countNonZero(rightRedMat);


                if (isBlue) {
                    double maxBluePercent = Math.max(centerBluePercent, Math.max(leftBluePercent, rightBluePercent));
                    if (maxBluePercent == centerBluePercent) {
                        position = ParkingPosition.CENTER;
                    } else if (maxBluePercent == leftBluePercent) {
                        position = ParkingPosition.LEFT;
                    } else if (maxBluePercent == rightBluePercent) {
                        position = ParkingPosition.RIGHT;
                    } else {
                        telemetry.addLine("does not see anything, but knows it is blue");
                    }
                } else {
                    double maxRedPercent = Math.max(centerRedPercent, Math.max(leftRedPercent, rightRedPercent));
                    if (maxRedPercent == centerRedPercent) {
                        position = ParkingPosition.CENTER;
                    } else if (maxRedPercent == leftRedPercent) {
                        position = ParkingPosition.LEFT;
                    } else if (maxRedPercent == rightRedPercent) {
                        position = ParkingPosition.RIGHT;
                    } else {
                        telemetry.addLine("does not see anything, but knows it is red");
                    }
                }


                leftBlueMat.release();
                rightBlueMat.release();
                centerBlueMat.release();
                leftRedMat.release();
                rightRedMat.release();
                centerRedMat.release();
                leftBlurredMat.release();
                centerBlurredMat.release();
                rightRedMat.release();

                telemetry.update();


            }
            return input;
        }

    }


}