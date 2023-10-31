package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.vision.VisionPortal;
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

import java.util.ArrayList;
import java.util.List;

public class Camera implements Component {

    private final OpenCvCamera camera;
    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public String deviceName;
    public Mecanum mecanum;
    private volatile ParkingPosition position = ParkingPosition.RIGHT;
    private static Scalar
        lowerRedBounds = new Scalar (175,0,0,255),
        upperRedBounds = new Scalar (255,50,50,255),
        lowerBlueBounds = new Scalar (0,0,175,255),
        upperBlueBounds = new Scalar (50,50,255,255);

    public Mat rightBlueMat = new Mat(),
            rightRedMat = new Mat(),
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
    //TODO make a way to find out where the team game element is


    public Camera(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, Mecanum mecanum) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.mecanum = mecanum;

    }

    @Override
    public void init() {

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, deviceName), aprilTag);


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

    }
    public void driveToTag(int wantedAprilTag){
        ArrayList<AprilTagDetection> list = aprilTag.getDetections();
        for(int i = 0; i < list.size(); i++){
            if(list.get(i).id == wantedAprilTag){

            }
        }
    }
    //TODO find out rectangle values
    public Rect leftRect = new Rect(0,0,0,0);
    public Rect rightRect = new Rect(0,0,0,0);
    public Rect centerRect = new Rect(0,0,0,0);
    public Mat processFrame(Mat input){
        //TODO fix submat
        Imgproc.blur(input, leftBlurredMat, new Size(5,5));
        Imgproc.blur(input, centerBlurredMat, new Size(5,5));
        Imgproc.blur(input, rightBlurredMat, new Size(5,5));
        leftBlurredMat = leftBlurredMat.submat(leftRect);
        rightBlurredMat = rightBlurredMat.submat(rightRect);
        centerBlurredMat = centerBlurredMat.submat(centerRect);

        Core.inRange(rightBlurredMat,lowerBlueBounds,upperBlueBounds,rightBlueMat);
        Core.inRange(leftBlurredMat,lowerBlueBounds,upperBlueBounds, leftBlueMat);
        Core.inRange(centerBlurredMat, lowerBlueBounds,upperBlueBounds,centerBlueMat);


        Core.inRange(rightBlurredMat,lowerRedBounds,upperRedBounds,rightRedMat);
        Core.inRange(leftBlurredMat,lowerRedBounds,upperRedBounds,leftRedMat);
        Core.inRange(centerBlurredMat,lowerRedBounds,upperRedBounds,centerRedMat);

        double centerBluePercent = Core.countNonZero(centerBlueMat);
        double centerRedPercent = Core.countNonZero(centerRedMat);
        double leftBluePercent = Core.countNonZero(leftBlueMat);
        double leftRedPercent = Core.countNonZero(leftRedMat);
        double rightBluePercent = Core.countNonZero(rightBlueMat);
        double rightRedPercent = Core.countNonZero(centerRedMat);

        boolean isBlue = Math.max(centerBluePercent,centerRedPercent) == centerBluePercent;

        if(isBlue){
            double maxBluePercent = Math.max(centerBluePercent,Math.max(leftBluePercent,rightBluePercent));
            if(maxBluePercent == centerBluePercent){
                position = ParkingPosition.CENTER;
            } else if(maxBluePercent == leftBluePercent){
                position = ParkingPosition.LEFT;
            } else if(maxBluePercent == rightBluePercent){
                position = ParkingPosition.RIGHT;
            } else{
                telemetry.addLine("does not see anything, but knows it is blue");
            }
        } else{
            double maxRedPercent = Math.max(centerRedPercent, Math.max(leftRedPercent,rightRedPercent));
            if(maxRedPercent == centerRedPercent){
                position = ParkingPosition.CENTER;
            } else if(maxRedPercent == leftRedPercent){
                position = ParkingPosition.LEFT;
            } else if(maxRedPercent == rightRedPercent){
                position = ParkingPosition.RIGHT;
            } else{
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

        return input;
    }

}