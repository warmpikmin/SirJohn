package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import java.util.concurrent.atomic.AtomicReference;
import org.opencv.android.Utils;
import org.firstinspires.ftc.teamcode.VisionProcessors.*;

import java.util.List;

public class Camera implements Component {

    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private TeamPropDetection visionProcessor;
    public CameraStreamProcessor streamSource;
    public AprilTagProcessor aprilTag;
    public String deviceName;
    public HardwareMap hardwareMap;

    public Camera(String deviceName, BaseOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.hardwareMap = hardwareMap;

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionProcessor = new TeamPropDetection(opMode);
        streamSource = new CameraStreamProcessor();


        // Create the vision portal the easy way.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
        android.util.Size cameraResolution = new android.util.Size(1280, 720);
//        builder.setCameraResolution(cameraResolution);
//
//
//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, deviceName), aprilTag,visionProcessor);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, deviceName))
                .addProcessors(visionProcessor, aprilTag, streamSource)
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
    public void update() {}

    @Override
    public String getTelemetry() {
//        telemetryAprilTag();
        return visionProcessor.getTelemetry();
    }

    public void setIsBlue(boolean isBlue){
        visionProcessor.setIsBlue(isBlue);
    }

    public TeamPropDetection.ParkingPosition getPosition() {
        return visionProcessor.getPosition();
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


    // maybe later
//    public void driveToTag(int wantedAprilTag){
//        ArrayList<AprilTagDetection> list = aprilTag.getDetections();
//        for(int i = 0; i < list.size(); i++){
//            if(list.get(i).id == wantedAprilTag){
//
//            }
//        }
//    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }


}