package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class TestAuto extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;

    public Trajectory tester;

    @Override
    protected Robot setRobot() {
        this.robot = new SirJohn();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit(){
        FtcDashboard.getInstance().startCameraStream(robot.camera.streamSource, 0);
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.init();
        robot.outtake.toMiddle();
        robot.intake.toggleArm();

        tester = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0,-100), RRMecanum.getVelocityConstraint(1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.camera.init();
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }
    @Override
    public void onStart() throws InterruptedException{
        robot.outtake.toMiddle();
        drive.waitForIdle();
        drive.followTrajectoryAsync(tester);


    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();

        List<AprilTagDetection> currentDetections = robot.camera.aprilTag.getDetections();
        for(AprilTagDetection detection : currentDetections){
            if(detection.metadata != null) {
                if (detection.ftcPose.y <= 2 || detection.ftcPose.y >= -2) {
                    drive.breakFollowing();
                    telemetry.addLine("The detection thing is working");
                    telemetry.update();
                }
            }
        }
    }
}
