package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Camera.ParkingPosition.CENTER;
import static org.firstinspires.ftc.teamcode.Components.Camera.ParkingPosition.LEFT;
import static org.firstinspires.ftc.teamcode.Components.Camera.ParkingPosition.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class BlueClose extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
    public Camera.ParkingPosition position = CENTER;

    public Trajectory forward;
    public Trajectory toCenter;
    public Trajectory extraForward;
    public Trajectory leftInitial;
    public Trajectory leftMiddle;
    public Trajectory centerMiddle;
    public Trajectory rightMiddle;

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
        robot.camera.isInit = true;
        robot.intake.openClaw();
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.setIsBlue(true);
        robot.outtake.toMiddle();


        forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(28,-5),RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        extraForward = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28,15), Math.toRadians(0),RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(38,15), Math.toRadians(0),RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        leftInitial = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(36,20), Math.toRadians(0),RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        rightMiddle = drive.trajectoryBuilder(new Pose2d())
                .forward(40, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(42,-8),Math.toRadians(0))
                .build();
        centerMiddle = drive.trajectoryBuilder(new Pose2d())
                .forward(21, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25,12),Math.toRadians(0))
                .build();
        leftMiddle = drive.trajectoryBuilder(new Pose2d())
                .forward(16,RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(21,12),Math.toRadians(0))
                .build();


        robot.camera.init();
        robot.intake.setAutoPos();



    }
    @Override
    public void onStart() throws InterruptedException {
        robot.camera.isInit = false;
        position = robot.camera.getPosition();
        robot.intake.setAutoPos();
        drive.waitForIdle();
        sleep(1000);


        if(position ==LEFT) {
            drive.waitForIdle();
            drive.followTrajectoryAsync(leftInitial);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(80));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(leftMiddle);
            robot.slides.move(85,1);
            sleep(2000);
            robot.outtake.unFlip();
            drive.waitForIdle();
        }


        if(position == CENTER){
            drive.waitForIdle();
            drive.followTrajectoryAsync(extraForward);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(80));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(centerMiddle);
            robot.slides.move(85,1);
            sleep(2000);
            robot.outtake.unFlip();
            drive.waitForIdle();
        }
        if(position == RIGHT){
            drive.followTrajectoryAsync(forward);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(80));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(rightMiddle);
//            sleep(1000);
            robot.slides.move(85,1);
            sleep(2000);
            robot.outtake.unFlip();
            drive.waitForIdle();


        }
        robot.intake.setAutoPos();




    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();

    }
}