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
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class RedClose extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
    public Camera.ParkingPosition position = CENTER;

    public Trajectory forward;
    public Trajectory toCenter;
    public Trajectory extraForward;
    public Trajectory rightInitial;

    @Override
    protected Robot setRobot() {
        this.robot = new SirJohn();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }
    //TODO make proper movement
    @Override
    public void onInit(){
        robot.intake.openClaw();
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.setIsBlue(false);
        robot.outtake.toMiddle();


        forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(28,-0.7))

                .build();
        extraForward = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28,-20), Math.toRadians(0))
                .splineTo(new Vector2d(36,-20), Math.toRadians(0))
                .build();
        rightInitial = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28,-30), Math.toRadians(0))
                .build();


        robot.camera.init();
        //robot.intake.setAutoPos();



    }
    @Override
    public void onStart() throws InterruptedException {
        robot.camera.isInit = false;
        position = robot.camera.getPosition();
        position = LEFT;



        drive.followTrajectoryAsync(forward);
        drive.waitForIdle();
        sleep(1000);

        if(position == CENTER){
            drive.waitForIdle();
            drive.followTrajectoryAsync(extraForward);
        }
        drive.waitForIdle();
        drive.turnAsync(Math.toRadians(-90));
        if(position == RIGHT){
            drive.waitForIdle();
            drive.followTrajectoryAsync(rightInitial);
        }
        drive.waitForIdle();
        //robot.intake.setAutoPos();
        robot.intake.toggleClaw();


    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();
        telemetry.addData("position",position);
        telemetry.update();

    }
}
