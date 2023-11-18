package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class RedClose extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;

    public Trajectory test;

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
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.setIsBlue(false);

        test = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12,0), 0)
                .build();
        robot.camera.init();
        robot.outtake.toMiddle();


    }
    @Override
    public void onStart(){
        robot.camera.isInit = false;

        drive.followTrajectoryAsync(test);

    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();

    }
}
