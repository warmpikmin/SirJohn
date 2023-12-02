package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class TestAuto extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
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
        drive = new RRMecanum(hardwareMap);
        robot.camera.init();
        robot.outtake.toMiddle();
        robot.intake.toggleArm();
    }
    @Override
    public void onStart(){
        robot.camera.isInit = false;
        robot.outtake.toMiddle();
//        List<AprilTagDetection> currentDetections = robot.camera.aprilTag.getDetections();

    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();
    }
}
