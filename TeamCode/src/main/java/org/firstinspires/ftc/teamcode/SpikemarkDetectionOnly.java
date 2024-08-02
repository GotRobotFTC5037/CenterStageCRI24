package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Spikemark Detection Only", group="Auto")
public class SpikemarkDetectionOnly extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // CENTER, RIGHT, or LEFT
        String propPosition = "CENTER";

        // MOVEMENT:
        // ----------------
        // FORWARD: +X
        // LEFT: +Y
        // RIGHT: -Y
        // BACKWARD: -X

        Trajectory driveToDetectionPosition = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(27,0,Math.toRadians(0)))
                .build();
        waitForStart();
        if(isStopRequested()) return;

        robot.intake.setPower(0.5);
        drive.followTrajectory(driveToDetectionPosition);

        robot.intake.setPower(0);

        sleep(100);

        if (robot.rightDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = "RIGHT";
        } else if (robot.leftDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = "LEFT";
        } else {
            propPosition = "CENTER";
        }

        telemetry.addData("Prop Position", propPosition);
        telemetry.update();
        sleep(100);

        switch (propPosition) {
            case "CENTER":
                Trajectory moveRight = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(moveRight);
                sleep(200);
                robot.transfer.setPower(-0.2);
                sleep(500);
                robot.transfer.setPower(0);
                sleep(1000);
                Trajectory unMoveRight = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(3)
                        .build();
                drive.followTrajectory(unMoveRight);
                break;

            case "RIGHT":
                Trajectory adjustLeft = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(4)
                        .build();
                drive.followTrajectory(adjustLeft);
                drive.turn(Math.toRadians(-95));
                robot.transfer.setPower(-0.35);
                sleep(1000);
                robot.transfer.setPower(0);
                sleep(1000);
                drive.turn(Math.toRadians(95));
                Trajectory unAdjustLeft = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(4)
                        .build();
                drive.followTrajectory(unAdjustLeft);
                break;

            case "LEFT":
                Trajectory adjustRightSlightly = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(adjustRightSlightly);
                drive.turn(Math.toRadians(85));
                robot.transfer.setPower(-0.435);
                sleep(1000);
                robot.transfer.setPower(0);
                sleep(1000);
                drive.turn(Math.toRadians(-85));
                Trajectory unAdjustRightSlightly = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(3)
                        .build();
                drive.followTrajectory(unAdjustRightSlightly);
                break;
        }

        Trajectory backUp = drive.trajectoryBuilder(new Pose2d())
                .back(27)
                .build();

        drive.followTrajectory(backUp);

    }
}
