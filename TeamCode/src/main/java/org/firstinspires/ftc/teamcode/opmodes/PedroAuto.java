package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class PedroAuto extends LinearOpMode {

    private Follower follower;
    private DcMotorEx launcher;
    private CRServo transfer;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose parkingPath = poseFactory.of(10, 106, 180);
    private final Pose parkingPathControl1 = poseFactory.of(27.5242, 49.8596, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> launcher.setVelocity(1450)),
                waitMs(3000),
                instant(() -> transfer.setPower(-1)),
                follow(follower, startingPath()),
                instant(() -> {
                    launcher.setVelocity(0);
                    transfer.setPower(0);
                }),
                follow(follower, parkingPath())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        transfer = hardwareMap.get(CRServo.class, "windmill");

        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path parkingPath() {
        return curve(start, parkingPathControl1, parkingPath).linear(start, parkingPath);
    }
}
