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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class PedroAuto extends LinearOpMode {

    private Follower follower;
    private DcMotorEx launcher;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose start_2 = poseFactory.of(56, 8, 270);
    private final Pose backEnd = poseFactory.of(56, 36, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> launcher.setVelocity(1350)),
                follow(follower, startPath()),
                instant(() -> launcher.setVelocity(0)),
                follow(follower, backEnd())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

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

    public Path startPath() {
        return line(start, start_2).constant(start_2);
    }

    public Path backEnd() {
        return line(start_2, backEnd).constant(backEnd);
    }
}
