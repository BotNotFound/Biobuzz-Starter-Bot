package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Teleop;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "HiveShoot", group = "Autonomous")
public class HiveShoot extends LinearOpMode {

    private static final double SHOOT_TIME_MS = 15000;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 9, 90);
    private final Pose startPark = poseFactory.of(59, 9, 270);
    private final Pose parkpath = poseFactory.of(14.5852, 94.8487, 270);
    private final Pose parkpathControl1 = poseFactory.of(16.8155, 40.3749, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startPark()),
                follow(follower, parkpath())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo windmill = hardwareMap.get(CRServo.class, "windmill");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setPower(0);

        waitForStart();

        // Spin up the shooter to teleop's target velocity
        launcher.setVelocity(Teleop.LAUNCHER_TARGET_VELOCITY);
        boolean transferStarted = false;
        boolean parking = false;
        ElapsedTime shootTimer = new ElapsedTime();

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            if (!parking) {
                if (!transferStarted && launcher.getVelocity() > Teleop.LAUNCHER_MIN_VELOCITY) {
                    // Up to speed: start the transfer
                    windmill.setPower(-1);
                    transferStarted = true;
                    shootTimer.reset();
                } else if (transferStarted && shootTimer.milliseconds() >= SHOOT_TIME_MS) {
                    // Done shooting: stop everything and park immediately
                    windmill.setPower(0);
                    launcher.setVelocity(0);
                    parking = true;
                    schedule(autoRoutine());
                }
            }

            telemetry.addData("launcher velocity", launcher.getVelocity());

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

    public Path startPark() {
        return Paths.line(start, startPark).constant(startPark);
    }

    public Path parkpath() {
        return Paths.curve(startPark, parkpathControl1, parkpath).constant(parkpath);
    }
}
