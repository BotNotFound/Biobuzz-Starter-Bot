package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Aimbot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "Teleop (no Aimbot)")
public class Teleop extends OpMode {
    public static final double LAUNCHER_TARGET_VELOCITY = 1280;
    public static final double LAUNCHER_MIN_VELOCITY = 1200;

    public static double intakePower = 1.0;
    private DcMotor intake;
    private CRServo transferServo;
    private DcMotorEx launcher;
    private Follower follower;
    private Aimbot aimbot;

    /** AprilTag IDs the Aimbot aims at. Return null (default) for no Aimbot. */
    protected int[] getTargetTagIds() {
        return null;
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        transferServo = hardwareMap.get(CRServo.class, "transferRight");

        follower = Constants.create(hardwareMap);
        if (getTargetTagIds() != null) {
            aimbot = new Aimbot(hardwareMap, getTargetTagIds());
        }
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        transferServo.setPower(0);
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        follower.setPose(new Pose(0,0,Math.toRadians(0)));
    }
    private double scaleCubic(double input) {
        return input * input * input;
    }

    @Override
    public void stop() {
        if (aimbot != null) aimbot.close();
    }

    @Override
    public void loop() {
        updateAimbot();
        double turn = scaleCubic(-gamepad1.right_stick_x);
        if (aimbot != null && aimbot.isEnabled() && gamepad2.right_trigger_pressed) {
            // Auto-aim overrides driver rotation while shooting
            turn = aimbot.getTurnPower();
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                scaleCubic(-gamepad1.left_stick_y),
                scaleCubic(-gamepad1.left_stick_x),
                turn,
                follower.pose().heading()
        );
        follower.manual(powers);
        follower.update();

        launch();

        intake();

        if(gamepad1.start){
            resetGyro();
        }

        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
    }

    // Controller 2: Y = Aimbot on, X = Aimbot off
    private void updateAimbot() {
        if (aimbot == null) return;
        if (gamepad2.y) {
            aimbot.setEnabled(true);
        } else if (gamepad2.x) {
            aimbot.setEnabled(false);
        }
        aimbot.update();
        telemetry.addData("Aimbot", aimbot.isEnabled() ? "ACTIVE" : "OFF");
        if (aimbot.isEnabled()) {
            telemetry.addData("Aimbot tag visible", aimbot.isTagVisible());
            telemetry.addData("Aimbot aligned", aimbot.isAligned());
        }
    }

    void launch() {
        if (gamepad2.right_trigger_pressed) {
            double target = LAUNCHER_TARGET_VELOCITY;
            if (aimbot != null && aimbot.isEnabled()) {
                // Auto-adjust RPM from distance/angle to the tag
                target = aimbot.getShooterVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("Aimbot range (in)", aimbot.getRange());
                telemetry.addData("Aimbot yaw (deg)", aimbot.getYaw());
            }
            launcher.setVelocity(target);
            telemetry.addData("Launcher target", target);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad2.right_trigger_pressed) {
            transferServo.setPower(-1);
        } else if (gamepad2.right_bumper) {
            transferServo.setPower(1);
        } else {
            transferServo.setPower(0);
        }
    }

    void intake() {
        if (gamepad2.left_trigger_pressed || gamepad2.right_trigger_pressed) {
            intake.setPower(intakePower);
        } else if(gamepad2.left_bumper) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }
    }

    void resetGyro() {
        follower.setPose(follower.pose().withHeading(Math.toRadians(0)));
    }
}
