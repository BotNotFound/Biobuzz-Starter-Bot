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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Aimbot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "Teleop (no Aimbot)")
public class Teleop extends OpMode {
    public static final double LAUNCHER_TARGET_VELOCITY = 1280;
    public static final double LAUNCHER_MIN_VELOCITY = 1200;

    // Delay after pressing RT before transfer + intake start, so the shooter can spin up
    public static final double SHOOT_FEED_DELAY_MS = 1500;
    // Drivetrain speed multiplier while controller 1's left trigger is held
    public static final double SLOW_MODE_SCALE = 0.5;

    public static double intakePower = 1.0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean shootWasPressed = false;
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
        updateShootTimer();

        double driveScale = gamepad1.left_trigger_pressed ? SLOW_MODE_SCALE : 1.0;
        double turn = scaleCubic(-gamepad1.right_stick_x) * driveScale;
        if (aimbot != null && aimbot.isEnabled() && gamepad2.right_trigger_pressed) {
            // Auto-aim overrides driver rotation while shooting
            turn = aimbot.getTurnPower();
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                scaleCubic(-gamepad1.left_stick_y) * driveScale,
                scaleCubic(-gamepad1.left_stick_x) * driveScale,
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

    // Restart the spin-up delay each time RT is pressed
    private void updateShootTimer() {
        boolean pressed = gamepad2.right_trigger_pressed;
        if (pressed && !shootWasPressed) {
            shootTimer.reset();
        }
        shootWasPressed = pressed;
    }

    private boolean feedReady() {
        return gamepad2.right_trigger_pressed && shootTimer.milliseconds() >= SHOOT_FEED_DELAY_MS;
    }

    // Controller 2: Y = Aimbot on, X = Aimbot off
    private void updateAimbot() {
        if (aimbot == null) return;
        if (gamepad2.y) {
            aimbot.setEnabled(true);
        } else if (gamepad2.x) {
            aimbot.setEnabled(false);
        }
        aimbot.update(follower.pose().x(), follower.pose().y(), follower.pose().heading());
        telemetry.addData("Aimbot", aimbot.isEnabled() ? "ACTIVE" : "OFF");
        if (aimbot.isEnabled()) {
            telemetry.addData("Aimbot source", aimbot.isTagVisible() ? "LIVE"
                    : aimbot.isUsingMemory() ? "MEMORY" : "NONE");
            telemetry.addData("Aimbot error", aimbot.getError());
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

        if (feedReady()) {
            transferServo.setPower(-1);
        } else if (gamepad2.right_bumper) {
            transferServo.setPower(1);
        } else {
            transferServo.setPower(0);
        }
    }

    void intake() {
        if (gamepad2.left_trigger_pressed || feedReady()) {
            intake.setPower(intakePower);
        } else if(gamepad2.left_bumper) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }
    }

    void resetGyro() {
        // The remembered tag position is in the old heading frame, so it is no longer valid
        if (aimbot != null) aimbot.clearMemory();
        follower.setPose(follower.pose().withHeading(Math.toRadians(0)));
    }
}
