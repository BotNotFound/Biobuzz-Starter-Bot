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

import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp
public class Teleop extends OpMode {
    private static final double LAUNCHER_TARGET_VELOCITY = 1500;
    private static final double LAUNCHER_MIN_VELOCITY = 1450;

    public static double intakePower = 1.0;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;
    private DcMotorEx launcher;
    private Follower follower;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "intakeleft");
        rightIntakeServo = hardwareMap.get(CRServo.class, "intakeright");

        follower = Constants.create(hardwareMap);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        follower.setPose(new Pose(0,0,Math.toRadians(0)));
    }
    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
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

    void launch() {
        if (gamepad1.right_trigger_pressed) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_trigger_pressed && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(-1);
        } else {
            windmillServo.setPower(0);
        }
    }

    void intake() {
        if (gamepad1.left_trigger_pressed) {
            intake.setPower(intakePower);
            leftIntakeServo.setPower(-intakePower);
            rightIntakeServo.setPower(-intakePower);
        } else {
            intake.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        }

        if(gamepad1.left_bumper) {
            intake.setPower(-intakePower);
        }
    }

    void resetGyro() {
        follower.setPose(follower.pose().withHeading(Math.toRadians(0)));
    }
}
