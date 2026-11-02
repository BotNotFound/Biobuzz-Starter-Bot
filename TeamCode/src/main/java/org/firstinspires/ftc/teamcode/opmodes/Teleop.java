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
    private static final double LAUNCHER_TARGET_VELOCITY = 1450;
    private static final double LAUNCHER_MIN_VELOCITY = 1400;

    public static double intakePower = 1.0;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;
    private CRServo transfer2Servo;
    private DcMotorEx launcher;
    private Follower follower;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "transfer");
        transfer2Servo = hardwareMap.get(CRServo.class, "transfer2");
        leftIntakeServo = hardwareMap.get(CRServo.class, "intakeleft");
        rightIntakeServo = hardwareMap.get(CRServo.class, "intakeright");

        follower = Constants.create(hardwareMap);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        setTransferPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer2Servo.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");

        follower.setPose(new Pose(0,0,Math.toRadians(0)));
    }
    private double scaleCubic(double input) {
        return input * input * input;
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                scaleCubic(-gamepad1.left_stick_y),
                scaleCubic(-gamepad1.left_stick_x),
                scaleCubic(-gamepad1.right_stick_x),
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
        if (gamepad2.right_trigger_pressed) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad2.right_trigger_pressed && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            setTransferPower(-1);
        } else if (gamepad2.right_bumper) {
            setTransferPower(1);
        } else {
            setTransferPower(0);
        }
    }

    // transfer2 spins opposite to the first transfer
    void setTransferPower(double power) {
        windmillServo.setPower(power);
        transfer2Servo.setPower(-power);
    }

    void intake() {
        if (gamepad2.left_trigger_pressed) {
            intake.setPower(intakePower);
            leftIntakeServo.setPower(-intakePower);
            rightIntakeServo.setPower(-intakePower);
        } else if(gamepad2.left_bumper) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        }
    }

    void resetGyro() {
        follower.setPose(follower.pose().withHeading(Math.toRadians(0)));
    }
}
