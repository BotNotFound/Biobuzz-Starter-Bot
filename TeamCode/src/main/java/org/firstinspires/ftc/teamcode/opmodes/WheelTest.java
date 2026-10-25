package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelTest extends OpMode {
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    public void init(){
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
    }

    public void loop(){
        if(gamepad1.a){
            FL.setPower(1);
        }
        if(gamepad1.b){
            FR.setPower(1);
        }
        if(gamepad1.x){
            BL.setPower(1);
        }
        if(gamepad1.y){
            BR.setPower(1);
        }
    }
}
