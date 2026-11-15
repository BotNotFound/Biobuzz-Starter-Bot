package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Aimbot;

@TeleOp(name = "RedTeleop")
public class RedTeleop extends Teleop {
    @Override
    protected int[] getTargetTagIds() {
        return Aimbot.RED_TAG_IDS;
    }
}
