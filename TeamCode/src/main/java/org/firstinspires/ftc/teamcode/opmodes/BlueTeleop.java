package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Aimbot;

@TeleOp(name = "BlueTeleop")
public class BlueTeleop extends Teleop {
    @Override
    protected int[] getTargetTagIds() {
        return Aimbot.BLUE_TAG_IDS;
    }
}
