package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner();
    }
}
