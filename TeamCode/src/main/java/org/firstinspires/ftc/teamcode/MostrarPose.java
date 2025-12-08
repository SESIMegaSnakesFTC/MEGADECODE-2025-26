package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name="Mostrar Pose do Robô")
public class MostrarPose extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // DEFINIR POSIÇÃO INICIAL DO ROBÔ
        follower.setPose(
                new Pose(0, 0, Math.toRadians(0))  // X, Y, Heading
        );
    }

    @Override
    public void loop() {
        // Atualiza odometria
        follower.update();

        Pose p = follower.getPose();

        telemetry.addLine("=== POSIÇÃO ATUAL DO ROBÔ ===");
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading (rad)", p.getHeading());
        telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));

        telemetry.update();
    }
}
