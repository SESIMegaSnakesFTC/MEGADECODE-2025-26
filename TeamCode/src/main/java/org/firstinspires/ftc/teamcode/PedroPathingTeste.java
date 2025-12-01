package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous
public class PedroPathingTeste extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // POSIÇÃO INICIAL-FINAL
        // DRIVE > ESTADO DE MOVIMENTO
        // ATIRAR > TENTATIVA DE PONTO

        POSICAOINICIAL_POSICAOATIRAR,
        CARREGAMENTO_ATIRAR
    }

    PathState pathState;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
    private final Pose feedPose = new Pose (56.000, 36.000, Math.toRadians(180));

    public void init(){

    }

    public void loop(){

    }
}
