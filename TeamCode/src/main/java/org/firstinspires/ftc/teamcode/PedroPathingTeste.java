package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class PedroPathingTeste extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // POSIÇÃO INICIAL-FINAL
        // DRIVE > ESTADO DE MOVIMENTO
        // ATIRAR > TENTATIVA DE PONTO

        POSICAOINICIAL_POSICAOATIRAR,
        CARREGAMENTO_ATIRAR,

    }

    PathState pathState;

    private final Pose startPose = new Pose(124, 122.88632619439869, Math.toRadians(36));
    private final Pose shootPose = new Pose (102.95881383855024, 102.00988467874795, Math.toRadians(0));
    private final Pose endPose = new Pose(102.95881383855024, 84, Math.toRadians(0));

    private PathChain driveStartPosShootPos;
    public void buildPath(){
        //Coordenadas para pose inicial > pose final
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case POSICAOINICIAL_POSICAOATIRAR:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.CARREGAMENTO_ATIRAR); // reseta o timer e faz um novo estado
                break;
            case CARREGAMENTO_ATIRAR:
                if (!follower.isBusy()){
                    // LOGICA DO SHOOTER
                    telemetry.addLine("Caminho 1 feito");
                }
                break;
            default:
                telemetry.addLine("Nenhum estado acontecendo");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init(){
        pathState = PathState.POSICAOINICIAL_POSICAOATIRAR;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPath();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Estado do caminho", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getX());
        telemetry.addData("direção", follower.getPose().getHeading());
        telemetry.addData("Tempo do Timer", pathTimer.getElapsedTimeSeconds());
    }
}
