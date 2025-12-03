package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous
public class AutonomoP1Vermelho extends OpMode {

    // Criando objetos
    private DcMotorEx feeder;
    private Follower follower;
    private Paths paths;
    private Timer pathTimer;
    private PathState pathState;

    public enum PathState{
        AJUSTANDO_FEEDING,
        FEEDING,
        AJUSTE_EMPURRAR,
        EMPURRAR,
        AJUSTE_PAREDE,
        DESCE_SHOOT1,
        FEEDING_BASE,
        DESCE_SHOOT2,
        FIM
    }

    public static class Paths{
        public PathChain AJUSTANDO_FEEDING;
        public PathChain FEEDING;
        public PathChain AJUSTE_EMPURRAR;
        public PathChain EMPURRAR;
        public PathChain AJUSTE_PAREDE;
        public PathChain DESCE_SHOOT1;
        public PathChain FEEDING_BASE;
        public PathChain DESCE_SHOOT2;

        public Paths(Follower follower){

            AJUSTANDO_FEEDING = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000,9.000), //POSIÇÃO INICIAL
                            new Pose(56.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(180)
                    )
                    .build();

            FEEDING = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 60.000),
                            new Pose(11.000, 60.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            AJUSTE_EMPURRAR = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.000,60.000),
                            new Pose(32.430, 47.919)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(153),
                            Math.toRadians(153)
                    )
                    .build();

            EMPURRAR = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(32.430, 47.919),
                            new Pose(12.343, 14.763)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(140),
                            Math.toRadians(140))
                    .build();

            AJUSTE_PAREDE = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.343, 14.763),
                            new Pose(12.343, 9.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            DESCE_SHOOT1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.343, 9.000),
                            new Pose(56.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            FEEDING_BASE = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 9.000),
                            new Pose(11.100, 9.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            DESCE_SHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.100, 9.000),
                            new Pose(56.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();
        }
    }

    @Override
    public void init(){

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();
        pathState = PathState.AJUSTANDO_FEEDING;

        // POSIÇÃO INICIAL
        follower.setPose(
                new Pose(56.000,9.000)
        );
    }

    @Override
    public void start(){
        pathTimer.resetTimer();
    }

    private void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void statePathUpdate() {
        // MÁQUINA DE ESTADO
        switch(pathState){

            case AJUSTANDO_FEEDING:
                follower.setMaxPower(1);
                follower.followPath(paths.AJUSTANDO_FEEDING, true);
                setPathState(PathState.FEEDING);
                telemetry.addLine("AJUSTE FEED - OK");
                break;

            case FEEDING:
                if(!follower.isBusy()){
                    feeder.setPower(-1);

                    follower.setMaxPower(0.6);
                    follower.followPath(paths.FEEDING, true);
                    setPathState(PathState.AJUSTE_EMPURRAR);
                    telemetry.addLine("FEEDING - OK");
                }
                break;

            case AJUSTE_EMPURRAR:
                if(!follower.isBusy()){
                    //follower.setMaxPower(0.3);
                    follower.followPath(paths.AJUSTE_EMPURRAR, true);
                    setPathState(PathState.AJUSTE_EMPURRAR);
                    telemetry.addLine("AJUSTE PARA EMPURRAR - OK");
                }
                feeder.setPower(0);
                break;

            case EMPURRAR:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.EMPURRAR, true);
                    setPathState(PathState.AJUSTE_PAREDE);
                    telemetry.addLine("EMPURRAR - OK");
                }
                break;

            case AJUSTE_PAREDE:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.AJUSTE_PAREDE, true);
                    setPathState(PathState.DESCE_SHOOT1);
                    telemetry.addLine("AJUSTE NA PAREDE - OK");
                }
                break;

            case DESCE_SHOOT1:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.DESCE_SHOOT1, true);
                    setPathState(PathState.FEEDING_BASE);
                    telemetry.addLine("VOLTAR PARA ATIRAR 1 - OK");
                }
                break;

            case FEEDING_BASE:
                if(!follower.isBusy()){
                    feeder.setPower(-1);

                    follower.setMaxPower(0.6);
                    follower.followPath(paths.FEEDING_BASE, true);
                    setPathState(PathState.DESCE_SHOOT2);
                    telemetry.addLine("FEEDING DA BASE - OK");
                }
                break;

            case DESCE_SHOOT2:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.DESCE_SHOOT2);
                    setPathState(PathState.FIM);
                    telemetry.addLine("VOLTAR PARA ATIRAR 2 - OK");
                }
                feeder.setPower(0);
                break;

            case FIM:
                break;

            default:
                telemetry.addLine("NENHUM CAMINHO RODANDO");
                break;
        }
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("Estado", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Tempo Estado", pathTimer.getElapsedTimeSeconds());
    }
}
