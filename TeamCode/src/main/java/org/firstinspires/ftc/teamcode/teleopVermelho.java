package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// IMPORTS DO PEDRO PATHING
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Teleoperado Vermelho")
public class teleopVermelho extends LinearOpMode {

    // --- MOTORES CHASSI ---
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    // --- MOTORES MECANISMOS ---
    private DcMotor feeder = null;

    // --- CONTROLES ---
    public double velocidade = 1.0;
    public boolean rbPressionadoUltimoEstado = false;
    public boolean lbPressionadoUltimoEstado = false;

    // --- PEDRO PATHING ---
    private Follower follower;

    // Variáveis fixas de posição
    private final double shootX = 96;
    private final double shootY = 95;
    private final double goalX = 126;
    private final double goalY = 131;

    // Variável do cálculo
    double DistanciaPosShootPosGoal = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status: ", "Iniciando...");
        telemetry.update();

        initconfigH();

        // --- INICIALIZAÇÃO DO SEGUIDOR PEDRO PATHING ---
        follower = Constants.createFollower(hardwareMap);

        telemetry.addData("Status: ", "INICIADO, #GOMEGA");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            driveMecanum();
            feederControl();

            // *********************************************
            // Quando o botão X é pressionado no gamepad1:
            // *********************************************
            if (gamepad1.x) {

                // --- POSE ATUAL DO ROBÔ ---
                Pose poseAtual = follower.getPose();

                // --- CRIAÇÃO DO CAMINHO PARA A POSIÇÃO DE SHOOT ---
                PathChain caminho = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new com.pedropathing.geometry.Pose(
                                        poseAtual.getX(),
                                        poseAtual.getY(),
                                        poseAtual.getHeading()
                                ),
                                new com.pedropathing.geometry.Pose(
                                        shootX,
                                        shootY,
                                        poseAtual.getHeading()
                                )
                        ))
                        .setLinearHeadingInterpolation(
                                poseAtual.getHeading(),
                                poseAtual.getHeading()
                        )
                        .build();

                // --- EXECUTA CAMINHO ---
                follower.followPath(caminho, false);

                // --- TRIGONOMETRIA PARA CALCULAR A DISTÂNCIA ENTRE SHOOT E GOAL ---
                double catetoX = goalX - shootX;
                double catetoY = goalY - shootY;

                DistanciaPosShootPosGoal = Math.sqrt(catetoX * catetoX + catetoY * catetoY);
            }

            // Atualiza o seguidor SEM parar o teleop
            follower.update();

            // TELEMETRIA
            telemetry.addData("Velocidade movimento atual", "%.3f", velocidade);
            telemetry.addData("Distância Shoot → Goal", DistanciaPosShootPosGoal);
            telemetry.update();
        }
    }

    // =========================================
    // =========== CONFIGS HARDWARE ============
    // =========================================
    private void initconfigH() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        feeder = hardwareMap.get(DcMotor.class, "feeder");

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        feeder.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // =========================================
    // =========== MOVIMENTAÇÃO =================
    // =========================================
    private void driveMecanum() {

        double ft = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double giro = gamepad1.right_stick_x;

        boolean rbPressionado = gamepad1.right_bumper;
        boolean lbPressionado = gamepad1.left_bumper;

        if (rbPressionado && !rbPressionadoUltimoEstado) {
            if (velocidade > 0.125) {
                velocidade /= 2.0;
                if (velocidade < 0.125) velocidade = 0.125;
            }
        }
        rbPressionadoUltimoEstado = rbPressionado;

        if (lbPressionado && !lbPressionadoUltimoEstado) {
            if (velocidade < 1.0) {
                velocidade *= 2;
                if (velocidade > 1.0) velocidade = 1.0;
            }
        }
        lbPressionadoUltimoEstado = lbPressionado;

        double fl = (ft + lateral + giro) * velocidade;
        double fr = (ft - lateral - giro) * velocidade;
        double bl = (ft - lateral + giro) * velocidade;
        double br = (ft + lateral - giro) * velocidade;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }

    // =========================================
    // =========== FEEDER =======================
    // =========================================
    private void feederControl() {

        String statusFeeder = "FEEDER: Desligado";

        if (gamepad2.right_bumper) {
            feeder.setPower(1); //usar 0.8
            statusFeeder = "FEEDER: Coletando";
        }
        else if (gamepad2.left_bumper) {
            feeder.setPower(-1); //usar 0.8
            statusFeeder = "FEEDER: Retirando";
        }
        else {
            feeder.setPower(0.0);
        }

        telemetry.addLine(statusFeeder);
    }
}
