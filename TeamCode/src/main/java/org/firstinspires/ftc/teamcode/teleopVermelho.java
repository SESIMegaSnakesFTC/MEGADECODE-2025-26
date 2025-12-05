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
public class teleopVermelho extends LinearOpMode
{

    // DEFININDO MOTORES DE MOVIMENTO
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    // DEFININDO MOTORES MECANISMO
    private DcMotor feeder = null;
    private DcMotorEx baseShooter = null;
    private DcMotor shooter = null;


    // DEFININDO SERVOS
    //private Servo regulShooter1 = null;
    //private Servo regulShooter2 = null;

    // CONSTANTES DE CONTROLE
    public double velocidade = 1.0;

    // VARIÁVEIS DE CONTROLE
    // VELOCIDADE CHASSI
    public boolean rbPressUltEst_G1 = false;
    public boolean lbPressUltEst_G1 = false;

    // SHOOTER
    boolean shooterLigado = false;
    public boolean rbPressUltEst_G2 = false;
    public boolean lbPressUltEst_G2 = false;
    double[] velShooter = {1.0, 0.8, 0.6, 0.4};
    int indiceVel = 0;

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

        //Iniciar hardwares
        initconfigH();

        telemetry.addData("Status: ", "INICIADO, #GOMEGA");
        telemetry.update();

        // --- INICIALIZAÇÃO DO SEGUIDOR PEDRO PATHING ---
        follower = Constants.createFollower(hardwareMap);

        // Espera até começar
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            driveMecanum();
            controleFeeder();
            ligarShooter();

            // Quando o botão X é pressionado no gamepad1:
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

    // =========== CONFIGS HARDWARE ============
    private void initconfigH() {

        // INICIANDO MOTORES DE MOVIMENTO
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // INICIANDO MOTORES E SERVOS MECANINSMOS
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");
        //regulShooter1 = hardwareMap.get(Servo.class, "regulShooter1");
        //regulShooter2 = hardwareMap.get(Servo.class, "regulShooter2");

        // DIREÇÃO MOTORES MOVIMENTOS
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // DIREÇÃO MOTORES MECANISMOS
        feeder.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        baseShooter.setDirection(DcMotorEx.Direction.FORWARD);

        // COMPORTAMENTO DOS MOTORES
        // CHASSI E BASE SHOOTER FREIO DE MÃO
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        baseShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // FEEDER E SHOOTER
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ENCODER MOTORES
        // SEM ENCODER
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RUN TO POSITION
        baseShooter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void driveMecanum() {

        double ft = -gamepad1.left_stick_y; // MOVIMENTO FRENTE E TRÁS
        double lateral = gamepad1.left_stick_x; // MOVIMENTO LATERAL
        double giro = gamepad1.right_stick_x; // MOVIMENTO DE GIRO

        // CONTROLES RB E LB PARA VELOCIDADE
        boolean rbPress_G1 = gamepad1.right_bumper;
        boolean lbPress_G1 = gamepad1.left_bumper;

        // RB DIMINUI A VELOCIDADE
        if (rbPress_G1 && !rbPressUltEst_G1) {
            if (velocidade > 0.125) {
                velocidade = velocidade / 2;

                // GARANTIR QUE NÃO PASSE DE 0.125
                if (velocidade < 0.125) {
                    velocidade = 0.125;
                }
            }
        }
        rbPressUltEst_G1 = rbPress_G1;

        // LB AUMENTA A VELOCIDADE
        if (lbPress_G1 && !lbPressUltEst_G1) {
            if (velocidade < 1.0) {
                velocidade = velocidade * 2;

                // GARANTIR QUE NÃO PASSE DE 1
                if (velocidade > 1.0){
                    velocidade = 1.0;
                }
            }
        }
        lbPressUltEst_G1 = lbPress_G1;

        double frontLeftPower  = (ft + lateral + giro) * velocidade;
        double frontRightPower = (ft - lateral - giro) * velocidade;
        double backLeftPower   = (ft - lateral + giro) * velocidade;
        double backRightPower  = (ft + lateral - giro) * velocidade;

        // Garante que nenhuma potência ultrapasse 1.0 (limite máximo do motor)
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  = frontLeftPower / max;
            frontRightPower = frontRightPower / max;
            backLeftPower   = backLeftPower / max;
            backRightPower  = backRightPower / max;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    // ============== FEEDER ======================
    // ============= GAMEPAD 2 ================
    private void controleFeeder() {

        String statusFeeder = "FEEDER: Desligado";

        if (gamepad2.a) {
            feeder.setPower(1); //usar 0.8
            statusFeeder = "FEEDER: Coletando";
        }
        else if (gamepad2.y) {
            feeder.setPower(-1); //usar 0.8
            statusFeeder = "FEEDER: Retirando";
        }
        else {
            feeder.setPower(0.0);
        }

        telemetry.addLine(statusFeeder);
    }

    private void ligarShooter(){

        String statusShooter = "SHOOTER: DESLIGADO";

        // CLIQUE NO RB
        boolean rbPress_G2 = gamepad2.right_bumper;

        if (rbPress_G2 && !rbPressUltEst_G2){
            shooterLigado = !shooterLigado; // SWITCH LIGA/DESLIGA
        }
        rbPressUltEst_G2 = rbPress_G2;

        // ALTERNAR ENTRE VELOCIDADES NO LB
        boolean lbPress_G2 = gamepad2.left_bumper;

        if (lbPress_G2 && !lbPressUltEst_G2) {

            indiceVel = indiceVel + 1;

            if (indiceVel >= velShooter.length){
                indiceVel = 0;
            }
        }
        lbPressUltEst_G2 = lbPress_G2;

        // DEFININDO POTÊNCIAS DO SHOOTER
        if (shooterLigado) {
            shooter.setPower(velShooter[indiceVel]);
            statusShooter = "SHOOTER: LIGADO | VELOCIDADE: " + velShooter[indiceVel];
        } else {
            shooter.setPower(0);
            statusShooter = "SHOOTER: DESLIGADO";
        }

        telemetry.addLine(statusShooter);
    }

    //private void
}
