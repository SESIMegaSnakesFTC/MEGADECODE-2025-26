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
    //private DcMotor shooter = null;


    // DEFININDO SERVOS
    //private Servo regulShooter1 = null;
    //private Servo regulShooter2 = null;

    // CONSTANTES DE CONTROLE
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

        // INICIANDO MOTORES DE MOVIMENTO
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // INICIANDO MOTORES E SERVOS MECANINSMOS
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        //shooter = hardwareMap.get(DcMotor.class, "shooter");
        //baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");
        //regulShooter1 = hardwareMap.get(Servo.class, "regulShooter1");
        //regulShooter2 = hardwareMap.get(Servo.class, "regulShooter2");

        // DIREÇÃO MOTORES MOVIMENTOS
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // DIREÇÃO MOTORES MECANISMOS
        feeder.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setDirection(DcMotor.Direction.REVERSE);
        //baseShooter.setDirection(DcMotorEx.Direction.FORWARD);

        // COMPORTAMENTO DOS MOTORES
        // CHASSI E BASE SHOOTER FREIO DE MÃO
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //baseShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // FEEDER E SHOOTER
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ENCODER MOTORES
        // SEM ENCODER
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RUN TO POSITION
        //baseShooter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void driveMecanum() {

        double ft = -gamepad1.left_stick_y; // MOVIMENTO FRENTE E TRÁS
        double lateral = gamepad1.left_stick_x; // MOVIMENTO LATERAL
        double giro = gamepad1.right_stick_x; // MOVIMENTO DE GIRO

        // CONTROLES RB E LB PARA VELOCIDADE
        boolean rbPressionado = gamepad1.right_bumper;
        boolean lbPressionado = gamepad1.left_bumper;

        // RB DIMINUI A VELOCIDADE
        if (rbPressionado && !rbPressionadoUltimoEstado) {
            if (velocidade > 0.125) {
                velocidade = velocidade / 2;

                // GARANTIR QUE NÃO PASSE DE 0.125
                if (velocidade < 0.125) {
                    velocidade = 0.125;
                }
            }
        }
        rbPressionadoUltimoEstado = rbPressionado;

        // LB AUMENTA A VELOCIDADE
        if (lbPressionado && !lbPressionadoUltimoEstado) {
            if (velocidade < 1.0) {
                velocidade = velocidade * 2;

                // GARANTIR QUE NÃO PASSE DE 1
                if (velocidade > 1.0){
                    velocidade = 1.0;
                }
            }
        }
        lbPressionadoUltimoEstado = lbPressionado;

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
