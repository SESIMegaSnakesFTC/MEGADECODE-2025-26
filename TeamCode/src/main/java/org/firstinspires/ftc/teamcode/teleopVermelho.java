package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleoperado Vermelho")
public class teleopVermelho extends LinearOpMode
{

    // DEFININDO MOTORES DE MOVIMENTO
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    // DEFININDO MOTORES/SERVOS MECANISMO
    private DcMotor feeder = null;
    private DcMotorEx baseShooter = null;
    private DcMotor shooter = null;
    private Servo alavanca = null;

    // CONSTANTES DE CONTROLE
    public double velocidade = 1.0;
    public boolean rbPressionadoUltimoEstado = false;
    public boolean lbPressionadoUltimoEstado = false;

    // SHOOTER
    boolean shooterLigado = false;
    public boolean A_PressUltEst_G2 = false;
    public boolean X_PressUltEst_G2 = false;
    double[] velShooter = {-1.0, -0.85, -0.76};
    int indiceVel = 0;

    // VARIÁVEIS LIMELIGHT
    private Limelight3A limelight;
    private enum LastSeenDirection {LEFT, RIGHT, NONE}
    private LastSeenDirection lastSeen = LastSeenDirection.NONE;
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;
    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.5;


    @Override
    public void runOpMode()
    {
        telemetry.addData("Status: ", "Iniciando...");
        telemetry.update();

        //Iniciar o hardware
        initconfigH();

        telemetry.addData("Status: ", "INICIADO, #GOMEGA");
        telemetry.update();

        //Espera até começar
        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            gamepad1.setLedColor(0,128,0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0,128,0, Gamepad.LED_DURATION_CONTINUOUS);

            // 1. CHASSI MECANUM (Gamepad 1)
            driveMecanum();

            // 2. FEEDER (Gamepad 2)
            feederControl();

            // 3. SHOOTER MANUAL (Gamepad 2)
            ligarShooter();

            // 4. SHOOTER E LIMELIGHT (Gamepad 2)
            limelightShooterControl();



            // TELEMETRIA
            telemetry.addData("Velocidade movimento atual", "%.3f", velocidade);
            telemetry.addData("Modo Limelight", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
            telemetry.update();
        }
    }

    private void initconfigH()
    {
        // INICIANDO MOTORES DE MOVIMENTO
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // INICIANDO MOTORES MECANINSMOS
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");

        // INICIANDO SERVOS
        alavanca = hardwareMap.get(Servo.class, "alavanca");
        alavanca.setPosition(0);

        // INICIALIZAÇÃO DA LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); // VERMELHO

        // DIREÇÃO MOTORES MOVIMENTO
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // DIREÇÃO MOTORES MECANISMOS
        feeder.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // COMPORTAMENTO DOS MOTORES
        // CHASSI E VIPER FREIO DE MÃO
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // FEEDER E SHOOTER
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ENCODER MOTORES
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ============ GAMEPAD 1 ============
    // ========== DRIVE MECANUM ===========
    private void driveMecanum() {
        double ft = gamepad1.left_stick_y; // MOVIMENTO FRETE E TRÁS
        double lateral = -gamepad1.left_stick_x; // MOVIMENTO LATERAL
        double giro = -gamepad1.right_stick_x; // MOVIMENTO DE GIRO

        // CONTROLE LB E RB PARA VELOCIDADE
        boolean rbPressionado = gamepad1.right_bumper;
        boolean lbPressionado = gamepad1.left_bumper;

        // RB - DIMINUI
        if (rbPressionado && !rbPressionadoUltimoEstado){
            if (velocidade > 0.125) {
                velocidade = velocidade / 2;

                // NÃO DEIXA PASSAR DE 0.125
                if (velocidade < 0.125) {
                    velocidade = 0.125;
                }
            }
        }
        rbPressionadoUltimoEstado = rbPressionado;

        // LB - AUMENTA
        if (lbPressionado && !lbPressionadoUltimoEstado) {
            if (velocidade < 1.0){
                velocidade = velocidade * 2;

                // NÃO PASSA DE 1
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

    // ============ GAMEPAD 2 ============
    // ============  FEEDER  =============
    private void feederControl() {

        String statusFeeder = "FEEDER: Desligado";

        // MANTIDO NO GAMEPAD 2
        if (gamepad2.right_bumper) {
            feeder.setPower(1);
            statusFeeder = "FEEDER: Coletando";
        }
        else if (gamepad2.left_bumper) {
            feeder.setPower(-1);
            statusFeeder = "FEEDER: Retirando";
        }
        else {
            feeder.setPower(0.0);
        }

        telemetry.addLine(statusFeeder);
    }


    // ============ GAMEPAD 2 ============
    // ======= SHOOTER E LIMELIGHT ========
    private void limelightShooterControl() {

        // Alternar modo automático (GAMEPAD 2 - Right Trigger)
        boolean rtPressed = gamepad2.right_trigger > 0.8;
        if (rtPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = rtPressed;

        // Executa modo (Automático ou Manual)
        if (modoAutomatico) runAutomaticMode();
        else runManualMode();
    }



    // ============ GAMEPAD 2 ============
    // =========== MODO MANUAL ===========
    private void runManualMode() {
        baseShooter.setPower(gamepad2.left_stick_x * 0.5);
    }

    // =========== MODO AUTOMÁTICO ===========
    private void runAutomaticMode() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double rawTx = result.getTx();
            double tx = -rawTx;

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            // Cálculo da correção PID (Proporcional)
            double correction = tx * KP;
            // Limita a velocidade máxima de correção
            correction = Math.max(-0.45, Math.min(0.45, correction));

            baseShooter.setPower(correction);

            telemetry.addLine("=== AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Detectada");
            telemetry.addData("Tx", tx);

        } else {

            telemetry.addLine("=== AUTOMÁTICO ===");
            telemetry.addData("Status", "Tag Perdida");

            // Lógica de busca (gira na última direção vista)
            switch (lastSeen) {

                case LEFT:
                    baseShooter.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Esquerda");
                    break;

                case RIGHT:
                    baseShooter.setPower(-SEARCH_SPEED);
                    telemetry.addData("Buscando", "Direita");
                    break;

                case NONE:
                    baseShooter.setPower(SEARCH_SPEED);
                    telemetry.addData("Buscando", "Padrão");
                    break;
            }
        }
    }

    private void ligarShooter(){

        String statusShooter = "SHOOTER: DESLIGADO";

        // CLIQUE NO RB (GAMEPAD 2)
        boolean A_Press_G2 = gamepad2.a;

        if (A_Press_G2 && !A_PressUltEst_G2){
            shooterLigado = !shooterLigado; // SWITCH LIGA/DESLIGA
        }
        A_PressUltEst_G2 = A_Press_G2;

        // ALTERNAR ENTRE VELOCIDADES NO LB (GAMEPAD 2)
        boolean X_Press_G2 = gamepad2.x;

        if (X_Press_G2 && !X_PressUltEst_G2) {

            indiceVel = indiceVel + 1;

            if (indiceVel >= velShooter.length){
                indiceVel = 0;
            }
        }
        X_PressUltEst_G2 = X_Press_G2;

        // DEFININDO POTÊNCIAS DO SHOOTER
        if (shooterLigado) {
            shooter.setPower(velShooter[indiceVel]);
            statusShooter = "SHOOTER: LIGADO | VELOCIDADE: " + velShooter[indiceVel];

            if (gamepad2.dpad_up){
                alavanca.setPosition(0.56);
            }
            if (gamepad2.dpad_down){
                alavanca.setPosition(0);
            }
        } else {
            alavanca.setPosition(0);
            shooter.setPower(0);
            statusShooter = "SHOOTER: DESLIGADO";
        }

        telemetry.addLine(statusShooter);
    }
}