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

import java.util.Arrays;

@TeleOp(name = "TELEOPERADO VERMELHO")
public class teleopVermelho extends LinearOpMode {

    // ===================== MOTORES DE MOVIMENTO =====================
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    // ===================== MECANISMOS =====================
    private DcMotor feeder = null;
    private DcMotorEx baseShooter = null;
    private DcMotor shooter = null;
    private Servo alavanca = null;

    // ===================== CONTROLE DE VELOCIDADE =====================
    private final double[] VELOCIDADES_CHASSI = {0.130, 0.300, 0.500, 0.850};
    private int indiceVelocidade = VELOCIDADES_CHASSI.length - 1;

    public boolean rbPressionadoUltimoEstado = false;
    public boolean lbPressionadoUltimoEstado = false;

    // ===================== SHOOTER =====================
    boolean shooterLigado = false;
    public boolean A_PressUltEst_G2 = false;
    public boolean X_PressUltEst_G2 = false;

    double[] velShooter = {-1.0, -0.85, -0.76};
    int indiceVel = 0;

    // ===================== LIMELIGHT =====================
    private Limelight3A limelight;

    private enum LastSeenDirection {
        LEFT, RIGHT, NONE
    }

    private LastSeenDirection lastSeen = LastSeenDirection.NONE;
    private boolean modoAutomatico = false;
    private boolean lastTriggerPressed = false;

    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.25;

    private static final int TICKS_PER_OUTPUT_REV = 1120;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_OUTPUT_REV;

    // ===================== LIMITES DE ÂNGULO =====================
    private static final double MAX_ANGLE = 180.0;
    private static final double MIN_ANGLE = -180.0;

    private boolean sweepToRight = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciando...");
        telemetry.update();

        initconfigH();

        telemetry.addData("Status", "INICIADO, #GOMEGA");
        telemetry.update();

        waitForStart();
        resetRuntime();

        Arrays.sort(VELOCIDADES_CHASSI);

        while (opModeIsActive()) {
            gamepad1.setLedColor(0, 128, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 128, 0, Gamepad.LED_DURATION_CONTINUOUS);

            // CHASSI
            driveMecanum();

            // FEEDER
            feederControl();

            // SHOOTER
            ligarShooter();

            // LIMELIGHT + BASE DO SHOOTER
            limelightShooterControl();

            double velocidadeAtual = VELOCIDADES_CHASSI[indiceVelocidade];

            telemetry.addData("Velocidade movimento atual", "%.3f", velocidadeAtual);
            telemetry.addData("Modo Limelight", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
            telemetry.update();
        }
    }

    // ===================== HARDWARE =====================
    private void initconfigH() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        feeder = hardwareMap.get(DcMotor.class, "feeder");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        baseShooter = hardwareMap.get(DcMotorEx.class, "baseShooter");

        alavanca = hardwareMap.get(Servo.class, "alavancaServo");
        alavanca.setPosition(0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); // VERMELHO

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        feeder.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===================== DRIVE MECANUM =====================
    private void driveMecanum() {
        double ft = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double giro = -gamepad1.right_stick_x;

        double velocidade = VELOCIDADES_CHASSI[indiceVelocidade];

        boolean rbPressionado = gamepad1.right_bumper;
        boolean lbPressionado = gamepad1.left_bumper;

        if (rbPressionado && !rbPressionadoUltimoEstado && indiceVelocidade > 0) {
            indiceVelocidade--;
        }
        rbPressionadoUltimoEstado = rbPressionado;

        if (lbPressionado && !lbPressionadoUltimoEstado && indiceVelocidade < VELOCIDADES_CHASSI.length - 1) {
            indiceVelocidade++;
        }
        lbPressionadoUltimoEstado = lbPressionado;

        double frontLeftPower = (ft + lateral + giro) * velocidade;
        double frontRightPower = (ft - lateral - giro) * velocidade;
        double backLeftPower = (ft - lateral + giro) * velocidade;
        double backRightPower = (ft + lateral - giro) * velocidade;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 0.850) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    // ===================== FEEDER =====================
    private void feederControl() {
        String statusFeeder = "FEEDER: Desligado";

        if (gamepad2.right_bumper) {
            feeder.setPower(1);
            statusFeeder = "FEEDER: Coletando";
        } else if (gamepad2.left_bumper) {
            feeder.setPower(-1);
            statusFeeder = "FEEDER: Retirando";
        } else {
            feeder.setPower(0);
        }

        telemetry.addLine(statusFeeder);
    }

    // ===================== LIMELIGHT =====================
    private void limelightShooterControl() {
        boolean rtPressed = gamepad2.right_trigger > 0.8;

        if (rtPressed && !lastTriggerPressed) {
            modoAutomatico = !modoAutomatico;
        }
        lastTriggerPressed = rtPressed;

        if (modoAutomatico) {
            runAutomaticMode();
        } else {
            runManualMode();
        }
    }

    private void runManualMode() {
        double stick = gamepad2.left_stick_x;
        double power = stick * 0.5;
        double angle = getAngleDegrees();

        if ((power > 0 && angle >= MAX_ANGLE) || (power < 0 && angle <= MIN_ANGLE)) {
            baseShooter.setPower(0);
            return;
        }

        baseShooter.setPower(power);
    }

    private void runAutomaticMode() {
        LLResult result = limelight.getLatestResult();
        double angle = getAngleDegrees();

        if (result != null && result.isValid()) {
            double tx = -result.getTx();

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            double correction = -tx * KP;

            if ((correction > 0 && angle >= MAX_ANGLE) || (correction < 0 && angle <= MIN_ANGLE)) {
                correction = 0;
            }

            baseShooter.setPower(correction);
            return;
        }

        double searchPower = sweepToRight ? SEARCH_SPEED : -SEARCH_SPEED;

        if (sweepToRight && angle >= MAX_ANGLE) {
            sweepToRight = false;
            searchPower = -SEARCH_SPEED;
        } else if (!sweepToRight && angle <= MIN_ANGLE) {
            sweepToRight = true;
            searchPower = SEARCH_SPEED;
        }

        baseShooter.setPower(searchPower);
    }

    private double getAngleDegrees() {
        return baseShooter.getCurrentPosition() * DEGREES_PER_TICK;
    }

    // ===================== SHOOTER =====================
    private void ligarShooter() {
        String statusShooter = "SHOOTER: DESLIGADO";

        boolean A_Press_G2 = gamepad2.a;

        if (A_Press_G2 && !A_PressUltEst_G2) {
            shooterLigado = !shooterLigado;
        }
        A_PressUltEst_G2 = A_Press_G2;

        boolean X_Press_G2 = gamepad2.x;

        if (X_Press_G2 && !X_PressUltEst_G2) {
            indiceVel = (indiceVel + 1) % velShooter.length;
        }
        X_PressUltEst_G2 = X_Press_G2;

        if (shooterLigado) {
            shooter.setPower(velShooter[indiceVel]);
            statusShooter = "SHOOTER: LIGADO | VELOCIDADE: " + velShooter[indiceVel];

            if (gamepad2.dpad_up) {
                alavanca.setPosition(0.56);
            }
            if (gamepad2.dpad_down) {
                alavanca.setPosition(0);
            }
        } else {
            alavanca.setPosition(0);
            shooter.setPower(0);
        }

        telemetry.addLine(statusShooter);
    }
}
