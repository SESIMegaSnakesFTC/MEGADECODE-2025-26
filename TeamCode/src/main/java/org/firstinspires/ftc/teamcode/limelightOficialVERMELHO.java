package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Disabled
@TeleOp(name = "Teste Limelight 180")
public class limelightOficialVERMELHO extends OpMode {

    private Limelight3A limelight;
    private DcMotor baseShooter;    // motor principal (pan) COM encoder
    private DcMotor shooter;        // motor auxiliar (toggle com A)
    private CRServo servoE;
    private CRServo servoD;

    private enum LastSeenDirection { LEFT, RIGHT, NONE }
    private LastSeenDirection lastSeen = LastSeenDirection.NONE;


    private boolean modoAutomatico = false;
    private boolean lastRtPressed = false;
    private final double RT_THRESHOLD = 0.6;


    private boolean shooterLigado = false;
    private boolean lastA = false;

    // --- controle ---
    private static final double KP = 0.02;
    private static final double SEARCH_SPEED = 0.3;
    private static final double MANUAL_LIMIT = 0.6;

    // --- encoder / redução ---
    private static final int TICKS_PER_OUTPUT_REV = 1120; // 28 x proporção
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_OUTPUT_REV;

    // limites angulares
    private static final double MAX_ANGLE = 180.0;
    private static final double MIN_ANGLE = -180.0;

    // direção atual da varredura
    private boolean sweepToRight = true;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        baseShooter = hardwareMap.get(DcMotor.class, "baseShooter");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        servoE = hardwareMap.get(CRServo.class, "servoE");
        servoD = hardwareMap.get(CRServo.class, "servoD");

        baseShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight.start();
        limelight.pipelineSwitch(0);

        baseShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        baseShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Init completo — encoder resetado (baseShooter = 0°)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // toggle automático via RT
        boolean rtPressed = gamepad1.right_trigger > RT_THRESHOLD;
        if (rtPressed && !lastRtPressed) modoAutomatico = !modoAutomatico;
        lastRtPressed = rtPressed;

        // toggle shooter via A
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastA) shooterLigado = !shooterLigado;
        lastA = aPressed;

        shooter.setPower(shooterLigado ? 1.0 : 0.0);

        // servos de antes
        double servoPower = gamepad1.left_stick_x * 0.5;
        servoE.setPower(servoPower);
        servoD.setPower(-servoPower);

        // modos
        if (modoAutomatico) runAutomaticMode();
        else runManualMode();

        // telemetria
        telemetry.addData("Modo", modoAutomatico ? "AUTOMÁTICO" : "MANUAL");
        telemetry.addData("Angle (°)", "%.2f", getAngleDegrees());
        telemetry.addData("Ticks", baseShooter.getCurrentPosition());
        telemetry.addData("LastSeen", lastSeen.toString());
        telemetry.addData("MotorExtra", shooterLigado ? "ON" : "OFF");
        telemetry.update();
    }

    private double getAngleDegrees() {
        return baseShooter.getCurrentPosition() * DEGREES_PER_TICK;
    }

    //      MODO MANUAL
    private void runManualMode() {

        double stick = gamepad1.right_stick_x;
        double power = stick * MANUAL_LIMIT;

        double angle = getAngleDegrees();

        if (power > 0 && angle >= MAX_ANGLE) {
            baseShooter.setPower(0);
            return;
        }

        if (power < 0 && angle <= MIN_ANGLE) {
            baseShooter.setPower(0);
            return;
        }

        baseShooter.setPower(power);
    }

    //    MODO AUTOMÁTICO
    private void runAutomaticMode() {

        LLResult result = limelight.getLatestResult();
        double angle = getAngleDegrees();

        if (result != null && result.isValid()) {

            double tx = -result.getTx(); // limelight invertida

            if (tx > 0) lastSeen = LastSeenDirection.RIGHT;
            else if (tx < 0) lastSeen = LastSeenDirection.LEFT;

            // controle proporcional simples (único KP)
            double correction = -tx * KP;

            if (correction > 0 && angle >= MAX_ANGLE) correction = 0;
            if (correction < 0 && angle <= MIN_ANGLE) correction = 0;

            baseShooter.setPower(correction);
            return;
        }

        // --- MODO BUSCA (swipe automático) ---
        double searchPower = sweepToRight ? SEARCH_SPEED : -SEARCH_SPEED;

        if (sweepToRight && angle >= MAX_ANGLE) {
            sweepToRight = false;
            searchPower = -SEARCH_SPEED;
        }
        else if (!sweepToRight && angle <= MIN_ANGLE) {
            sweepToRight = true;
            searchPower = SEARCH_SPEED;
        }

        baseShooter.setPower(searchPower);
    }
}
