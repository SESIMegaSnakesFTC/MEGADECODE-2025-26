package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    //private DcMotor shooter = null;
    //private DcMotor viper = null;

    // DEFININDO SERVOS
    //private Servo regulShooter = null;
    //private Servo baseShooter = null;

    // CONSTANTES DE CONTROLE
    public double velocidade = 1.0;
    public boolean rbPressionadoUltimoEstado = false;
    public boolean lbPressionadoUltimoEstado = false;


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
            driveMecanum();

            String statusFeeder = "FEEDER: Desligado";

            if (gamepad2.right_bumper)
            {
                feeder.setPower(0.8);
                statusFeeder = "FEEDER: Coletando";
            }
            else if (gamepad2.left_bumper)
            {
                feeder.setPower(-0.8);
                statusFeeder = "FEEDER: Retirando";
            }
            else
            {
                feeder.setPower(0.0);
            }

            // TELEMETRIA (AGORA EM ORDEM FIXA)
            telemetry.addLine(statusFeeder);
            telemetry.addData("Velocidade movimento atual", "%.3f", velocidade);

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

        // INICIANDO MOTORES E SERVOS MECANINSMOS
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        //shooter = hardwareMap.get(DcMotor.class, "shooter");
        //viper = hardwareMap.get(DcMotor.class, "viper1");
        //regulShooter = hardwareMap.get(Servo.class, "regulShooter");
        //baseShooter = hardwareMap.get(Servo.class, "baseShooter");

        // DIREÇÃO MOTORES MOVIMENTO
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // DIREÇÃO MOTORES MECANISMOS
        feeder.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setDirection(DcMotor.Direction.REVERSE);
        //viper.setDirection(DcMotor.Direction.REVERSE);

        // COMPORTAMENTO DOS MOTORES
        // CHASSI E VIPER FREIO DE MÃO
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FEEDER E SHOOTER
        //shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ENCODER MOTORES
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //viper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void driveMecanum() {
        double ft = -gamepad1.left_stick_y; // MOVIMENTO FRETE E TRÁS
        double lateral = gamepad1.left_stick_x; // MOVIMENTO LATERAL
        double giro = gamepad1.right_stick_x; // MOVIMENTO DE GIRO

        // Controles RB e LB para velocidade
        boolean rbPressionado = gamepad1.right_bumper;
        boolean lbPressionado = gamepad1.left_bumper;

        // RB - Diminui a velocidade
        if (rbPressionado && !rbPressionadoUltimoEstado){
            if (velocidade > 0.125) {
                velocidade = velocidade / 2;

                // Garantir que não passe de 0.125
                if (velocidade < 0.125) {
                    velocidade = 0.125;
                }
            }
        }
        rbPressionadoUltimoEstado = rbPressionado;

        // LB - Aumenta a velocidade
        if (lbPressionado && !lbPressionadoUltimoEstado) {
            if (velocidade < 1.0){
                velocidade = velocidade * 2;

                // Garantir que não passe de 1
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
}


