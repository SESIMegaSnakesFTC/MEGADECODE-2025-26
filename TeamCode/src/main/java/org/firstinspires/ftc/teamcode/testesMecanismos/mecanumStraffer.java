package org.firstinspires.ftc.teamcode.testesMecanismos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="Mecanum Straffer v5")
public class mecanumStraffer extends OpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor feeder = null;
    public double velocidade = 1.0;
    public boolean rtPressionadoUltimoEstado = false;

    @Override
    public void init() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // (o valor é invertido porque empurrar o analógico para frente gera valor negativo)
        double ft = -gamepad1.left_stick_y; // frente/trás
        double lateral = gamepad1.left_stick_x; // movimento lateral
        double giro = gamepad1.right_stick_x; // rotação
        boolean rtPressionado = gamepad1.right_trigger > 0.8;

        if (rtPressionado && !rtPressionadoUltimoEstado) {
            if (velocidade == 1.0) {
                velocidade = 0.5;
            } else if (velocidade == 0.5) {
                velocidade = 0.25;
            } else if (velocidade == 0.25) {
                velocidade = 0.125;
            } else {
                velocidade = 1.0;
            }
        }
        rtPressionadoUltimoEstado = rtPressionado;

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

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        if (gamepad1.right_bumper){
            feeder.setPower(0.8); // Ativa rotação total
        }
        else if (gamepad1.left_bumper) {
            feeder.setPower(-0.8);
        }
        else {
            feeder.setPower(0.0); // Desativa o motor
        }
    }
}