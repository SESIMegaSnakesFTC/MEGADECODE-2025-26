package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TESTE RODAS")
public class testeRodas extends LinearOpMode {

    // DEFININDO MOTORES DE MOVIMENTO
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    @Override
    public void runOpMode() {
        //Iniciar o hardware
        initconfigH();

        //Espera até começar
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            leftFront.setPower(0.15);
            sleep(2000);
            leftFront.setPower(0);

            leftBack.setPower(0.15);
            sleep(2000);
            leftBack.setPower(0);

            rightFront.setPower(0.15);
            sleep(2000);
            rightFront.setPower(0);

            rightBack.setPower(0.15);
            sleep(2000);
            rightBack.setPower(0);

        }
    }

    private void initconfigH() {
        // INICIANDO MOTORES DE MOVIMENTO
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        // DIREÇÃO MOTORES MOVIMENTO
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
    }
}
