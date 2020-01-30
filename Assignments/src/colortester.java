import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.remote.ev3.RMISampleProvider;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

import java.awt.Color;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
// Eric Xu 730095427
// Loi Pham 730095087

@SuppressWarnings("unused")

public class colortester {

	public static void main(String[] args) {

		EV3MediumRegulatedMotor motorA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor motorB = new EV3MediumRegulatedMotor(MotorPort.B);
		
		EV3ColorSensor cs = new EV3ColorSensor(SensorPort.S4);
		
	    motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB}); //sync tire speed
	    motorA.setSpeed(500);
	    motorB.setSpeed(500);
	   
	    //START
	    Button.waitForAnyPress();
	    
		
	    //wall following
	    SampleProvider colorprovider = cs.getColorIDMode();
	  	float[] colorsample = new float[colorprovider.sampleSize()];
	    while(colorsample[0] != 2){
	    	motorA.forward();
	    	motorB.forward();
	    	colorprovider.fetchSample(colorsample, 0);
	    	//System.out.println(colorsample[0]);
	    	//Delay.msDelay(250);
	  	}
	    motorA.stop(true);
	    motorB.stop(true);
	    motorA.close();
	    motorB.close();
		
	}
}
