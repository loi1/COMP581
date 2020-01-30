import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.lcd.LCD;
import java.util.ArrayList;

public class lab4_takoda {
    private static final double wheelRadius = .028; // cm
	private static final float L = .127f;
	public static float xCoordinate, yCoordinate;
	public static double heading;
	public static double startTime;
	
	public static EV3MediumRegulatedMotor mA;
	public static EV3MediumRegulatedMotor mB;
	
	public static void main(String[] args){
		mA = new EV3MediumRegulatedMotor(MotorPort.A);
		mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3UltrasonicSensor usltrasonic = new EV3UltrasonicSensor(SensorPort.S4);
		EV3TouchSensor touchLeft = new EV3TouchSensor(SensorPort.S1);
		EV3TouchSensor touchRight = new EV3TouchSensor(SensorPort.S2); 
		
		mA.synchronizeWith(new EV3MediumRegulatedMotor[]{mB});
		
		SensorMode ultrasonicSensor = (SensorMode) usltrasonic.getDistanceMode();
		SensorMode touchSensorLeft = touchLeft.getTouchMode();
		SensorMode touchSensorRight = touchRight.getTouchMode();
		
		float[] ultrasonicSamples = new float[ultrasonicSensor.sampleSize()];
		float[] touchSamplesLeft = new float[touchSensorLeft.sampleSize()];
		float[] touchSamplesRight = new float[touchSensorRight.sampleSize()];
		
		mA.setSpeed(200);
		mB.setSpeed(200);
		
		Button.ENTER.waitForPressAndRelease();
		
		touchLeft.fetchSample(touchSamplesLeft, 0);
		touchRight.fetchSample(touchSamplesRight, 0);
		long time = System.currentTimeMillis();
		
		while(touchSamplesLeft[0] == 0 && touchSamplesRight[0] == 0){
			forward(mA, mB);
			touchLeft.fetchSample(touchSamplesLeft, 0);
			touchRight.fetchSample(touchSamplesRight, 0);
		}
		time = System.currentTimeMillis() - time;
		
		stop(mA, mB);
		Sound.beep();
		backward(mA, mB);
		Delay.msDelay(1500);
		stop(mA, mB);
		
		mA.rotate(380);

		Delay.msDelay(500);
		
		xCoordinate = 0; yCoordinate = 0; heading = Math.PI / 2.0;
		
		ultrasonicSensor.fetchSample(ultrasonicSamples,0);
		float distance = ultrasonicSamples[0];
		float desiredDistance = .15f;
		
		float sampleVal;
		ultrasonicSensor.fetchSample(ultrasonicSamples, 0);
		sampleVal = ultrasonicSamples[0];
		ArrayList<Float> running = new ArrayList<Float>(10);
		running.add(sampleVal);
		int count = 0;
	
		startTime = System.nanoTime();
		while(distance < .5){
			LCD.drawInt((int)(heading * 100), 0, 1);
			LCD.drawInt((int)(100 * xCoordinate), 0, 2);
			LCD.drawInt((int)(100 * yCoordinate), 0, 3);
			count++;
			LCD.clear();

			
			float dist = .3f;
			if(count > 10000 && xCoordinate > -dist && xCoordinate < dist && yCoordinate > -dist && yCoordinate < dist){
				break;
			}
			ArrayList<Float> goodReadings = new ArrayList<Float>(5);
			float[] reads = new float[5];
			float lastReading = running.get(running.size()-1);
			boolean gotSample = false;
			for (int i = 0; i < 5; i++) {
				ultrasonicSensor.fetchSample(ultrasonicSamples, 0);
				sampleVal = ultrasonicSamples[0];
				reads[i] = sampleVal;
				if((lastReading - 0.1) < sampleVal && (lastReading + 0.1) > sampleVal && sampleVal != Float.POSITIVE_INFINITY) {
					gotSample = true;
					goodReadings.add(sampleVal);
				}
			}
			if(gotSample == false) {
				touchLeft.fetchSample(touchSamplesLeft, 0);
				if(touchSamplesLeft[0] == 1) {
					backAndTurn();
					float avg = getSamples(ultrasonicSensor, ultrasonicSamples);
					running.add(avg);
				}
			} else {
				touchLeft.fetchSample(touchSamplesLeft, 0);
				if(touchSamplesLeft[0] == 1) {
					backAndTurn();
					float avg = getSamples(ultrasonicSensor, ultrasonicSamples);
					running.add(avg);
				}
				else {
					float total = 0.0f;
					for (int i = 0; i < goodReadings.size(); i++) {
						total += goodReadings.get(i);
					}
					float avg = total / (float) goodReadings.size();
					distance = avg;
					running.add(avg);
					
					if(distance > desiredDistance){
						moveLeft(mA, mB);
						calculate(mA, mB);
					} else if (distance < desiredDistance){
						moveRight(mA, mB);
						calculate(mA, mB);
					} else {
						equalMove(mA, mB);
						calculate(mA, mB);
					}
				}
				
			}
			
		}
		stop(mA, mB);
		
		mA.rotate(360);
		
		Delay.msDelay(500);
		
		mA.setSpeed(200);
		mB.setSpeed(200);
		forward(mA, mB);
		Delay.msDelay(time);
		stop(mA, mB);
		usltrasonic.close();
		touchLeft.close();
		touchRight.close();
	}
	
	static void backAndTurn(){
		calculate(mA, mB);
		stop(mA, mB);
		moveBack(.1);
		
		//rotate in place
		mA.setSpeed(200);
		mB.setSpeed(200);
		mA.rotate(90);
		mB.rotate(-90);
		//39 might still be too high
		heading -= (39 * (Math.PI / 180.0));
	}
	
	static void calculate(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		double endTime = System.nanoTime();
		double dT = (endTime - startTime) / 1000000000.0;
		startTime = endTime;
		double wl = ma.getRotationSpeed() * (Math.PI / 180.0);
		double wr = mb.getRotationSpeed() * (Math.PI / 180.0);
		double vl = wl * wheelRadius;
		double vr = wr * wheelRadius;
		double v = (vl + vr) / 2.0;

		double omega = (vr - vl) / L;
		heading += omega * dT;
		xCoordinate += v * dT * Math.cos(heading);
		yCoordinate += v * dT * Math.sin(heading);
		
	}
	
	static void moveBack(double distance){
		mA.setSpeed(200);
		mB.setSpeed(200);
		double seconds = distance / (200 * (Math.PI / 180.0) * wheelRadius);
		mA.startSynchronization();
		mA.backward();
		mB.backward();
		mA.endSynchronization();
		Delay.msDelay((long)(seconds * 1000));
		stop(mA, mB);
		xCoordinate -= distance * Math.cos(heading);
		yCoordinate -= distance * Math.sin(heading);
	}
	
	static void forward(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.forward();
		mb.forward();
		ma.endSynchronization();
	}
	
	static void stop(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.stop(true);
		mb.stop(true);
		ma.endSynchronization();
	}
	
	static void backward(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.startSynchronization();
		ma.backward();
		mb.backward();
		ma.endSynchronization();
	}
	
	static void moveLeft(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.setSpeed(125);
		mb.setSpeed(200);
		forward(ma, mb);
	}
	
	static void moveRight(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.setSpeed(200);
		mb.setSpeed(125);
		forward(ma, mb);
	}
	
	static void equalMove(EV3MediumRegulatedMotor ma, EV3MediumRegulatedMotor mb){
		ma.setSpeed(125);
		mb.setSpeed(125);
		forward(ma, mb);
	}
	
	static float getSamples(SensorMode ultrasonicSensor, float[] ultrasonicSamples) {
		float sampleVal;
		float total = 0.0f;
		for (int i = 0; i < 5; i++) {
			ultrasonicSensor.fetchSample(ultrasonicSamples, 0);
			sampleVal = ultrasonicSamples[0];
			total += sampleVal;
		}
		float avg = total / 5.0f;
		return avg;
	}
}