package org.usfirst.frc.team4206.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends Subsystem {


	// These values are the default if you instantiate a PixySPI without arguments.
	// To create multiple PixySPI objects and thus use multiple Pixy cameras via SPI
	// Copy the items below, change variable names as needed and especially change
	// the SPI port used eg; Port.kOnboardCS[0-3] or Port.kMXP
	public PixySPI pixy1;
	Port port = Port.kOnboardCS0;
	String print;
	public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();
	public double avgX = 0;

	public Vision(){
		// Open a pipeline to a Pixy camera.
		pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));
	}


	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double testPixy1() {
		int ret = -1, totalPackets = 0;
		double xTotal = 0;
		// Get the packets from the pixy.
		try {
			ret = pixy1.readPackets();
		} catch (PixyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		SmartDashboard.putNumber("Pixy Vision: packets size: ", packets.size());

		for(int i = 1; i <= PixySPI.PIXY_SIG_COUNT ; i++) {
			//SmartDashboard.putString("Pixy Vision: Signature: ", Integer.toString(i));

			//SmartDashboard.putNumber("Pixy Vision: packet: " + Integer.toString(i) + ": size: ", packets.get(i).size());
			
			// Loop through the packets for this signature.
			for(int j=0; j < packets.get(i).size(); j++) {
				/*
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": X: ", packets.get(i).get(j).X);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Y: ", packets.get(i).get(j).Y);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Width: ", packets.get(i).get(j).Width);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Height: ", packets.get(i).get(j).Height);
				*/
				xTotal += packets.get(i).get(j).X;
				totalPackets = packets.get(i).size();
				/*
				System.out.println("X: " + packets.get(i).get(j).X);
				System.out.println("Y: " + packets.get(i).get(j).Y);
				System.out.println("Width: " + packets.get(i).get(j).Width);
				System.out.println("Height: " + packets.get(i).get(j).Height);
				System.out.println("---------------");
				*/
			}
			SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Avg X: ", xTotal/packets.get(i).size());
		}
		if (totalPackets > 1) return xTotal/totalPackets;
		else if (totalPackets == 1) return 999;
		else return 998;
	}
}

