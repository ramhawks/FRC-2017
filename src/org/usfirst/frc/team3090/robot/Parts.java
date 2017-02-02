package org.usfirst.frc.team3090.robot;

import com.ctre.CANTalon;

public class Parts {
	public static final int D1 = 15;
	public static final int D2 = 12;
	public static final int D3 = 13;
	public static final int D4 = 14;
		
	public static final CANTalon front_left = new CANTalon(D1);
	public static final CANTalon back_left = new CANTalon(D2);
	public static final CANTalon front_right= new CANTalon(D4);
	public static final CANTalon back_right = new CANTalon(D3);
}
