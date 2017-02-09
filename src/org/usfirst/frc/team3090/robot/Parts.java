package org.usfirst.frc.team3090.robot;

import com.ctre.CANTalon;

public class Parts {
	public static final int D1 = 15;
	public static final int D2 = 12;
	public static final int D3 = 13;
	public static final int D4 = 14;
	
	public static final int L1 = 10;
	public static final int L2 = 11;
		
	public static final CANTalon front_left = new CANTalon(D1);
	public static final CANTalon back_left = new CANTalon(D2);
	public static final CANTalon front_right= new CANTalon(D4);
	public static final CANTalon back_right = new CANTalon(D3);
	
	public static final CANTalon lift_1 = new CANTalon(L1);
	public static final CANTalon lift_2 = new CANTalon(L2);
	
	public static void init() {
		back_left.setInverted(true);
		front_left.setInverted(true);
		back_right.setInverted(true);
		front_right.setInverted(true);
	}
}
