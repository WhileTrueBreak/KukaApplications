package application;

import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.vividsolutions.jts.math.Vector2D;

public class Position {
	public double x,y;
	public Position(double x, double y){
		this.x = x;
		this.y = y;
	}

	public Position getRelDistance(Position oldPosition){
		// Returns the relative distance from the old position to current position
		return new Position(x-oldPosition.x, y-oldPosition.y);
	}
	
//	public double x() {return x;};
//	public double y() {return y;};
//	public double z() {return z;};
}
