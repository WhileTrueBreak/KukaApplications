package application.robotControl.IK;

public class Configuration {
	
	public int arm = 1;
	public int elbow = 1;
	public int wrist = 1;
	public int confBin;
	
	public Configuration(int confBin) {
		this.confBin = confBin;
		if((confBin&1) != 0) {
			arm = -1;
		}
		if((confBin&2) != 0) {
			elbow = -1;
		}
		if((confBin&4) != 0) {
			wrist = -1;
		}
	}
	
}
