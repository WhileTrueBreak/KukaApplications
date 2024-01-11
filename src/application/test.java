package application;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import application.utils.MathHelper;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		Double[] numbers = {0d,30d,-30d,0d};
		List<Double> points = Arrays.asList(numbers);
		for(double t = 0;t <= 1;t += 0.05) {
			double x = MathHelper.bezier(t, points);
			System.out.println(x);
		}
	}

}
