package application.utils;

public class Matrix{
	
	private int rows, cols;
	private double[] matrix;
	
	private Matrix(int cols, int rows) {
		this.rows = rows;
		this.cols = cols;
		this.matrix = new double[rows*cols];
	}
	
	private Matrix(int cols, int rows, double fill) {
		this.rows = rows;
		this.cols = cols;
		this.matrix = new double[rows*cols];
		for(int i = 0;i < rows*cols;i++) this.matrix[i] = fill;
	}
	
	public double getIndex(int row, int col) {
		return this.matrix[col+row*cols];
	}
	
	public void setIndex(int row, int col, double value) {
		this.matrix[col+row*cols] = value;
	}
	
	public Matrix getSubMatrix(int r1, int c1, int r2, int c2) {
		Matrix subMatrix = new Matrix(c2-c1+1, r2-r1+1, 0);
		for(int i = r1;i <= r2;i++) {
			for(int j = c1;j <= c2;j++) {
				subMatrix.setIndex(i-r1, j-c1, this.getIndex(i, j));
			}
		}
		return subMatrix;
	}
	
	public Matrix transpose() {
		Matrix matrix = new Matrix(this.rows, this.cols);
		for(int i = 0;i < this.rows;i++){
			for(int j = 0;j < this.cols;j++){
				matrix.setIndex(j, i, this.getIndex(i, j));
			}
		}
		return matrix;
	}

	public String toString() {
		String printString = "";
		for(int i = 0;i < this.rows;i++) {
			printString += "[";
			for(int j = 0;j < this.cols;j++) {
				if(j != 0) printString += ", ";
				printString += String.format("%.2f", this.getIndex(i, j));
			}
			printString += "]\n";
		}
		return printString;
	}
	
	public static Matrix dot(Matrix m1, Matrix m2) {
		Matrix out = new Matrix(m1.rows, m2.cols);
		for(int i = 0;i < m1.rows;i++) {
			for(int j = 0;j < m2.cols;j++) {
				for(int n = 0;n < m1.cols;n++) {
					out.matrix[j+out.cols*i] += m1.getIndex(i, n)*m2.getIndex(n, j);
				}
			}
		}
		return out;
	}
	
	public static Matrix zero(int rows, int cols) {
		return new Matrix(rows, cols, 0);
	}
	
	public static Matrix identity(int size) {
		Matrix matrix = Matrix.zero(size, size);
		for(int i = 0;i < size;i++) matrix.matrix[i*size+i] = 1;
		return matrix;
	}
	
	
	public static Matrix skew(double v0, double v1, double v2) {
		Matrix matrix = new Matrix(3, 3, 0);
		matrix.setIndex(1, 2, -v0);
		matrix.setIndex(2, 0, -v1);
		matrix.setIndex(0, 1, -v2);
		matrix.setIndex(2, 1, v0);
		matrix.setIndex(0, 2, v1);
		matrix.setIndex(1, 0, v2);
		return matrix;
	}
	
}
