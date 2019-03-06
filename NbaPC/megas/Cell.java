package megas;

public class Cell{


	public static final float WALL_DISTANCE_THRESHOLD = 25.0f;
	
	int colorId = -1;
	
	boolean frontWall = false;
	boolean rightWall = false;
	boolean backWall = false;
	boolean leftWall = false;
	
	boolean isVisited = false;
	
	public Cell() {}
	
	public Cell(int colorId, boolean[] walls) {
		
		this.colorId = colorId;
		
		this.frontWall = walls[0];
		this.rightWall = walls[1];
		this.backWall = walls[2];
		this.leftWall = walls[3];

	}
}
