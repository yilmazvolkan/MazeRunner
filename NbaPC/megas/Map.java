package megas;

import megas.Cell;

public class Map {


	public static final int MAP_WIDTH = 7;
	
	private Cell[][] map;
	
	public Map() {
		map = new Cell[MAP_WIDTH][MAP_WIDTH];
		initializeMap();
	}
	
	public Cell[][] getMap() {
		return this.map;
	}
	
	public void addCell(Cell cell, int firstIndex, int secondIndex) {
		this.map[firstIndex][secondIndex] = cell;
	}
	
	public Cell getCellAt(int firstIndex, int secondIndex) {
		return this.map[firstIndex][secondIndex];
	}
	
	// Fills the map with empty cells
	private void initializeMap() {
		System.out.println("Map is initializing..");
		for(int i = 0; i< MAP_WIDTH; i++) {
			for(int j = 0; j<MAP_WIDTH; j++) {
				Cell cell = new Cell();
				this.map[i][j] = cell;
			}
		}
	}


}
