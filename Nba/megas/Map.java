package megas;

import java.awt.Point;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

import megas.Cell;

public class Map implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public static final int MAP_WIDTH = 7;
	
	private Cell[][] map;
	
	public Point green_coordinates;
	public Point red_coordinates;
	public Point blue_coordinates;

	public Map() {
		map = new Cell[MAP_WIDTH][MAP_WIDTH];
		green_coordinates = new Point(-1, -1);
		red_coordinates = new Point(-1, -1);
		blue_coordinates = new Point(-1, -1);

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
		for(int i = 0; i< MAP_WIDTH; i++) {
			for(int j = 0; j<MAP_WIDTH; j++) {
				Cell cell = new Cell();
				this.map[i][j] = cell;
			}
		}
	}
    public void writeObjectToFile(String filepath) {

        try {
            FileOutputStream fileOut = new FileOutputStream(filepath);
            ObjectOutputStream objectOut = new ObjectOutputStream(fileOut);
            objectOut.writeObject(this);
            objectOut.close();
            fileOut.flush();
            System.out.println("The Object  was succesfully written to a file");

        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
    

    public Map ReadObjectFromFile(String filepath) {
 
        try {
 
            FileInputStream fileIn = new FileInputStream(filepath);
            ObjectInputStream objectIn = new ObjectInputStream(fileIn);
 
            Map obj = (Map) objectIn.readObject();
 
            System.out.println("The Object has been read from the file");
            objectIn.close();
            return obj;

        } catch (Exception ex) {
            ex.printStackTrace();
            return null;
        }
    }
    
    
    @Override
    public String toString() {
        return new StringBuffer(" THIS IS A MAP: ").append(this.map[3][3].colorId).toString();
    }

}
