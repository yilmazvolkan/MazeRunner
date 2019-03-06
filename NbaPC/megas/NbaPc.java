package megas;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Line2D;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.ListIterator;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class NbaPc extends JFrame {
	
	private static final long serialVersionUID = 2280874288993963333L;
	
	// =================================================================
	// ============================ MAP INFO ===========================
	// =================================================================
	public static final int CELL_WIDTH = 100;
	public static final int MEGAS_WIDTH = 50;
	public static final int PARTICLE_WIDTH = 10;
	public static final int PARTICLE_PADDING = 10;
	
	
	public static final int FRAME_WIDTH = CELL_WIDTH * 7;
	public static final int FRAME_HEIGHT = CELL_WIDTH * 7;

	public static final Color MEGAS_COLOR = Color.ORANGE;
	public static final Color WALL_COLOR = Color.YELLOW;
	public static final Color BACKGROUND_COLOR = Color.GRAY;
	public static final Color STRIPE_COLOR = Color.DARK_GRAY;
	public static final Color PARTICLE_COLOR = Color.MAGENTA;
	
	public static final int MAPPING_MODE = 0;
	public static final int LOCALIZATION_MODE = 1;
	public static final int GO_TO_BALL_MODE = 2;
	
	// =================================================================
	// ========================= POSITION INFO =========================
	// =================================================================
	static Map map;
	static ArrayList<int[]> particles;
	
	static int xPos = 3;
	static int yPos = 3;
	static int orientation = 0;
	
	static int current_mod = 0; // mapping
	
	static InputStream inputStream;
	static OutputStream outputStream;
	static DataInputStream dataInputStream;

	public NbaPc() {
		super("Map Making");
		setSize(FRAME_WIDTH, FRAME_HEIGHT);
		setBackground(BACKGROUND_COLOR);
		setResizable(true);
		setVisible(true);
		map = new Map();
		particles = new ArrayList<int[]>();
	}
	
	public static void main(String[] args) throws Exception	{
		
		NbaPc monitor = new NbaPc();
		
		monitor.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
		
		// Enter the ip here.
		String ip = "10.0.1.1";
		
		@SuppressWarnings("resource")
		Socket socket = new Socket(ip, 1234);
		System.out.println("Connected!");

		inputStream = socket.getInputStream();
		outputStream = socket.getOutputStream();
		dataInputStream = new DataInputStream(inputStream);
		DataOutputStream dataOutputStream = new DataOutputStream(outputStream);
		
		current_mod = dataInputStream.readInt();
		System.out.println("Current Mode " + current_mod);

		while(current_mod == MAPPING_MODE){
			receivePositionInfo(dataInputStream);
			monitor.repaint();
			dataOutputStream.flush();
		}
		
		// Get the discovered map data
		if(current_mod == LOCALIZATION_MODE) {
			System.out.println("MOD IS CHANGED TO LOCALIZATION");
			receiveLocalizationMapInfo(dataInputStream);		
			//dataOutputStream.flush(); // TODO: This can be dangerous.
		}
		
		// Get particles.
		while(current_mod == LOCALIZATION_MODE) {
			receiveParticlesInfo(dataInputStream);
			monitor.repaint();
			dataOutputStream.flush();
		}
		
		// Get particles.
		while(current_mod == GO_TO_BALL_MODE) {
			receiveGoToBallInfo(dataInputStream);
			monitor.repaint();
			dataOutputStream.flush();
		}
		
	}
	
	public static void receiveGoToBallInfo(DataInputStream dataInputStream) throws IOException {
		System.out.println("Go to ball behavior is starting...");
		
		current_mod = dataInputStream.readInt();
		System.out.println("Current Mode " + current_mod);

		if (current_mod != GO_TO_BALL_MODE) {
			return;
		}
		xPos = dataInputStream.readInt();
		System.out.println(xPos);
		yPos = dataInputStream.readInt();
		System.out.println(yPos);
		System.out.println(".................");
	}
	
	public static void receiveParticlesInfo(DataInputStream dataInputStream) throws IOException {
		System.out.println("Particle transfer is starting...");

		current_mod = dataInputStream.readInt();
		System.out.println("Current Mode " + current_mod);

		if (current_mod != LOCALIZATION_MODE) {
			return;
		}
	
		boolean isFinished = false;
		particles.clear();
		
		while(!isFinished) {
			int[] particle = new int[3];
			
			int xCoordinate = dataInputStream.readInt();
			int yCoordinate = dataInputStream.readInt();
			int orientation = dataInputStream.readInt();
			
			particle[0] = xCoordinate;
			particle[1] = yCoordinate;
			particle[2] = orientation;
			System.out.println(xCoordinate);
			System.out.println(yCoordinate);
			particles.add(particle);
			isFinished = dataInputStream.readBoolean();
		}
	
		System.out.println("Particle transfer is complete");
	}
	
	public static void receiveLocalizationMapInfo(DataInputStream dataInputStream) throws IOException {
		
		System.out.println("Map transfer is starting");
		current_mod = dataInputStream.readInt();
		System.out.println("Current Mode " + current_mod);

		if (current_mod != LOCALIZATION_MODE) {
			return;
		}
		
		for(int i = 0; i< map.MAP_WIDTH; i++) {
			for(int j = 0; j<map.MAP_WIDTH; j++) {
				int xCoordinate = dataInputStream.readInt();
				int yCoordinate = dataInputStream.readInt();
				int colorId = dataInputStream.readInt();
				
				boolean frontWall = dataInputStream.readBoolean();
				boolean rightWall = dataInputStream.readBoolean();
				boolean backWall = dataInputStream.readBoolean();
				boolean leftWall = dataInputStream.readBoolean();
				boolean[] walls = { frontWall, rightWall, backWall, leftWall };
				
				System.out.println("*****************");
				System.out.println("x " + xCoordinate);
				System.out.println("y " + yCoordinate);
				System.out.println("colorId " + colorId);
				System.out.println( frontWall);
				System.out.println( rightWall);
				System.out.println( backWall);
				System.out.println( leftWall);
				System.out.println("*****************");

				Cell cell = new Cell(colorId, walls);
				cell.isVisited = true;
				map.addCell(cell, xCoordinate, yCoordinate);
				boolean isFinished = dataInputStream.readBoolean();
			}
		}
		System.out.println("Map transfer is complete");
	}
	
	public static void receivePositionInfo(DataInputStream dataInputStream) throws IOException {
		System.out.println("---------- receivePositionInfo ---------");

		current_mod = dataInputStream.readInt();
		System.out.println("Current Mode " + current_mod);
		if (current_mod !=0) {
			return;
		}
		xPos = dataInputStream.readInt();
		System.out.println("xPos " + xPos);
		yPos = dataInputStream.readInt();
		System.out.println("yPos " + yPos);

		orientation = dataInputStream.readInt();
		System.out.println("orientation " + orientation);
		int colorId = dataInputStream.readInt();
		System.out.println("colorId " + colorId);

		boolean frontWall = dataInputStream.readBoolean();
		System.out.println("frontWall " + frontWall);
		boolean rightWall = dataInputStream.readBoolean();
		System.out.println("rightWall " + rightWall);
		boolean backWall = dataInputStream.readBoolean();
		System.out.println("backWall " + backWall);
		boolean leftWall = dataInputStream.readBoolean();
		System.out.println("leftWall " + leftWall);
		boolean[] walls = { frontWall, rightWall, backWall, leftWall };
		System.out.println("----------------------------------------");

		Cell cell = new Cell(colorId, walls);
		cell.isVisited = true;
		map.addCell(cell, xPos, yPos);
	}
	
	public void paint(Graphics g) {
		super.paint(g);
		
		if (current_mod == MAPPING_MODE) {
			displayMap(map, g);
			displayMegas(xPos,yPos,g);	
		} else if (current_mod == LOCALIZATION_MODE) {
			displayMap(map,g);
			displayParticles(g);
		} else if (current_mod == GO_TO_BALL_MODE) {
			displayMap(map,g);
			displayMegas(xPos,yPos,g);
		}
	}

	public void displayParticles(Graphics g) {
		
		Iterator<int[]> iterator = particles.iterator();
		
		while(iterator.hasNext()) {
			int[] current_particle = iterator.next();
			
			int x = current_particle[0];
			int y = current_particle[1];
			int particle_orientation = current_particle[2];
			
			// Print the particle
			Graphics2D g2 = (Graphics2D) g;
			g2.setColor(PARTICLE_COLOR);
			int xDraw = 0;
			int yDraw = 0;
			
			if (particle_orientation == 0) {
				xDraw = y * CELL_WIDTH + (CELL_WIDTH - PARTICLE_WIDTH)/2;
				yDraw = x * CELL_WIDTH + PARTICLE_PADDING;
			} else if (particle_orientation == 1) {
				xDraw = (y + 1) * CELL_WIDTH - (PARTICLE_WIDTH + PARTICLE_PADDING);
				yDraw = x * CELL_WIDTH + ((CELL_WIDTH - PARTICLE_WIDTH)/2);	
			} else if (particle_orientation == 2) {
				xDraw = y * CELL_WIDTH + (CELL_WIDTH - PARTICLE_WIDTH)/2;
				yDraw = (x + 1) * CELL_WIDTH - (PARTICLE_WIDTH + PARTICLE_PADDING);
			} else if (particle_orientation == 3) {
				xDraw = y * CELL_WIDTH + PARTICLE_PADDING;
				yDraw = x * CELL_WIDTH + ((CELL_WIDTH - PARTICLE_WIDTH)/2);	
			}
			
			g2.fillRect(xDraw,yDraw, PARTICLE_WIDTH, PARTICLE_WIDTH);
		}	
	}
	
	public void displayMap( Map map, Graphics g ){
		Graphics2D g2 = (Graphics2D) g;

		// Draw Cells
		for (int i = 0; i < map.getMap().length; i++ ){
			for (int j = 0; j < map.getMap()[0].length; j++) {
				// Draw a cell according to the cell data
				Cell currentCell = map.getCellAt(i, j);

				Color color = Color.GRAY;
				// If the cell is visited then get the color from inside.
				if (currentCell.isVisited) {
					int colorId = map.getCellAt(i, j).colorId;
					if (colorId == 6) {
						color = Color.WHITE;
					} else if (colorId == 1) {
						color = Color.GREEN;
					} else if (colorId == 2) {
						color = Color.BLUE;
					} else if(colorId == 7) {
						color = Color.BLACK;
					} else if(colorId ==0) {
						color = Color.RED;
					}
				}
				
				// g2.setColor(color);
				g2.setPaint(color);
				
				// g2.drawRect(i * CELL_WIDTH, (7-j) * CELL_WIDTH , CELL_WIDTH, CELL_WIDTH);
				g2.fillRect(j * CELL_WIDTH, i * CELL_WIDTH , CELL_WIDTH, CELL_WIDTH);
				
				// Draw the walls
				g2.setStroke(new BasicStroke(10.0f));
				
				// Front Wall
				if (currentCell.frontWall) {
					g2.setColor(WALL_COLOR);
					// g2.drawLine(i*CELL_WIDTH,(7-j)*CELL_WIDTH,(i+1)*CELL_WIDTH,(7-j)*CELL_WIDTH);
					g2.draw(new Line2D.Double(j*CELL_WIDTH,i*CELL_WIDTH,(j+1)*CELL_WIDTH,i*CELL_WIDTH));	
				}
				
				// Right Wall
				if (currentCell.rightWall) {
					g2.setColor(WALL_COLOR);
					// g.drawLine((i+1)*CELL_WIDTH, (7-j)*CELL_WIDTH,(i+1)*CELL_WIDTH,  (7-j-1)*CELL_WIDTH);
					g2.draw(new Line2D.Double((j+1)*CELL_WIDTH, i*CELL_WIDTH,(j+1)*CELL_WIDTH,  (i+1)*CELL_WIDTH));	
				}
				
				// Back Wall
				if (currentCell.backWall) {
					g2.setColor(WALL_COLOR);
					// g.drawLine(i*CELL_WIDTH,(7-j) * CELL_WIDTH,(i+1)*CELL_WIDTH, (7-j-1)*CELL_WIDTH);
					g2.draw(new Line2D.Double(j*CELL_WIDTH,(i+1) * CELL_WIDTH,(j+1)*CELL_WIDTH, (i+1) *CELL_WIDTH));
				}
				
				// Left Wall
				if (currentCell.leftWall) {
					g2.setColor(WALL_COLOR);
					// g.drawLine(i*CELL_WIDTH,(7-j)*CELL_WIDTH,i*CELL_WIDTH, (7-j-1)*CELL_WIDTH);
					g2.draw(new Line2D.Double(j*CELL_WIDTH,i*CELL_WIDTH,j*CELL_WIDTH, (i+1)*CELL_WIDTH));
				}
			}
		}
		
		// Draw stripes
		g2.setColor(STRIPE_COLOR);
		g2.setStroke( new BasicStroke(0.5f));
		
		// Vertical Lines
		for(int i = 1; i<= 6; i++) {
			// g2.drawLine(i*CELL_WIDTH, 0, i*CELL_WIDTH, FRAME_HEIGHT);
			g2.draw(new Line2D.Double(i*CELL_WIDTH, 0, i*CELL_WIDTH, FRAME_HEIGHT));
		}
		
		// Horizontal Lines
		for(int i = 1; i<= 6; i++) {
			// g2.drawLine(0,i*CELL_WIDTH,FRAME_WIDTH, i*CELL_WIDTH);
			g2.draw(new Line2D.Double(0,i*CELL_WIDTH,FRAME_WIDTH, i*CELL_WIDTH));
		}
	}
	
	public void displayMegas(int xPos, int yPos, Graphics g ){
		Graphics2D g2 = (Graphics2D) g;
		g2.setColor(MEGAS_COLOR);
		// g2.setStroke( new BasicStroke( 5.0f ));
		g2.fillRect(yPos * CELL_WIDTH + ((CELL_WIDTH - MEGAS_WIDTH)/2), xPos * CELL_WIDTH + ((CELL_WIDTH - MEGAS_WIDTH)/2), MEGAS_WIDTH , MEGAS_WIDTH);
	}
}

