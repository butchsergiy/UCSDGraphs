package roadgraph;

import java.util.*;

/**
 * Created by BSV on 26.09.2016.
 */



class Cell {
    private int         i, j;
    private char        printRepresentation = '.';
    private List<Cell>  neighbors = new ArrayList<>();


    public Cell(int i, int j){
        this.i=i;
        this.j=j;
    }

    public Cell(int i, int j, char printRepresentation, List<Cell> neighbors) {
        this.i = i;
        this.j = j;
        this.printRepresentation = printRepresentation;
        this.neighbors = neighbors;
    }

    /**
     * Return char representation of the Cell
     *
     */
    public char getPrintRepresentation() {
        return printRepresentation;
    }

    public int getI() {
        return i;
    }

    public int getJ() {
        return j;
    }

    public List<Cell> getNeighbors() {
        return neighbors;
    }

    public boolean equals(Cell cell){
        if(cell.getI()==i && cell.getJ()==j ) return true;
        return false;
    }
}








class Board{

    private int i,j;
    private Cell[][] board = new Cell[10][10];

    public Board(int i, int j){
        this.i = i;
        this.j = j;
        this.board = new Cell[i][j];
    }

    public void setCell(int i, int j, char representation, List<Cell> neighbors){
        this.board[i][j] = new Cell(i,j,representation,neighbors);
    }

    public void printBoard() {
        for (int i = 0; i < board.length; i++) {
            for (int j = 0; j < board[0].length; j++) {
                System.out.print(((board[i][j] != null)?
                        board[i][j].getPrintRepresentation() :"X") + "\t");
            }
            System.out.print("\n");
        }
    }

    public List<Cell> findPathDFS(int startI, int startJ, int endI, int endJ){
        Stack<Cell> stack = new Stack<>();
        HashSet<Cell> visited = new HashSet<>();

        return null;
    }
}





public class MapGraphTest {
    public static void main(String[] args){

        Board board = new Board(9, 9);
        board.setCell(0,0, 'x', null);
        board.setCell(0,1, 'o', null);

        board.printBoard();

    }
}


