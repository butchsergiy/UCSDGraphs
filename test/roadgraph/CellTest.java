package roadgraph;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Created by BSV on 26.09.2016.
 */
public class CellTest {
	@Before
	public void setUp() throws Exception {

	}

	@After
	public void tearDown() throws Exception {

	}

	@Test
	public void getPrintRepresentation() throws Exception {
		Cell cell = new Cell(1, 1, 'a', null);
		assertEquals("Cell printRepresentationMethod return bad value.", 'a', cell.getPrintRepresentation());
	}

	@Test
	public void getI() throws Exception {
		Cell cell = new Cell(1, 1);
		assertEquals("Cell gives wrong index", 1, cell.getI());
	}

	@Test
	public void getJ() throws Exception {
		Cell cell = new Cell(1, 88);
		assertEquals("Cell gives wrong index", 88, cell.getJ());
	}

	@Test
	public void getNeighbors() throws Exception {
		List<Cell> neighborsList = new LinkedList<>();
		neighborsList.add(new Cell(11, 11));

		Cell cell01 = new Cell(1, 1, 'a', new LinkedList<>(neighborsList));

		neighborsList.clear();
		neighborsList.add(new Cell(11, 11));

		assertEquals("Cell gives wrong Neighbor", true, cell01.getNeighbors().get(0).equals(neighborsList.get(0)));
	}

}