package roadgraph;

import geography.GeographicPoint;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import util.GraphLoader;

import java.util.List;

import static org.junit.Assert.*;

/**
 * Created by BSV on 12.10.2016.
 */
public class MapGraphTest {
    @Before
    public void setUp() throws Exception {

    }




    @Test
    public void dijkstra() throws Exception {
        MapGraph map = new MapGraph();
        GraphLoader.loadRoadMap("data/graders/mod3/map3.txt", map);


        GeographicPoint start = new GeographicPoint(0, 0);
        GeographicPoint end = new GeographicPoint(0, 4);

        List<GeographicPoint> path = map.dijkstra(start, end);

        CorrectAnswer corr = new CorrectAnswer("data/graders/mod3/map3.txt.answer", false);

        assertEquals("Dijkstra's algorithm error. ",corr.path, path);

    }

    @Test
    public void aStarSearch() throws Exception {

    }






    @After
    public void tearDown() throws Exception {

    }

}