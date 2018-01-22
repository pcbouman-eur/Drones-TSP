/* Copyright 2017 Paul Bouman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the  
 * "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * In case you use this software for research purposes, it is appreciated if you provide a citation of the following paper:
 * 
 * N.A.H. Agatz, P.C. Bouman & M.E. Schmidt. Optimization Approaches for the Traveling Salesman Problem with Drone. Transportation Science.
 * 
 * The paper still has to appear, but was accepted for publication. This notice will be updated with a more detailed reference if that
 * information is available.
 */
package nl.rsm.tom.drones.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.triangulate.DelaunayTriangulationBuilder;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.Instance;


/**
 * Class with Delaunay Triangulation related helper methods
 * @author Paul Bouman
 *
 */

public class DelaunayTriangulation
{
	/**
	 * Wrapper function around the JTS implementation of the Delaunay Triangulation
	 * @param i The instance to be converted
	 * @return The GraphInstance corresponding to the Delaunay Triangulation
	 */
	public static GraphInstance<Vec2D> convert(Instance<Vec2D> i)
	{
		List<Vec2D> l = new ArrayList<>(i.getLocations());
		l.add(i.getDepot());
		GeometryFactory gf = new GeometryFactory();
		Map<Coordinate,Vec2D> map = new HashMap<>();
		List<Coordinate> points = new ArrayList<>(l.size());
		for (Vec2D v : l)
		{
			Coordinate p = new Coordinate(v.x, v.y, 0);
			map.put(p,v);
			points.add(p);
		}
		DelaunayTriangulationBuilder dtb = new DelaunayTriangulationBuilder();
		dtb.setSites(points);
		MultiLineString mls = (MultiLineString) dtb.getEdges(gf);
		List<Vec2D> locs = new ArrayList<>(i.getLocations());
		Distance<Vec2D> drive = i.getDriveDistance();
		Distance<Vec2D> fly = i.getFlyDistance();
		GraphInstance<Vec2D> gi = new GraphInstance<Vec2D>(i.getDepot(),locs,drive,fly);
		for (int t=0; t < mls.getNumGeometries(); t++)
		{
			LineString ls = (LineString) mls.getGeometryN(t);
			Point s = ls.getStartPoint();
			Point e = ls.getEndPoint();
			Vec2D sV = map.get(s.getCoordinate());
			Vec2D eV = map.get(e.getCoordinate());
			gi.addDistance(sV, eV, drive.getContextFreeDistance(sV, eV), fly.getContextFreeDistance(sV, eV));
		}
		return gi;
	}
}
