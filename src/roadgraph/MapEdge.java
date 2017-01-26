package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private String roadName;
	private String roadType;
	private Double length;
	private GeographicPoint dst;


	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public Double getLength() {
		return length;
	}

	public GeographicPoint getDst() {
		return dst;
	}

	public String toString() {
		return new String(dst.toString());
	}

	public MapEdge(GeographicPoint to, String rdName, String rdType, Double lng) {
		roadName = new String(rdName);
		roadType = new String(rdType);
		length = lng;
		dst = to;
	}
}
