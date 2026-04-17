/// Represents an Occupancy Grid, typically used to store LIDAR or SLAM data.
class OccupancyGrid {
  final Header header;             // Metadata such as timestamp and frame ID
  final MapMetaData mapMetaData;   // Grid resolution, size, and origin
  final List<int> data;            // Flattened 2D array of occupancy values

  OccupancyGrid({
    required this.header,
    required this.mapMetaData,
    required this.data,
  });

  /// Factory constructor to create an OccupancyGrid from JSON data.
  factory OccupancyGrid.fromJson(Map<String, dynamic> json) {
    List<int> dataIntList = (json["data"] as List<dynamic>)
        .map<int>((item) => item as int)
        .toList();

    return OccupancyGrid(
      header: Header(
        frameId: json["header"]["frame_id"],
        stamp: Stamp(
          sec: json["header"]["stamp"]["sec"],
          nanosec: json["header"]["stamp"]["nanosec"],
        ),
      ),
      mapMetaData: MapMetaData(
        resolution: json["info"]["resolution"],
        width: json["info"]["width"],
        height: json["info"]["height"],
        origin: Pose(
          position: Position(
            x: json["info"]["origin"]["position"]["x"],
            y: json["info"]["origin"]["position"]["y"],
            z: json["info"]["origin"]["position"]["z"],
          ),
          orientation: Orientation(
            x: json["info"]["origin"]["orientation"]["x"],
            y: json["info"]["origin"]["orientation"]["y"],
            z: json["info"]["origin"]["orientation"]["z"],
            w: json["info"]["origin"]["orientation"]["w"],
          ),
        ),
      ),
      data: dataIntList,
    );
  }
}

/// Contains metadata about the grid's coordinate frame and timestamp.
class Header {
  final String frameId;  // Coordinate frame (e.g. "map", "base_link")
  final Stamp stamp;     // Time when the data was recorded

  Header({required this.frameId, required this.stamp});
}

/// Represents the timestamp using seconds and nanoseconds.
class Stamp {
  final int sec;
  final int nanosec;

  Stamp({required this.sec, required this.nanosec});
}

/// Contains resolution, dimensions, and the origin of the map.
class MapMetaData {
  final double resolution;  // Cell size in meters
  final int width;          // Grid width (cells)
  final int height;         // Grid height (cells)
  final Pose origin;        // Origin of the map in world coordinates

  MapMetaData({
    required this.resolution,
    required this.width,
    required this.height,
    required this.origin,
  });
}

/// The position and orientation of the grid’s origin in space.
class Pose {
  final Position position;
  final Orientation orientation;

  Pose({required this.position, required this.orientation});
}

/// Represents the spatial position (x, y, z).
class Position {
  final double x, y, z;

  Position({required this.x, required this.y, required this.z});
}

/// Represents the orientation as a quaternion (x, y, z, w).
class Orientation {
  final double x, y, z, w;

  Orientation({
    required this.x,
    required this.y,
    required this.z,
    required this.w,
  });
}
