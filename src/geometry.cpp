#include "geometry.hpp"


namespace Geometry{



Point operator*(const double& ratio, const Point& point){
    return point*ratio;
}

/**
 * Get Boundries of a Surface Patch
 */
void getBoundries(const RectangularSurfacePatch& surface_patch, std::array<std::unique_ptr<LineSegment>,4>& boundries){

    boundries[0] = std::make_unique<LineSegment>(std::make_pair(surface_patch.getCorners()[0], surface_patch.getCorners()[1]));
    boundries[1] = std::make_unique<LineSegment>(std::make_pair(surface_patch.getCorners()[1], surface_patch.getCorners()[2]));
    boundries[2] = std::make_unique<LineSegment>(std::make_pair(surface_patch.getCorners()[2], surface_patch.getCorners()[3]));
    boundries[3] = std::make_unique<LineSegment>(std::make_pair(surface_patch.getCorners()[3], surface_patch.getCorners()[0]));
}

/**
 * Get the faces of a Cuboid
 */
void getFaces(const RectangularCuboid& cupoid, std::array<std::unique_ptr<RectangularSurfacePatch>,6>& cupoid_faces){
    const std::array<Eigen::Vector3d, 3> cuboid_edges =  cupoid.getEdges();
    const Point cuboid_corner  = cupoid.getCorner();

    cupoid_faces[0] = std::make_unique<RectangularSurfacePatch>(std::array<Point, 3>{cuboid_corner,
                                                                            cuboid_corner + cuboid_edges[0],
                                                                            cuboid_corner + cuboid_edges[1]});

    cupoid_faces[1] = std::make_unique<RectangularSurfacePatch>(std::array<Point, 3>{cuboid_corner,
                                                                            cuboid_corner + cuboid_edges[1],
                                                                            cuboid_corner + cuboid_edges[2]});

    cupoid_faces[2] = std::make_unique<RectangularSurfacePatch>(std::array<Point, 3>{cuboid_corner,
                                                                            cuboid_corner + cuboid_edges[0],
                                                                            cuboid_corner + cuboid_edges[2]});

    cupoid_faces[3] = std::make_unique<RectangularSurfacePatch>(*(cupoid_faces[0]));
    cupoid_faces[3]->translate(cuboid_edges[2]);
    cupoid_faces[4] = std::make_unique<RectangularSurfacePatch>(*(cupoid_faces[1]));
    cupoid_faces[4]->translate(cuboid_edges[0]);
    cupoid_faces[5] = std::make_unique<RectangularSurfacePatch>(*(cupoid_faces[2]));
    cupoid_faces[5]->translate(cuboid_edges[1]);
}


/**
 * \brief Calculate the unsigned distance between Two Point
 */
const double getDistance(const Point& point1, const Point& point2){
    double result = (point2 - point1).norm();
    return result;
}

/**
 * \brief Calculate the shortest unsigned distance between a point and a line segment
 */
const double getDistance(const Point& point, const LineSegment& line_segment){
    double result{0};
    double d1 = getDistance(point,line_segment.getEnds().first);
    double d2 = getDistance(point,line_segment.getEnds().second);
    double dm = getDistance(point, (line_segment.getEnds().first + (1.0/2.0) * line_segment.getEnds().second.getCoordinates()));

    if (dm < d1 && dm < d2){
        Eigen::Vector3d l1 = line_segment.getEnds().second - line_segment.getEnds().first;
        Eigen::Vector3d l2 = point - line_segment.getEnds().first;
        result = l2.cross(l1).norm()/l1.norm();
    }else if (dm >= d1 || dm >= d2){
        result = (d1<d2)?d1:d2;
    }
    return result;
}

const double getDistance(const LineSegment& line_segment, const Point& point){
    return getDistance(std::forward<const Point>(point), std::forward<const LineSegment>(line_segment));
}

/**
 * \brief Calculate the shortest SIGNED distance from a point to a rectanle surface patch
 * Its sign is equal to the sign of the dot product between the normal to the surface patch
 * and the vector from the point to any point on the surface patch
 * Distance From Point TO Surface Patch
 */
const double getDistance(const Point& point, const RectangularSurfacePatch& surface_patch) {
    double result{0.0};
    std::array<std::unique_ptr<LineSegment>,4> surface_boundries{nullptr};
    getBoundries(surface_patch, surface_boundries);
    double min_distance = std::numeric_limits<double>::max();
    for(size_t i{0}; i != surface_boundries.size(); ++i){
        assert(!!surface_boundries[i]);
        double boundries_distance = getDistance(point, *surface_boundries[i]);
        if(min_distance>boundries_distance)min_distance=boundries_distance;
    }
    const std::array<Point, 4> corners =  surface_patch.getCorners();
    double mx{0},my{0},mz{0};
    for(const auto& corner: corners){
        mx+=corners[0].getX();
        my+=corners[0].getY();
        mz+=corners[0].getZ();
    }
    Point mid_point({mx/corners.size(),my/corners.size(),mz/corners.size()});
    double mid_distance = getDistance(point, mid_point);
    if(min_distance<mid_distance){
        Eigen::Vector3d normal = (surface_patch.getCorners()[1]  
                                    - surface_patch.getCorners()[0]).cross(surface_patch.getCorners()[2]  
                                                                            - surface_patch.getCorners()[0]);
        if(std::signbit(normal.dot(surface_patch.getCorners()[0] - point))){
            result = -std::fabs(min_distance);
        }else{
            result = std::fabs(min_distance);
        }
    }else{
        Eigen::Vector3d normal = (surface_patch.getCorners()[1]  
                                    - surface_patch.getCorners()[0]).cross(surface_patch.getCorners()[2]  
                                                                            - surface_patch.getCorners()[0]);
        double& A{normal[0]}, B{normal[1]}, C{normal[2]};
        double D = -(A * surface_patch.getCorners()[0].getX() 
                        + B * surface_patch.getCorners()[0].getY() 
                        + C * surface_patch.getCorners()[0].getZ());
        double numerator = std::fabs( A * point.getX() + B * point.getY() + C * point.getZ() + D);
        double denominator = std::sqrt(A * A + B * B + C * C);
        if(std::signbit(normal.dot(surface_patch.getCorners()[0] - point))){
            result = -std::fabs(numerator / denominator);
        }else{
            result = std::fabs(numerator / denominator);
        }
    }
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance a rectanle surface patch to a point
 * Its sign is equal to the sign of the dot product between the normal to the surface patch
 * and the vector from any point on the surface patch the point
 * Distance From Surface Patch TO Point
 */
const double getDistance(const RectangularSurfacePatch& surface_patch, const Point& point){
    const double result = - getDistance(std::forward<const Point>(point), std::forward<const RectangularSurfacePatch>(surface_patch));
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance from a POINT to a SPHERE
 * Its sign is equal to the sign of the dot product of the vector from the point to any point on the sphere 
 * and the normal to the Sphere. 
 * The distance is Positive if the Point is inside of the SPHERE, so the distance is in the 
 * direction of the surface normal, and 
 * The distance is Negative if the Point is outside of the SPHERE, so the distance is in the 
 * reverse direction of the surface normal.
 * * Distance From POINT to a SPHERE
 */
const double getDistance(const Point& point, const Sphere& sphere){
    double result =  sphere.getRadus() - getDistance(point, sphere.getCenter());
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance from a SPHERE to a POINT
 * Its sign is equal to the sign of the dot product of the vector from any point on the SPHERE to the POINT
 * and the normal to the Sphere. 
 * The distance is Positive if the Point is outside of the SPHERE, so the distance is in the 
 * direction of the surface normal, and 
 * The distance is Negative if the Point is inside of the SPHERE, so the distance is in the 
 * reverse direction of the surface normal.
 * * Distance from a SPHERE to a POINT
 */
const double getDistance(const Sphere& sphere, const Point& point){
    const double result = - getDistance(point, sphere);
    return result;
}


/**
 * Calculate the shortest SIGNED distance from a POINT to a RECTANGULAR CUBOID
 * If the point is Inside the RECTANGULAR CUBOID the distance is Positive, and,
 * If the point is Outside the RECTANGULAR CUBOID the distance is Negative
 * Distance from a POINT to a RECTANGULAR CUBOID
 */
const double getDistance(const Point& point, const RectangularCuboid& cuboid){
    double result{std::numeric_limits<double>::max()};

    std::array<std::unique_ptr<RectangularSurfacePatch>,6> cupoid_faces{nullptr};
    std::array<double,6> face_distances{0.0};
    getFaces(cuboid, cupoid_faces);
    bool inside{true};
    for(size_t i{0}; i != cupoid_faces.size(); ++i){
        assert(!!cupoid_faces[i]);
        face_distances[i] = getDistance(point, *cupoid_faces[i]);
        inside = inside & std::signbit(face_distances[i]);
        result = (result<std::fabs(face_distances[i])?result:std::fabs(face_distances[i]));
    }

    if(!inside){
        result = - std::fabs(result);
    }
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance from a RECTANGULAR CUBOID to a POINT
 * If the point is Inside the RECTANGULAR CUBOID the distance is Negative, and,
 * If the point is Outside the RECTANGULAR CUBOID the distance is Positive
 * Distance from a RECTANGULAR CUBOID to a POINT
 */
const double getDistance(const RectangularCuboid& cuboid, const Point& point){
    const double result = - getDistance(point, cuboid);
    return result;
}

/**
 * \brief Calculate the shortest UNSIGNED distance between two Line Segments
 * The Line Segments can be part of Two Parallel lines, Tow Intersecting Lines, 
 * or Two Skew Lines. In case of Intersecting or skew line, the closest points on 
 * the line segments are the ones closest to the common prependiculat line. 
 * In case of two parallel lines the the candidates for the closest points are 
 * the end points of each line segments 
 */
const double getDistance(const LineSegment& line_segment1, const LineSegment& line_segment2){

    double result{0};
    Eigen::Vector3d vec1 = line_segment1.getEnds().second - line_segment1.getEnds().first;
    Eigen::Vector3d vec2 = line_segment1.getEnds().second - line_segment1.getEnds().first;
    
    if(std::fabs(vec1.dot(vec2) - vec1.norm() * vec2.norm())<=Geometry::eps){
        // Two Line Segments are part of Two Parallel Lines
        // The Minimum Distance between Two Line Segments is the minimum of 
        // their ends distance from other line
        double d0 = getDistance(line_segment1.getEnds().first, line_segment2);
        double d1 = getDistance(line_segment1.getEnds().second, line_segment2);
        double d2 = getDistance(line_segment2.getEnds().first, line_segment1);
        double d3 = getDistance(line_segment2.getEnds().second, line_segment1);
        result = std::min(std::min(std::min(d0,d2),d2),d3);

    }else{
        // Two Line Segments are part of Two Intersecting or Skew Lines
        Eigen::Vector3d common_normal = vec1.cross(vec2);

        // Find the closest points on each line
        Point Q1 = line_segment1.getEnds().first + (line_segment2.getEnds().first - line_segment1.getEnds().first).dot(common_normal)/
                                                    vec1.dot(common_normal)
                                                    * vec1;
        Point Q2 = line_segment2.getEnds().first + (line_segment1.getEnds().first - line_segment2.getEnds().first).dot(common_normal)/
                                                    vec1.dot(common_normal)
                                                    * vec2;
        result = (Q2 - Q1).norm();
    }
    return result;

}

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const LineSegment& line_segment, const RectangularSurfacePatch& surface_patch){
    double result{0.0};

    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Line Segment
 * \todo Implement
 */ 
const double getDistance( const RectangularSurfacePatch& surface_patch, const LineSegment& line_segment){
    double result{0.0};

    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Sphere
 * If the Line Segment is completely Inside of the Sphere, then, the distance will be Positive.
 * If the Line Segment is completely Outside of the Sphere, then, the distance will Negative.
 * If the Line Segmen Intersects the Sphere, then, the distance will be zero
 * 
 */ 
const double getDistance(const LineSegment& line_segment, const Sphere& sphere){
    double result{0};
    const double d1{line_segment.getEnds().first.getDistance(sphere.getCenter())};
    const double d2{line_segment.getEnds().second.getDistance(sphere.getCenter())};
    if(d1 < sphere.getRadus() && d2 < sphere.getRadus()){
        // The Line Segment is completely Inside of the Sphere
        result = std::min(sphere.getRadus() - d1, sphere.getRadus() - d2);
    }else if(d1 > sphere.getRadus() && d2 > sphere.getRadus()){
        // The Line Segment is completely Outside of the Sphere
        result = std::max(sphere.getRadus() - d1, sphere.getRadus() - d2);
    }else{
        // The Line Segmen Intersects the Sphere
        result = 0.0;
    }
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Line Segment
 * If the Line Segment is completely Inside of the Sphere, then, the distance will be Negative.
 * If the Line Segment is completely Outside of the Sphere, then, the distance will Positive.
 * If the Line Segmen Intersects the Sphere, then, the distance will be zero
 * 
 */ 
const double getDistance(const Sphere& sphere, const LineSegment& line_segment){
    double result = - getDistance(line_segment, sphere);
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const LineSegment& line_segment, const RectangularCuboid& cuboid){
    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Line Segment
 * \todo Complete the Comment
 */ 
const double getDistance( const RectangularCuboid& cuboid, const LineSegment& line_segment){
    double result = - getDistance(line_segment, cuboid);
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between Two Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch1, const RectangularSurfacePatch& surface_patch2){
    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Sphere
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch, const Sphere& sphere){

    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Rectangular Surface Patch
 * \todo Complete the Comment
 */ 
const double getDistance(const Sphere& sphere, const RectangularSurfacePatch& surface_patch){
    double result = - getDistance(surface_patch, sphere);
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch, const RectangularCuboid& cuboid){
    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Rectangular Surface Patch
 * \todo Complete the Comment
 */ 
const double getDistance(const RectangularCuboid& cuboid, const RectangularSurfacePatch& surface_patch){
    double result = - getDistance(surface_patch, cuboid);
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between Two Spheres
 * \todo Implement
 */ 
const double getDistance(const Sphere& sphere, const Sphere& sphere2){
    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const Sphere& sphere, const RectangularCuboid& cuboid){
    double result{0.0};
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Sphere
 * \todo Complete the Comment
 */ 
const double getDistance(const RectangularCuboid& cuboid, const Sphere& sphere){
    double result = - getDistance(sphere, cuboid);
    return result;
}

/**
 * \brief Calculate the shortest SIGNED distance between Two Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const RectangularCuboid& cuboid1, const RectangularCuboid& cuboid2){
    double result{0.0};
    return result;
}


/**
 * \brief Calculate the shortest SIGNED distance between Two AABB
 */ 
const double getDistance(const AABB& aabb1, const AABB& aabb2){
    double result{0.0};
    return result;
}


/**
 * Find the unsigned distance of a Point from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double Point::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(*this, (shape_ptr->getAsCuboid()));
    }
    return result;
}


/**
 * Find the unsigned distance of a LineSegment from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double LineSegment::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        //result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        //result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        //result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        //result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(*this, (shape_ptr->getAsCuboid()));
    }
    return result;
}


/**
 * Find the unsigned distance of a RectangularSurfacePatch from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double RectangularSurfacePatch::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(*this, (shape_ptr->getAsCuboid()));
    }
    return result;
}


/**
 * Find the unsigned distance of a Sphere from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double Sphere::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(*this, (shape_ptr->getAsCuboid()));
    }
    return result;
}


/**
 * Find the unsigned distance of a RectangularCuboid from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double RectangularCuboid::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        result = Geometry::getDistance(*this, *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(*this, (shape_ptr->getAsCuboid()));
    }
    return result;
}


/**
 * Find the unsigned distance of a RectangularCuboid from:
* - AABB
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double AABB::getDistance(const Shape& other) const{
    double result{0.0};
    if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), *shape_ptr);
    }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), *shape_ptr);
    }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), *shape_ptr);
    }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), *shape_ptr);
    }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), *shape_ptr);
    }else if(const AABB* shape_ptr = dynamic_cast<const AABB*>(&other)){
        result = Geometry::getDistance(this->getAsCuboid(), (shape_ptr->getAsCuboid()));
    }
    return result;
}



/**
 * Find the Relative State of two Shapes
 */
const IntersectData Point::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData LineSegment::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData RectangularSurfacePatch::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData Sphere::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData RectangularCuboid::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData AABB::Intersect(const Shape& other) const{
    double distance = this->getDistance(other);
    return IntersectData(distance);
}

/**
 * \brief Find the Bounding Sphere
 */
const Sphere Point::BoundingSphere() const{
    return Sphere(*this, 0.0);
    assertm(false, "Do not use!");
}


/**
 * \brief Find the Bounding Sphere
 */
const Sphere LineSegment::BoundingSphere() const{
    
    double x = (this->getEnds().first.getX() + this->getEnds().second.getX())/2.0;
    double dx = (this->getEnds().second.getX() - this->getEnds().first.getX());
    double y = (this->getEnds().first.getY() + this->getEnds().second.getY())/2.0;
    double dy = (this->getEnds().second.getY() - this->getEnds().first.getY());
    double z = (this->getEnds().first.getZ() + this->getEnds().second.getZ())/2.0;
    double dz = (this->getEnds().second.getZ() - this->getEnds().first.getZ());

    double radius = std::sqrt(dx * dx + dy * dy + dz * dz) / 2.0;
    Point center({x,y,z});
    return Sphere(center,radius);
}

/**
 * \brief Find the Bounding Sphere
 */
const Sphere RectangularSurfacePatch::BoundingSphere() const{

    double x{0.0}, y{0.0}, z{0.0};
    for(const auto& point: this->getCorners()){
        x += point.getX();
        y += point.getY();
        z += point.getZ();
    }
    x /= this->getCorners().size();
    y /= this->getCorners().size();
    z /= this->getCorners().size();
    Point center({x,y,z});

    double radius{std::max(this->getCorners()[2].getDistance(this->getCorners()[0])
                            , this->getCorners()[3].getDistance(this->getCorners()[1])
                            )};
    return Sphere(center,radius);

}

/**
 * \brief Find the Bounding Sphere
 */
const Sphere Sphere::BoundingSphere() const{
    return *this;
}


/**
 * \brief Find the Bounding Sphere
 */
const Sphere RectangularCuboid::BoundingSphere() const{
    Point p1 = this->corner_;
    Point p2 = this->corner_ + this->edges_[0] + this->edges_[2] + this->edges_[3];
    LineSegment diagonal(std::make_pair(p1, p2));
    return diagonal.BoundingSphere();
}

/**
 * \brief Find the Bounding Sphere
 */
const Sphere AABB::BoundingSphere() const{
    return this->cuboid_->BoundingSphere();
}



} // namespace Geometry