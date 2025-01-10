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
 * Calculate the shortest unsigned distance from a point to a cuboid
 */
const double getDistance(const RectangularCuboid& cuboid, const Point& point){
    const double result = - getDistance(point, cuboid);
    return result;
}




/**
 * Find the unsigned distance of a Point from:
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
    }
    return result;
}


/**
 * Find the unsigned distance of a LineSegment from:
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double LineSegment::getDistance(const Shape& other) const{
    double result{0.0};
    // if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
    //     result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }
    return result;
}


/**
 * Find the unsigned distance of a RectangularSurfacePatch from:
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double RectangularSurfacePatch::getDistance(const Shape& other) const{
    double result{0.0};
    // if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
    //     result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }
    return result;
}


/**
 * Find the unsigned distance of a Sphere from:
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double Sphere::getDistance(const Shape& other) const{
    double result{0.0};
    // if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
    //     result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }
    return result;
}



/**
 * Find the unsigned distance of a RectangularCuboid from:
* - Point
* - LineSegment
* - RectangularSurfacePatch
* - Sphere
* - RectangularCuboid
 */
const double RectangularCuboid::getDistance(const Shape& other) const{
    double result{0.0};
    // if(const Point* shape_ptr = dynamic_cast<const Point*>(&other)){
    //     result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const LineSegment* shape_ptr = dynamic_cast<const LineSegment*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularSurfacePatch* shape_ptr = dynamic_cast<const RectangularSurfacePatch*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const Sphere* shape_ptr = dynamic_cast<const Sphere*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }else if(const RectangularCuboid* shape_ptr = dynamic_cast<const RectangularCuboid*>(&other)){
    //     //result = Geometry::getDistance(*this, *shape_ptr);
    // }
    return result;
}


/**
 * Find the Relative State of two Shapes
 */
const IntersectData Point::Intersect(const Shape& other) const{
    double distance{0.0};
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData LineSegment::Intersect(const Shape& other) const{
    double distance{0.0};
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData RectangularSurfacePatch::Intersect(const Shape& other) const{
    double distance{0.0};
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData Sphere::Intersect(const Shape& other) const{
    double distance{0.0};
    return IntersectData(distance);
}

/**
 * Find the Relative State of two Shapes
 */
const IntersectData RectangularCuboid::Intersect(const Shape& other) const{
    double distance{0.0};
    return IntersectData(distance);
}


} // namespace Geometry