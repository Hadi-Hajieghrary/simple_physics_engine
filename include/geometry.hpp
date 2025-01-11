#ifndef __GEOMETRY_HPP_
#define __GEOMETRY_HPP_

#define assertm(exp, msg) assert((void(msg), exp))


#include "basic_geometry.hpp"

#include <vector>
#include <array>
#include <cmath>
#include <cassert>
#include <utility>
#include <memory>
#include <Eigen/Dense>


namespace Geometry{


static constexpr double eps{1e-10};

/**
 * Forward Dicelarations
 */
class Shape;
class Point;
class LineSegment;
class RectangularSurfacePatch;
class Sphere;
class RectangularCuboid;
class AABB;

/**
 * Get the faces of a Cuboid
 */
void getFaces(const RectangularCuboid& cupoid, std::array<std::unique_ptr<RectangularSurfacePatch>,6>& cupoid_faces);


/**
 * \brief Calculate the unsigned distance between Two Point
 */
const double getDistance(const Point& point1, const Point& point2);

/**
 * \brief Calculate the shortest unsigned distance between a point and a line segment
 */
const double getDistance(const Point& point, const LineSegment& line_segment);
const double getDistance(const LineSegment& line_segment, const Point& point);


/**
 * \brief Calculate the shortest SIGNED distance from a point to a rectanle surface patch
 * Its sign is equal to the sign of the dot product between the normal to the surface patch
 * and the vector from the point to any point on the surface patch
 * Distance From Point TO Surface Patch
 */
const double getDistance(const Point& point, const RectangularSurfacePatch& surface_patch);

/**
 * \brief Calculate the shortest SIGNED distance a rectanle surface patch to a point
 * Its sign is equal to the sign of the dot product between the normal to the surface patch
 * and the vector from any point on the surface patch the point
 * Distance From Surface Patch TO Point
 */
const double getDistance(const RectangularSurfacePatch& surface_patch, const Point& point);

/**
 * \brief Calculate the shortest SIGNED distance from a POINT to a SPHERE
 * Its sign is equal to the sign of the dot product of the vector from the POINT to any point on the SPHERE 
 * and the normal to the Sphere. 
 * The distance is Positive if the Point is inside of the SPHERE, so the distance is in the 
 * direction of the surface normal, and 
 * The distance is Negative if the Point is outside of the SPHERE, so the distance is in the 
 * reverse direction of the surface normal.
 * * Distance From POINT to a SPHERE
 */
const double getDistance(const Point& point, const Sphere& sphere);

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
const double getDistance(const Sphere& sphere, const Point& point);

/**
 * \brief Calculate the shortest SIGNED distance from a POINT to a RECTANGULAR CUBOID
 * If the point is Inside the RECTANGULAR CUBOID the distance is Positive, and,
 * If the point is Outside the RECTANGULAR CUBOID the distance is Negative
 * Distance from a POINT to a RECTANGULAR CUBOID
 */
const double getDistance(const Point& point, const RectangularCuboid& cuboid);

/**
 * \brief Calculate the shortest SIGNED distance from a RECTANGULAR CUBOID to a POINT
 * If the point is Inside the RECTANGULAR CUBOID the distance is Negative, and,
 * If the point is Outside the RECTANGULAR CUBOID the distance is Positive
 * Distance from a RECTANGULAR CUBOID to a POINT
 */
const double getDistance(const RectangularCuboid& cuboid, const Point& point);

/**
 * \brief Calculate the shortest UNSIGNED distance between two Line Segments
 * The Line Segments can be part of Two Parallel lines, Tow Intersecting Lines, 
 * or Two Skew Lines. In case of Intersecting or skew line, the closest points on 
 * the line segments are the ones closest to the common prependiculat line. 
 * In case of two parallel lines the the candidates for the closest points are 
 * the end points of each line segments 
 */
const double getDistance(const LineSegment& line_segment1, const LineSegment& line_segment2);

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const LineSegment& line_segment, const RectangularSurfacePatch& surface_patch);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Line Segment
 * \todo Implement
 */ 
const double getDistance( const RectangularSurfacePatch& surface_patch, const LineSegment& line_segment);

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Sphere
 * If the Line Segment is completely Inside of the Sphere, then, the distance will be positive.
 * If the Line Segment is completely Outside of the Sphere, then, the distance will Negative.
 * If the Line Segmen Intersects the Sphere, then, the distance will be zero
 * 
 */ 
const double getDistance(const LineSegment& line_segment, const Sphere& sphere);

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Line Segment
 * If the Line Segment is completely Inside of the Sphere, then, the distance will be Negative.
 * If the Line Segment is completely Outside of the Sphere, then, the distance will Positive.
 * If the Line Segmen Intersects the Sphere, then, the distance will be zero
 * 
 */ 
const double getDistance(const Sphere& sphere, const LineSegment& line_segment);

/**
 * \brief Calculate the shortest SIGNED distance between a Line Segment and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const LineSegment& line_segment, const RectangularCuboid& cuboid);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Line Segment
 * \todo Implement
 */ 
const double getDistance( const RectangularCuboid& cuboid, const LineSegment& line_segment);

/**
 * \brief Calculate the shortest SIGNED distance between Two Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch1, const RectangularSurfacePatch& surface_patch2);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Sphere
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch, const Sphere& sphere);

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const Sphere& sphere, const RectangularSurfacePatch& surface_patch);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Surface Patch and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const RectangularSurfacePatch& surface_patch, const RectangularCuboid& cuboid);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Rectangular Surface Patch
 * \todo Implement
 */ 
const double getDistance(const RectangularCuboid& cuboid, const RectangularSurfacePatch& surface_patch);

/**
 * \brief Calculate the shortest SIGNED distance between Two Spheres
 * \todo Implement
 */ 
const double getDistance(const Sphere& sphere, const Sphere& sphere2);

/**
 * \brief Calculate the shortest SIGNED distance between a Sphere and a Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const Sphere& sphere, const RectangularCuboid& cuboid);

/**
 * \brief Calculate the shortest SIGNED distance between a Rectangular Cuboid and a Sphere
 * \todo Implement
 */ 
const double getDistance(const RectangularCuboid& cuboid, const Sphere& sphere);

/**
 * \brief Calculate the shortest SIGNED distance between Two Rectangular Cuboid
 * \todo Implement
 */ 
const double getDistance(const RectangularCuboid& cuboid1, const RectangularCuboid& cuboid2);


/**
 * Class for Intersect Data
 */
class IntersectData{

    private:

        bool intersect_;
        double distance_;

    protected:


    public:

        /**
         * Constructor
         */
        IntersectData() = delete;

        /**
         * Constructor
         * Set the Minimum Distance of two shapes
         */
        IntersectData(double distance):
        distance_{distance}, intersect_{distance==0}
        {

        }

        /**
         * Coppy Constructor
         */
        IntersectData(IntersectData& other) = default;

        /**
         * Move Constructor
         */
        IntersectData(IntersectData&& other) = default;

        /**
         * Coppy Assignment
         */
        IntersectData& operator= (IntersectData& other) = default;

        /**
         * Move Assignment
         */
        IntersectData& operator=(IntersectData&& other) = default;

        /**
         * Virtual Destructor
         */
        virtual ~IntersectData() = default;

        /**
         * Get the Minimum Distance
         */
        const double getDistance() const{
            return distance_;
        }

        /**
         * Get the Minimum Distance
         */
        const bool doesIntersect() const{
            return intersect_;
        }

};


/**
 * Base Class For Shape
 */
class Shape{

    private:


    protected:

        /**
         * Constructor
         */
        Shape() = default;


    public:


        /**
         * Copy Constructor
         */
        Shape(const Shape& other) = default;

        /**
         * Move Constructor
         */
        Shape(Shape&& other) noexcept = default;

        /**
         * Copy Assignment
         */
        Shape& operator=(const Shape& other) = default;

        /**
         * Move Assignment
         */
        Shape& operator=(Shape&& other) noexcept = default;

        /**
         * Virtual Destructor
         */
        virtual ~Shape() = default;

        /**
         * Resize a Shape
         */
        virtual Shape& resize(const double ratio) = 0;

        /**
         * Translate the Shape
         */
        virtual Shape& translate(const Eigen::Vector3d& translation_vector) = 0;

        /**
         * Rotate the Shape
         */
        virtual Shape& rotate(const Eigen::Matrix3d& rotation_matrix) = 0;

        /**
         * Transform a Shape
         */
        virtual Shape& transform(const Eigen::Matrix4d& transformation_matrix) = 0;

        /**
         * Get distance of Two Shapes
         */
        virtual const double getDistance(const Shape& other) const = 0;

        /**
         * Find the Relative State of two Shapes
         */
        virtual const IntersectData Intersect(const Shape& other) const= 0;

        /**
         * Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const = 0;


}; //class Shape


/**
 * Point
 */
class Point: public Shape{

    private:
        Eigen::Vector3d coordinates_;

        /**
         * Resize a Point - DELETED
         */
        virtual Point& resize(const double ratio) override final{
            return *this;
            assertm(false, "Do not use!");
        }

        virtual const Sphere BoundingSphere() const override final;

    protected:


    public:

        /**
         * \brief Constructor
         */
        Point()
        :coordinates_{Eigen::Vector3d({0.0, 0.0, 0.0})}
        {
            
        }

        /**
         * \brief Constructor
         */
        Point(const Eigen::Vector3d& coordinates)
        : coordinates_{coordinates_}
        {}

        /**
         * \brief Copy Constructor
         */
        Point(const Point& other)
            : coordinates_{other.coordinates_}
        {}

        /**
         * \brief Move Constructor
         */
        Point(Point&& other) noexcept
            : coordinates_(std::move(other.coordinates_)) {
            other.coordinates_ = Eigen::Vector3d({0.0, 0.0, 0.0});
        }

        /**
         * \brief Copy Assignment
         */
        Point& operator=(const Point& other) {
            if (this != &other) {  // self-assignment check
                coordinates_ = other.coordinates_;
            }
            return *this;
        }

        /**
         * \brief Move Assignment
         */
        Point& operator=(Point&& other) noexcept {
            if (this != &other) {  // self-assignment check
                coordinates_ = std::move(other.coordinates_);
                other.coordinates_ = Eigen::Vector3d({0.0, 0.0, 0.0});
            }
            return *this;
        }

        /**
         * \brief Virtual Destructor
         */
        virtual ~Point() = default;


        /**
         * \brief Translate the Point
         */
        virtual Point& translate(const Eigen::Vector3d& translation_vector) override final{
            coordinates_ += translation_vector;
            return *this;
        }

        /**
         * \brief Rotate the Point
         */
        virtual Point& rotate(const Eigen::Matrix3d& rotation_matrix) override final{
            coordinates_ = rotation_matrix * coordinates_;
            return *this;
        }

        /**
         * \brief Transform a Point
         */
        virtual Point& transform(const Eigen::Matrix4d& transformation_matrix) override final{
            Eigen::Vector3d translation_vector = transformation_matrix.block<3, 1>(0, 3);
            this->translate(translation_vector);
            Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            this->rotate(rotation_matrix);
            return *this;
        }

        /**
         * \brief Get Coordinates
         */
        const Eigen::Vector3d& getCoordinates() const{
            return coordinates_;
        }

        /**
         * \brief Set Coordinates
         */
        void setCoordinates(const Eigen::Vector3d& coordinates){
            coordinates_ = coordinates;
        }

        double getX() const{return coordinates_[0];}
        double getY() const{return coordinates_[1];}
        double getZ() const{return coordinates_[2];}

        void setX(double x){coordinates_[0] = x;}
        void setY(double y){coordinates_[0] = y;}
        void setZ(double z){coordinates_[0] = z;}

        Point operator*(const double ratio) const{
            return Point(ratio * coordinates_);
        }

        friend Point operator*(const double& ratio, const Point& point);

        Point operator/(const double ratio) const{
            if(ratio != 0.0) return Point(coordinates_/ratio);
            else throw std::runtime_error("Devision by Zero");
        }

        Point operator+(const Eigen::Vector3d& vec) const{
            return Point(coordinates_ + vec);
        }

        Point operator-(const Eigen::Vector3d& vec) const{
            return Point(coordinates_ - vec);
        }

        Eigen::Vector3d operator-(const Point& point) const{
            return (coordinates_ - point.getCoordinates());
        }
        
        /**
         * \brief Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const override final;

        // /**
        //  * Find the Relative State of two Shapes- DELETED
        //  */
        virtual const IntersectData Intersect(const Shape& other) const override final;


}; // class Point


/**
 * LineSegment
 */
class LineSegment: public Shape{

    private:
        std::pair<Point, Point> ends_;

    protected:


    public:

        /**
         * \brief Constructor - DELETED
         */
        LineSegment() = delete;

        /**
         * \brief Constructor
         * 
         * \param std::pair of {Line Segment START Point, Line Segment END Point}
         */
        LineSegment(const std::pair<const Point&, const Point&>& ends)
        : ends_{ends}
        {}

        /**
         * \brief Copy Constructor
         */
        LineSegment(const LineSegment& other)
            : ends_{other.ends_}
        {}

        /**
         * \brief Move Constructor
         */
        LineSegment(LineSegment&& other) noexcept
            : ends_(std::move(other.ends_)) {
            other.ends_ = std::make_pair(Point(), Point());
        }

        /**
         * \brief Copy Assignment
         */
        LineSegment& operator=(const LineSegment& other) {
            if (this != &other) {  // self-assignment check
                ends_ = other.ends_;
            }
            return *this;
        }

        /**
         * \brief Move Assignment
         */
        LineSegment& operator=(LineSegment&& other) noexcept {
            if (this != &other) {  // self-assignment check
                ends_ = std::move(other.ends_);
                other.ends_ = std::make_pair(Point(), Point());
            }
            return *this;
        }

        /**
         * \brief Virtual Destructor
         */
        virtual ~LineSegment() = default;

        /**
         * \brief Resize the LineSegment
         * 
         * \param ratio Resize Ratio. If it was negative it will change the direction of the line segment
         * If the ratio was zero it will raise a exception
         */
        virtual LineSegment& resize(const double ratio)  override final{
            if(ratio == 0.0) throw std::runtime_error("The Line Segment will collapse to a Point!");
            if(ratio>0){
                ends_.first = ratio * ends_.first;
                ends_.second = ratio * ends_.second;
            }else{
                ends_.first = ratio * ends_.second;
                ends_.second = ratio * ends_.first;
            }
            return *this;
        }

        /**
         * \brief Translate the LineSegment
         * 
         * \param translation_vector
         */
        virtual LineSegment& translate(const Eigen::Vector3d& translation_vector) override final{
            ends_.first.translate(translation_vector);
            ends_.second.translate(translation_vector);
            return *this;
        }

        /**
         * \brief Rotate the LineSegment
         * 
         * \param rotation_matrix
         */
        virtual LineSegment& rotate(const Eigen::Matrix3d& rotation_matrix) override final{
            ends_.first.rotate(rotation_matrix);
            ends_.second.rotate(rotation_matrix);
            return *this;
        }

        /**
         * \brief Transform a Point
         * 
         * \param transformation_matrix
         */
        virtual LineSegment& transform(const Eigen::Matrix4d& transformation_matrix) override final{
            Eigen::Vector3d translation_vector = transformation_matrix.block<3, 1>(0, 3);
            this->translate(translation_vector);
            Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            this->rotate(rotation_matrix);
            return *this;
        }


        const std::pair<Point, Point>& getEnds() const{
            return ends_;
        }

        /**
         * \brief Get unsigned Length of the Line Segment
         */
        const double getLength() const{
            return Geometry::getDistance(ends_.second, ends_.first);
        }

        /**
         * \brief Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const  override final;

        /**
         * \brief Find the Relative State of two Shapes
         */
        virtual const IntersectData Intersect(const Shape& other) const override final;

        /**
         * \brief Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const override final;

}; //class LineSegment


/**
 * RectangularSurfacePatch
 */
class RectangularSurfacePatch: public Shape{

    private:

        std::array<Point, 4> corners_;

    protected:


    public:

        /**
         * Constructor
         */
        RectangularSurfacePatch() = delete;

        /**
         * Constructor
         */
        RectangularSurfacePatch(const std::array<Point, 3> corners)
        {
            if((corners[0] - corners[1]).dot(corners[0] - corners[2])<=Geometry::eps){
                corners_ = {corners[0], corners[1], corners[1] + (corners[2] - corners[0]) ,corners[2]};
            }else if((corners[1] - corners[0]).dot(corners[1] - corners[2])<=Geometry::eps){
                corners_ = {corners[1], corners[2], corners[2] + (corners[0] - corners[1]) ,corners[0]};
            }else if((corners[2] - corners[0]).dot(corners[2] - corners[1])<=Geometry::eps){
                corners_ = {corners[2], corners[0], corners[0] + (corners[1] - corners[2]) ,corners[1]};
            }else{
                throw std::runtime_error("The corners do not constitute a RectangularSurfacePatch!");
            }
        }

        /**
         * Copy Constructor
         */
        RectangularSurfacePatch(const RectangularSurfacePatch& other)
            : corners_{other.corners_} 
        {}

        /**
         * Move Constructor
         */
        RectangularSurfacePatch(RectangularSurfacePatch&& other) noexcept
            : corners_(std::move(other.corners_)) {
            other.corners_ = std::array<Point, 4>({Point(),Point(), Point(), Point()});
        }

        /**
         * Copy Assignment
         */
        RectangularSurfacePatch& operator=(const RectangularSurfacePatch& other) {
            if (this != &other) {  // self-assignment check
                corners_ = other.corners_;
            }
            return *this;
        }

        /**
         * Move Assignment
         */
        RectangularSurfacePatch& operator=(RectangularSurfacePatch&& other) noexcept {
            if (this != &other) {  // self-assignment check
                corners_ = std::move(other.corners_);
                other.corners_ = std::array<Point, 4>({Point(),Point(), Point(), Point()});
            }
            return *this;
        }

        /**
         * Virtual Destructor
         */
        virtual ~RectangularSurfacePatch() = default;

        /**
         * Resize the RectangularSurfacePatch
         */
        virtual RectangularSurfacePatch& resize(const double ratio) override final{
            for(auto& corner: corners_){
                corner = ratio * corner;
            }
            return *this;
        }

        /**
         * Translate the RectangularSurfacePatch
         */
        virtual RectangularSurfacePatch& translate(const Eigen::Vector3d& translation_vector)  override final{
            for(auto& corner: corners_){
                corner.translate(translation_vector);
            }
            return *this;
        }

        /**
         * Rotate the RectangularSurfacePatch
         */
        virtual RectangularSurfacePatch& rotate(const Eigen::Matrix3d& rotation_matrix)  override final{
            for(auto& corner: corners_){
                corner.rotate(rotation_matrix);
            }
            return *this;
        }

        /**
         * Transform the Shape
         */
        virtual RectangularSurfacePatch& transform(const Eigen::Matrix4d& transformation_matrix) override final{
            Eigen::Vector3d translation_vector = transformation_matrix.block<3, 1>(0, 3);
            this->translate(translation_vector);
            Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            this->rotate(rotation_matrix);
            return *this;
        }

        const std::array<Point, 4>& getCorners() const{
            return corners_;
        }

        // const Eigen::Vector3d getNormal() const{

        // }

        /**
         * Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const override final;

        /**
         * Find the Relative State of two Shapes
         */
        virtual const IntersectData Intersect(const Shape& other) const override final;

        /**
         * \brief Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const override final;


}; // class RectangularSurfacePatch


/**
 * Sphere
 */
class Sphere: public Shape{

    private:

        Point center_;
        double radius_;


    protected:


    public:

        /**
         * Constructor
         */
        Sphere() = delete;

        /**
         * Constructor
         */
        Sphere(const Point& center, const double radius)
        : center_{center}, radius_{radius}
        {

        }

        /**
         * Copy Constructor
         */
        Sphere(const Sphere& other)
            : center_(other.center_), radius_(other.radius_) {}

        /**
         * Move Constructor
         */
        Sphere(Sphere&& other) noexcept
            : center_(std::move(other.center_)), radius_(other.radius_) {
            other.center_ = Point();
            other.radius_ = 0;
        }

        /**
         * Copy Assignment
         */
        Sphere& operator=(const Sphere& other) {
            if (this != &other) {  // self-assignment check
                center_ = other.center_;
                radius_ = other.radius_;
            }
            return *this;
        }

        /**
         * Move Assignment
         */
        Sphere& operator=(Sphere&& other) noexcept {
            if (this != &other) {  // self-assignment check
                center_ = std::move(other.center_);
                other.center_ = Point();
                radius_ = other.radius_;
                other.radius_ = 0;
            }
            return *this;
        }

        /**
         * Resize the Sphere
         */
        virtual Sphere& resize(const double ratio) override final{
            radius_ *= ratio;
            return *this;
        }

        /**
         * Translate the Sphere
         */
        virtual Sphere& translate(const Eigen::Vector3d& translation_vector)  override final{
            center_.translate(translation_vector);
            return *this;
        }

        /**
         * Rotate the Sphere
         */
        virtual Sphere& rotate(const Eigen::Matrix3d& rotation_matrix)  override final{
            center_.rotate(rotation_matrix);
            return *this;
        }

        /**
         * Transform a Sphere
         */
        virtual Sphere& transform(const Eigen::Matrix4d& transformation_matrix)  override final{
            Eigen::Vector3d translation_vector = transformation_matrix.block<3, 1>(0, 3);
            this->translate(translation_vector);
            Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            this->rotate(rotation_matrix);
            return *this;
        }

        /**
         * Get the Radius of the Sphere
         */
        const double getRadus() const{
            return radius_;
        }

        /**
         * Get the Center of the Sphere
         */
        const Point& getCenter() const{
            return center_;
        }

        /**
         * Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const override final;


        /**
         * Find the Relative State of two Shapes
         */
        virtual const IntersectData Intersect(const Shape& other) const override final;

        /**
         * \brief Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const override final;


}; //class Sphere


/**
 * RectangularCuboid
 */
class RectangularCuboid: public Shape{

    private:

        Point corner_;
        std::array<Eigen::Vector3d, 3> edges_;

    protected:


    public:

        /**
         * Constructor
         */
        RectangularCuboid() = delete;

        /**
         * Constructor
         */
        RectangularCuboid(const Point& corner, const std::array<Eigen::Vector3d, 3>& sides)
        : corner_{corner}, edges_{sides}
        {
            assertm(edges_[0].dot(edges_[1])<eps, "Edge 0 and Edge 1 of the RectangularCuboid are not Orthogonal!");
            assertm(edges_[0].dot(edges_[2])<eps, "Edge 0 and Edge 2 of the RectangularCuboid are not Orthogonal!");
            assertm(edges_[1].dot(edges_[2])<eps, "Edge 1 and Edge 2 of the RectangularCuboid are not Orthogonal!");
        }

        /**
         * Copy Constructor
         */
        RectangularCuboid(const RectangularCuboid& other)
            : corner_{other.corner_}, edges_{other.edges_}{}

        /**
         * Move Constructor
         */
        RectangularCuboid(RectangularCuboid&& other) noexcept
            : corner_(std::move(other.corner_)), edges_(std::move(other.edges_)) {
                other.corner_ = Point();
                other.edges_ = std::array<Eigen::Vector3d, 3>({Eigen::Vector3d({0.0, 0.0, 0.0}),
                                                                Eigen::Vector3d({0.0, 0.0, 0.0}),
                                                                Eigen::Vector3d({0.0, 0.0, 0.0})});
        }

        /**
         * Copy Assignment
         */
        RectangularCuboid& operator=(const RectangularCuboid& other) {
            if (this != &other) {  // self-assignment check
                corner_ = other.corner_;
                edges_ = other.edges_;
            }
            return *this;
        }

        /**
         * Move Assignment
         */
        RectangularCuboid& operator=(RectangularCuboid&& other) noexcept {
            if (this != &other) {  // self-assignment check
                corner_ = std::move(other.corner_);
                other.corner_ = Point();
                edges_ = std::move(other.edges_);
                other.edges_ = std::array<Eigen::Vector3d, 3>({Eigen::Vector3d({0.0, 0.0, 0.0}),
                                                                Eigen::Vector3d({0.0, 0.0, 0.0}),
                                                                Eigen::Vector3d({0.0, 0.0, 0.0})});
            }
            return *this;
        }

        /**
         * Resize a RectangularCuboid
         */
        virtual RectangularCuboid& resize(const double ratio) override final{
            for(auto& edge: edges_){
                edge*=ratio;
            }
            return *this;
        }

        /**
         * Translate the RectangularCuboid
         */
        virtual RectangularCuboid& translate(const Eigen::Vector3d& translation_vector)  override final{
            corner_ = corner_ + translation_vector;
            return *this;
        }

        /**
         * Rotate the RectangularCuboid
         */
        virtual RectangularCuboid& rotate(const Eigen::Matrix3d& rotation_matrix)  override final{
            corner_.rotate(rotation_matrix);
            for(auto& edge: edges_){
                edge = rotation_matrix * edge;
            }
            return *this;
        }

        /**
         * Transform a RectangularCuboid
         */
        virtual RectangularCuboid& transform(const Eigen::Matrix4d& transformation_matrix)  override final{
            Eigen::Vector3d translation_vector = transformation_matrix.block<3, 1>(0, 3);
            this->translate(translation_vector);
            Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            this->rotate(rotation_matrix);
            return *this;
        }

        /**
         * Get the Edges of the RectangularCuboid
         */
        const std::array<Eigen::Vector3d, 3>& getEdges() const{
            return edges_;
        }

        /**
         * Get the Corner of the RectangularCuboid
         */
        const Point& getCorner() const{
            return corner_;
        }


        /**
         * Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const override final;


        /**
         * Find the Relative State of two RectangularCuboid
         */
        virtual const IntersectData Intersect(const Shape& other) const override final;


        /**
         * \brief Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const override final;


}; //class RectangularCuboid



/**
 * Axis Aligned Bounding Box
 */
class AABB: public Shape{

    private:

        std::shared_ptr<RectangularCuboid> cuboid_{nullptr};
        std::pair<Point, Point> corners_;

        /**
         * Rotate the AABB
         */
        virtual AABB& rotate(const Eigen::Matrix3d& rotation_matrix)  override final{
            assertm(false, "Do not use!");
            return *this;
        }


        /**
         * Transform a AABB
         */
        virtual AABB& transform(const Eigen::Matrix4d& transformation_matrix)  override final{
            assertm(false, "Do not use!");
            return *this;
        }

    protected:


    public:

        /**
         * Constructor
         */
        AABB() = delete;

        /**
         * Constructor
         */
        AABB(const Point& corner1, const Point& corner2)
        {
            assertm((corner2 - corner1)[0]<Geometry::eps, "The Diagonal of the AABB is Parallel to X axis");
            assertm((corner2 - corner1)[1]<Geometry::eps, "The Diagonal of the AABB is Parallel to Y axis");
            assertm((corner2 - corner1)[2]<Geometry::eps, "The Diagonal of the AABB is Parallel to Z axis");
            
            std::array<Eigen::Vector3d, 3> edges{Eigen::Vector3d({(corner2 - corner1)[0], 0.0, 0.0}),
                                                    Eigen::Vector3d({0.0, (corner2 - corner1)[1], 0.0}),
                                                    Eigen::Vector3d({0.0, 0.0, (corner2 - corner1)[2]})
                                                 };
            cuboid_ = std::make_shared<RectangularCuboid>(corner1, edges);
            corners_ = std::make_pair(corner1, corner2);
        }

        /**
         * Copy Constructor
         */
        AABB(const AABB& other)
            : cuboid_{std::make_shared<RectangularCuboid>(*(other.cuboid_))}, corners_{other.corners_}
        {}

        /**
         * Move Constructor
         */
        AABB(AABB&& other) noexcept
            : cuboid_{other.cuboid_}, corners_{other.corners_} {
                other.cuboid_ = nullptr;
                other.corners_ = std::make_pair(Point(), Point());
        }

        /**
         * Copy Assignment
         */
        AABB& operator=(const AABB& other) {
            if (this != &other) {  // self-assignment check
                cuboid_ = std::make_shared<RectangularCuboid>(*(other.cuboid_));
            }
            return *this;
        }

        /**
         * Move Assignment
         */
        AABB& operator=(AABB&& other) noexcept {
            if (this != &other) {  // self-assignment check
                cuboid_ = other.cuboid_;
                other.cuboid_ = nullptr; 
                corners_ = other.corners_;
                other.corners_ = std::make_pair(Point(), Point());
            }
            return *this;
        }

        /**
         * Resize a AABB
         */
        virtual AABB& resize(const double ratio) override final{
            this->cuboid_->resize(ratio);
            double new_x = (this->corners_.second.getX() - this->corners_.first.getX()) * ratio;
            double new_y = (this->corners_.second.getY() - this->corners_.first.getY()) * ratio;
            double new_z = (this->corners_.second.getZ() - this->corners_.first.getZ()) * ratio;
            this->corners_.second = Point({new_x, new_y, new_z});
            return *this;
        }

        /**
         * Translate the AABB
         */
        virtual AABB& translate(const Eigen::Vector3d& translation_vector)  override final{
            this->cuboid_->translate(translation_vector);
            this->corners_.first->translate(translation_vector);
            this->corners_.second->translate(translation_vector);
            return *this;
        }

        /**
         * Get the Edges of the AABB
         */
        const std::array<Eigen::Vector3d, 3>& getEdges() const{
            return this->cuboid_->getEdges();
        }

        /**
         * Get the Corner of the AABB
         */
        const std::pair<Point, Point>& getCorner() const{
            return this->corners_;
        }

        /**
         * \brief Get the RectangularCuboid 
         */
        const RectangularCuboid& getAsCuboid() const{
            return *(this->cuboid_);
        }

        /**
         * Find the Distance of a AABB from:
         * - AABB
         * - Point
         * - LineSegment
         * - RectangularSurfacePatch
         * - Sphere
         * - RectangularCuboid
         */
        virtual const double getDistance(const Shape& other) const override final;

        /**
         * Find the Relative State of two RectangularCuboid
         */
        virtual const IntersectData Intersect(const Shape& other) const override final;

        /**
         * \brief Find the Bounding Sphere
         */
        virtual const Sphere BoundingSphere() const override final;


}; //class RectangularCuboid



} // namespace Geometry




#endif // __SHAPE_HPP_