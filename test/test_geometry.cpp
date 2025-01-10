#include <gtest/gtest.h>
#include <basic_geometry.hpp>
#include "geometry.hpp"
#include <Eigen/Dense>



TEST(IntersectDataTest, DefaultConstructor) {
    Geometry::IntersectData intersect_data(5.0);  // Distance > 0, should not intersect.
    EXPECT_DOUBLE_EQ(intersect_data.getDistance(), 5.0);
    EXPECT_FALSE(intersect_data.doesIntersect());
}

TEST(IntersectDataTest, IntersectionConstructor_1) {
    Geometry::IntersectData intersect_data(0.0);  // Distance = 0, should intersect.
    EXPECT_DOUBLE_EQ(intersect_data.getDistance(), 0.0);
    EXPECT_TRUE(intersect_data.doesIntersect());
}

TEST(IntersectDataTest, IntersectionConstructor_2) {
    Geometry::IntersectData intersect_data(-0.5);  // Distance < 0, should intersect.
    EXPECT_DOUBLE_EQ(intersect_data.getDistance(), -0.5);
    EXPECT_TRUE(intersect_data.doesIntersect());
}


TEST(SphereTest, Constructor) {
    Geometry::Point center({0.0, 0.0, 0.0});
    double radius = 5.0;
    Geometry::Sphere Sphere(center, radius);

    EXPECT_EQ(Sphere.getRadus(), 5.0);
    EXPECT_TRUE(Sphere.getCenter().getDistance(center)<=Geometry::eps);
}

TEST(SphereTest, Resize) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere(center, 5.0);
    Sphere.resize(2.0);  // Resize should double the radius

    EXPECT_EQ(Sphere.getRadus(), 10.0);
}

TEST(SphereTest, Translate) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere(center, 5.0);
    Eigen::Vector3d translation({1.0, 2.0, 3.0});
    
    Sphere.translate(translation);  // Translate by (1, 2, 3)

    Geometry::Point new_center = Sphere.getCenter();
    EXPECT_EQ(new_center.getX(), 1.0);
    EXPECT_EQ(new_center.getY(), 2.0);
    EXPECT_EQ(new_center.getZ(), 3.0);
}

TEST(SphereTest, Transform_1) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere(center, 5.0);
    const double roll{0.1}, pitch{1.0}, yaw{10.0};
    Eigen::Matrix4d transform = Geometry::createTransformationMatrix(roll, pitch, yaw, Eigen::Vector3d({0.0, 0.0, 0.0}));
    Sphere.transform(transform);
    EXPECT_TRUE(Sphere.getCenter().getDistance(center)<=Geometry::eps);
}

TEST(SphereTest, Transform_2) {
    Geometry::Point center({0.1, 1.0, 10.0});
    Geometry::Sphere Sphere(center, 5.0);
    const double roll{0.1}, pitch{1.0}, yaw{10.0};
    Eigen::Matrix4d transform = Geometry::createTransformationMatrix(roll, pitch, yaw, Eigen::Vector3d({0.0, 0.0, 0.0}));
    Sphere.transform(transform);

    double cos_r = cos(roll);
    double cos_p = cos(pitch);
    double cos_y = cos(yaw);
    double sin_r = sin(roll);
    double sin_y = sin(yaw); 
    double sin_p = sin(pitch); 

    double Axx = cos_y * cos_p;
    double Axy = cos_y * sin_p * sin_r - sin_y * cos_r;
    double Axz = cos_y * sin_p * cos_r + sin_y * sin_r;

    double Ayx = sin_y * cos_p;
    double Ayy = sin_y * sin_p * sin_r + cos_y * cos_r;
    double Ayz = sin_y * sin_p * cos_r - cos_y * sin_r;

    double Azx = -sin_p;
    double Azy = cos_p * sin_r;
    double Azz = cos_p * cos_r;

    double x = Axx * center.getX() + Axy * center.getY() + Axz * center.getZ();
    double y = Ayx * center.getX() + Ayy * center.getY() + Ayz * center.getZ();
    double z = Azx * center.getX() + Azy * center.getY() + Azz * center.getZ();

    EXPECT_TRUE(Sphere.getCenter().getDistance(Geometry::Point({x,y,z}))<=Geometry::eps);
}


TEST(SphereTest, Transform_3) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere(center, 5.0);
    const double roll{0.0}, pitch{0.0}, yaw{0.0};
    Eigen::Matrix4d transform = Geometry::createTransformationMatrix(roll, pitch, yaw, Eigen::Vector3d({10.0, 1.0, 0.1}));
    Sphere.transform(transform);
    EXPECT_TRUE((Sphere.getCenter().getDistance(Geometry::Point({10.0, 1.0, 0.1}))) <= Geometry::eps);
}

TEST(SphereTest, Transform_4) {
    Geometry::Point center({0.1, 1.0, 10.0});
    Geometry::Sphere Sphere(center, 5.0);
    const double roll{0.1}, pitch{1.0}, yaw{10.0};
    Eigen::Matrix4d transform = Geometry::createTransformationMatrix(roll, pitch, yaw, Eigen::Vector3d({10.0, 1.0, 0.1}));
    Sphere.transform(transform);

    double cos_r = cos(roll);
    double cos_p = cos(pitch);
    double cos_y = cos(yaw);
    double sin_r = sin(roll);
    double sin_y = sin(yaw); 
    double sin_p = sin(pitch); 

    double Axx = cos_y * cos_p;
    double Axy = cos_y * sin_p * sin_r - sin_y * cos_r;
    double Axz = cos_y * sin_p * cos_r + sin_y * sin_r;

    double Ayx = sin_y * cos_p;
    double Ayy = sin_y * sin_p * sin_r + cos_y * cos_r;
    double Ayz = sin_y * sin_p * cos_r - cos_y * sin_r;

    double Azx = -sin_p;
    double Azy = cos_p * sin_r;
    double Azz = cos_p * cos_r;
    auto new_center = center + Eigen::Vector3d({10.0, 1.0, 0.1});
    double x = Axx * new_center.getX() + Axy * new_center.getY() + Axz * new_center.getZ();
    double y = Ayx * new_center.getX() + Ayy * new_center.getY() + Ayz * new_center.getZ();
    double z = Azx * new_center.getX() + Azy * new_center.getY() + Azz * new_center.getZ();

    EXPECT_TRUE(Sphere.getCenter().getDistance(Geometry::Point({x,y,z}))<=Geometry::eps);
}


TEST(SphereTest, CopyConstructor) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere original(center, 5.0);
    Geometry::Sphere copy = original;

    EXPECT_EQ(copy.getRadus(), original.getRadus());
    EXPECT_TRUE(copy.getCenter().getDistance(original.getCenter())<=Geometry::eps);
}

TEST(SphereTest, MoveConstructor) {
    Geometry::Point center({0.0, 0.0, 0.0});
    Geometry::Sphere original(center, 5.0);
    Geometry::Sphere moved = std::move(original);

    EXPECT_EQ(moved.getRadus(), 5.0);
    EXPECT_TRUE(moved.getCenter().getDistance(original.getCenter())<=Geometry::eps);
}

TEST(SphereTest, CopyAssignment) {
    Geometry::Point center1({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere1(center1, 5.0);
    Geometry::Point center2({2.0, 2.0, 2.0});
    Geometry::Sphere Sphere2(center2, 10.0);

    Sphere2 = Sphere1;  // Copy assignment

    EXPECT_EQ(Sphere2.getRadus(), 5.0);
    EXPECT_TRUE(Sphere2.getCenter().getDistance(center1)<=Geometry::eps);
}

TEST(SphereTest, MoveAssignment) {
    Geometry::Point center1({0.0, 0.0, 0.0});
    Geometry::Sphere Sphere1(center1, 5.0);
    Geometry::Point center2({2.0, 2.0, 2.0});
    Geometry::Sphere Sphere2(center2, 10.0);

    Sphere2 = std::move(Sphere1);  // Move assignment

    EXPECT_EQ(Sphere2.getRadus(), 5.0);
    EXPECT_TRUE(Sphere2.getCenter().getDistance(center1)<=Geometry::eps);
}


TEST(SphereTest, GetDistance) {
    Geometry::Point center1({0.0, 0.0, 0.0});
    double radius1 = 5.0;
    Geometry::Sphere Sphere1(center1, radius1);

    Geometry::Point center2({10.0, 10.0, 10.0});
    double radius2 = 3.0;
    Geometry::Sphere Sphere2(center2, radius2);

    // Assume the function returns correct distance
    // Implement getDistance logic in the Sphere class first!
    double expected_distance = std::sqrt(
        std::pow(center2.getX() - center1.getX(), 2) + 
        std::pow(center2.getY() - center1.getY(), 2) + 
        std::pow(center2.getZ() - center1.getZ(), 2)
    ) - (radius1 + radius2);  // Adjusted by the radius for distance between surfaces

    EXPECT_TRUE((Sphere1.getDistance(Sphere2) - expected_distance)<=Geometry::eps);
}

TEST(SphereTest, Intersect) {
    Geometry::Point center1({0.0, 0.0, 0.0});
    double radius1 = 5.0;
    Geometry::Sphere Sphere1(center1, radius1);

    Geometry::Point center2({10.0, 10.0, 10.0});
    double radius2 = 3.0;
    Geometry::Sphere Sphere2(center2, radius2);

    const double expected_distance = sqrt(300.0) - radius1 - radius2;
    Geometry::IntersectData result = Sphere1.Intersect(Sphere2);
    
    EXPECT_TRUE((result.getDistance() - expected_distance)<=Geometry::eps);
    EXPECT_FALSE(result.doesIntersect());  // Based on example distance calculation
}
