#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

struct Cube {
  unsigned int type;
  geometry_msgs::Pose pose;
}

class LaundryMan {
public:
  std::vector<unsigned int> cubeTypes;
  void execute() {
    categorizeBuckets();
    sortCubes();
  };

  void categorizeBuckets() {
    // go to bucket 0, 1, 2
    ROS_INFO("categorizing");
    bool arrived = false;
    while (!arrived) {
      arrived = goToBucket(0);
    }
    queryCubeType();
    assignBucketType();
    arrived = false;

    while (!arrived) {
      arrived = goToBucket(1);
    }
    queryCubeType();
    assignBucketType();
    arrived = false;

    while (!arrived) {
      arrived = goToBucket(2);
    }
    queryCubeType();
    assignBucketType();
    arrived = false;
    ROS_INFO("categorized");
  }

  void sortCubes() {
    ROS_INFO("sorting");
    bool cubeLeft = true;
    while (cubeLeft) {
      bool arrived = false;
      while (!arrived) {
        arrived = goToBucket(3);
      }
      arrived = false;

      Cube cube;
      for (unsigned int i = 0; i < cubeTypes.size(); i++) {
        cubeLeft = findOneCubeByType(cubeTypes[i], cube);
        if (cubeLeft)
          break;
      }

      pickUp(cube);
      goToBucket(queryBucketId(cube.type));
      place();

      ROS_INFO("sorted");
    }
  }

  bool goToBucket(const unsigned int id){};
  void pickUp(const Cube cube){};
  void place(){};
  void queryBucketType(){};
  void assignBucketType(){};
  void queryCubeType(){};
  bool findOneCubeByType(const unsigned int type, Cube &cube){};
  unsigned int queryBucketIdByType(const unsigned int type){};
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "main_node");

  return 0;
}