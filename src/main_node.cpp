#include "geometry_msgs/Pose.h"
#include "laundryman/CubeInfo.h"
#include "laundryman/CubeInfoList.h"
#include "laundryman/MotionService.h"
#include "ros/ros.h"
#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;
// color to be changed according to the vision node
#define WHITE 45
#define RED 2
#define BLACK 1
#define UNUSED_CLOTH_COLOR 0xff
#define MAXMIUM_CUBE_NUMBER 3

#define CLOTH_TYPE 1
#define NO_CLOTH_RETURN 3

struct Cube {
  unsigned int color_id;
  geometry_msgs::Pose pose;
};

struct BucketInfo {
  unsigned int bucket_id;
  geometry_msgs::Pose pose;
};

class LaundryMan {

private:
  ros::NodeHandle nh;
  ros::ServiceClient client_motion;
  ros::Subscriber sub_cubeinfolist;
  std::vector<BucketInfo> bucket_map;

  PrologClient pl;
  unsigned int buckettype[3]; // index: id     value: color type
  bool msgRecvFlag;

public:
  LaundryMan(){};
  // std::vector<unsigned int> cubeTypes;
  struct Cube cubeinfolist[MAXMIUM_CUBE_NUMBER]; //
  struct Cube cubeinfo;                          //

  LaundryMan(ros::NodeHandle nh);

  void execute() {
    categorizeBuckets();
    ros::spinOnce();
    sortCubes();
  };

  void assignBucketType(int id, int color) { buckettype[id] = color; }

  // query function:
  // function: get the bucket ID from the color of cloth inside
  unsigned int query_color(std::string color) {

    ROS_INFO("query in ontology");

    pl.query("rdf_assert(ssy235Ontology:'temp',rdf:type,ssy235Ontology:" +
             color + ")");

    PrologQuery bdgs = pl.query(
        "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType1)");

    if (!(bdgs.begin() == bdgs.end())) {
      return WHITE;
    }
    bdgs = pl.query(
        "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType2)");

    if (!(bdgs.begin() == bdgs.end())) {
      return RED;
    }
    bdgs = pl.query(
        "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType3)");

    if (!(bdgs.begin() == bdgs.end())) {
      return BLACK;
    }
    return 0;
  };

  unsigned int queryBucketType() {
    // get cloth color
    ROS_INFO("query bucket type");
    std::string color_string = getClothColor();

    // query
    return query_color(color_string);
  };

  // timeout: 20 second, return false if no more message received from Vision
  // node
  bool checkCubeLeft(struct Cube cubeinfolist[]) {

    ros::spinOnce();

    if (msgRecvFlag) {
      return true;
    }

    ros::Duration(3).sleep(); // sleep for 3s and check it again
    ros::spinOnce();
    if (msgRecvFlag) {
      return true;
    }
    ros::Duration(3).sleep(); // repeat
    ros::spinOnce();
    if (msgRecvFlag) {
      return true;
    }

    return false;
  };

  void categorizeBuckets() {
    // go to bucket 0, 1, 2
    ROS_INFO("categorizing");
    bool arrived = false;
    ROS_INFO("categorizing bucket 0");
    while (!arrived) {
      arrived = goToBucket(0);
    }
    ros::Duration(3).sleep();
    ros::spinOnce();
    ROS_INFO("spin 0");
    assignBucketType(0, queryBucketType());

    arrived = false;
    ROS_INFO("categorizing bucket 1");
    while (!arrived) {
      arrived = goToBucket(1);
    }

    ros::Duration(3).sleep();
    ros::spinOnce();
    assignBucketType(1, queryBucketType());

    arrived = false;
    ROS_INFO("categorizing bucket 2");
    while (!arrived) {
      arrived = goToBucket(2);
    }

    ros::Duration(3).sleep();
    ros::spinOnce();
    assignBucketType(2, queryBucketType());
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

      msgRecvFlag = false; // prepare the flag for the receivin timeout check
      ROS_INFO("Reach clothes bucket to pick up");

      ros::Duration(3).sleep();
      ros::spinOnce();

      // check if cube left
      cubeLeft = checkCubeLeft(cubeinfolist);
      if (!cubeLeft) {
        ROS_INFO("No more cubes left");
        break;
      }

      ROS_INFO("pick the cube with color:%d\n", cubeinfolist[0].color_id);
      // pick up the first cloth
      pickUp(cubeinfolist[0]);

      // ros::spinOnce();
      // put it in the right bucket
      unsigned int targetId;
      targetId = queryBucketIdByType(cubeinfolist[0].color_id);
      goToBucket(targetId);

      // ros::Duration(3).sleep();
      place(targetId);

      ROS_INFO("sorted");
    }
  }

  bool goToBucket(const unsigned int id) {
    laundryman::MotionService req;

    req.request.bucketId = id;
    req.request.action = laundryman::MotionService::Request::MOVETOBUCKET;
    req.request.pose.position.x = 0;
    req.request.pose.position.y = 0;
    req.request.pose.position.z = 0;
    req.request.pose.orientation.x = 0;
    req.request.pose.orientation.y = 0;
    req.request.pose.orientation.z = 0;
    req.request.pose.orientation.w = 0;

    if (!client_motion.call(req)) {

      ROS_ERROR("Failed to call gotobucket");
      return false;
    }
    ROS_INFO("gotobucket send out");
    return true;
  };

  bool pickUp(const Cube cube) {
    laundryman::MotionService req;

    req.request.bucketId = 0;
    req.request.action = laundryman::MotionService::Request::PICKUP;
    req.request.pose = cube.pose;

    if (!client_motion.call(req)) {

      ROS_ERROR("Failed to call pickup");
      return false;
    }
    ROS_INFO("pickup send out");
    return true;
  };

  bool place(unsigned int id) {
    // to define a pose
    laundryman::MotionService req;

    req.request.bucketId = id;
    req.request.action = laundryman::MotionService::Request::PUTDOWN;

    // might to be modified: the postion should be align with vision node
    req.request.pose.position.x = 0.500;
    req.request.pose.position.y = 0.000 + (1 + (rand() % 3)) * 0.05;
    req.request.pose.position.z = 0.86;
    req.request.pose.orientation.x = -0.5481993;
    req.request.pose.orientation.y = 0.4501637;
    req.request.pose.orientation.z = 0.4551655;
    req.request.pose.orientation.w = 0.5381957;

    if (!client_motion.call(req)) {

      ROS_ERROR("Failed to call pickup");
      return 1;
    }
    return 0;
  };

  // to be define: return a color string wit
  std::string getClothColor() {
    std::string temp;
    if (WHITE == cubeinfolist[0].color_id) {
      temp = "white";
    } else if (BLACK == cubeinfolist[0].color_id) {
      temp = "black";
    } else {
      temp = "red"; // limited by the vision node design, all the other colors
                    // are prepreseted by red for querying
    }
    ROS_INFO("clothcolor%s\n", temp.c_str());
    return temp;
  };

  // void queryCubeType(){return cubeinfo};

  // invoked to decide which bucket the cloth should be put in
  unsigned int queryBucketIdByType(const unsigned int type) {
    for (int i = 0; i < 3; i++) {
      if (type == buckettype[i]) {
        return i;
      }
      //
    }
  };

  void callback_cubelist(laundryman::CubeInfoList cube_visionlist) {
    ROS_INFO("Msg recv");
    for (int i = 0; i < cube_visionlist.data.size(); i++) {

      msgRecvFlag = true;
      cubeinfolist[i].color_id = cube_visionlist.data[i].color_id;

      // It is decided to handle all the other colors as RED, limited by the
      // vision node
      if ((WHITE != cube_visionlist.data[i].color_id) &&
          (BLACK != cube_visionlist.data[i].color_id)) {
        cubeinfolist[i].color_id = RED;
      }
      cubeinfolist[i].pose.position.x = cube_visionlist.data[i].pose.position.x;
      cubeinfolist[i].pose.position.y = cube_visionlist.data[i].pose.position.y;
      cubeinfolist[i].pose.position.z = cube_visionlist.data[i].pose.position.z;
      cubeinfolist[i].pose.orientation.x =
          cube_visionlist.data[i].pose.orientation.x;
      cubeinfolist[i].pose.orientation.y =
          cube_visionlist.data[i].pose.orientation.y;
      cubeinfolist[i].pose.orientation.z =
          cube_visionlist.data[i].pose.orientation.z;
      cubeinfolist[i].pose.orientation.w =
          cube_visionlist.data[i].pose.orientation.w;
    }
  };

  void init() {

    client_motion =

        nh.serviceClient<laundryman::MotionService>("motion_control");

    sub_cubeinfolist = nh.subscribe<laundryman::CubeInfoList>(
        "/vision_node/cube_info_list", 1, &LaundryMan::callback_cubelist, this);

    pl = PrologClient("/rosprolog", true);

    cubeinfolist[MAXMIUM_CUBE_NUMBER];

    msgRecvFlag = false;

    // initialize the mpas info, used when to place the cloth
    bucket_map.resize(4);

    bucket_map[0].bucket_id = 0;
    bucket_map[0].pose.position.x = 0.0;
    bucket_map[0].pose.position.y = 2.0;
    bucket_map[0].pose.position.z = 0.0;
    bucket_map[0].pose.orientation.x = 0.0;
    bucket_map[0].pose.orientation.y = 0.0;
    bucket_map[0].pose.orientation.z = 0.0;
    bucket_map[0].pose.orientation.w = 1.0;

    bucket_map[1].bucket_id = 1;
    bucket_map[1].pose.position.x = 0.0;
    bucket_map[1].pose.position.y = 0.0;
    bucket_map[1].pose.position.z = 0.0;
    bucket_map[1].pose.orientation.x = 0.0;
    bucket_map[1].pose.orientation.y = 0.0;
    bucket_map[1].pose.orientation.z = 0.0;
    bucket_map[1].pose.orientation.w = 1.0;

    bucket_map[2].bucket_id = 2;
    bucket_map[2].pose.position.x = 0.0;
    bucket_map[2].pose.position.y = -2.0;
    bucket_map[2].pose.position.z = 0.0;
    bucket_map[2].pose.orientation.x = 0.0;
    bucket_map[2].pose.orientation.y = 0.0;
    bucket_map[2].pose.orientation.z = 0.0;
    bucket_map[2].pose.orientation.w = 1.0;

    bucket_map[3].bucket_id = 3;
    bucket_map[3].pose.position.x = -1.0;
    bucket_map[3].pose.position.y = 0.0;
    bucket_map[3].pose.position.z = 0.0;
    bucket_map[3].pose.orientation.x = 0.0;
    bucket_map[3].pose.orientation.y = 0.0;
    bucket_map[3].pose.orientation.z = 1.0;
    bucket_map[3].pose.orientation.w = 0.0;

    ROS_INFO("Initialize");
  };
};

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "main_node");
  LaundryMan mylaundryman;

  mylaundryman.init();
  mylaundryman.execute();
  ros::spinOnce();

  return 0;
}