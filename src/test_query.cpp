#include <iostream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;
#define WHITE 1
#define OTHER 2
#define BLACK 3

// function: get the bucket ID from the color of cloth inside
int query_color(PrologClient pl, std::string color) {

  pl.query("rdf_assert(ssy235Ontology:'temp',rdf:type,ssy235Ontology:" + color +
           ")");

  PrologQuery bdgs = pl.query(
      "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType1)");

  if (!(bdgs.begin() == bdgs.end())) {
    return WHITE;
  }
  bdgs = pl.query(
      "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType2)");

  if (!(bdgs.begin() == bdgs.end())) {
    return OTHER;
  }
  bdgs = pl.query(
      "owl_individual_of(ssy235Ontology:'temp',ssy235Ontology:bucketType3)");

  if (!(bdgs.begin() == bdgs.end())) {
    return BLACK;
  }
  return 0;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_ros_prolog");

  PrologClient pl = PrologClient("/rosprolog", true);

  // test with colore: blue
  std::string color_string = "blue";
  cout << query_color(pl, color_string) << endl;

  return 0;
}
