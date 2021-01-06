#include <string>
#include <iostream>
 
#include <rosprolog/rosprolog_client/PrologClient.h>
 
using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_ros_prolog");
 
  PrologClient pl = PrologClient("/rosprolog",true);
 
  PrologQuery bdgs = pl.query("member(A, [1, 2, 3, 4]), B = ['x', A], C = foo(bar, A, B)");
 
  for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    cout << "Found solution: " << (bool)(it == bdgs.end()) << endl;
    cout << "A = "<< bdg["A"] << endl;
    cout << "B = "<< bdg["B"] << endl;
    cout << "C = "<< bdg["C"] << endl;
  }
  return 0;
}
