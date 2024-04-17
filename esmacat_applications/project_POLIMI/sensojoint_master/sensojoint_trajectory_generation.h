#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#endif // TRAJECTORY_GEN_H


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "error_list.h"
#include "sensojoint_structs.h"
#include "sensojoint_manager.h"
#include <unistd.h>
#include "file_handling/include/json_handling.h"
#include <fstream>
#include <vector>
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class trajectory_gen : public sensojoint_manager {
private:
//int n_points; // n of points of the vectors//
int rep_number;
int n_points;
std::vector<std::vector<float>> data_repetition;

public:

    trajectory_gen();
    void read_ad_file();
    void compute_average_vel();
    void discard_rep();

};

/* void read_ad_file() {
   ifstream inputFile;
  inputFile.open("Rep_data.csv",ios::in);
if (inputFile.is_open()) {

while (inputFile.good()) {
  std::string line;
  std::vector<std::vector<float>> a;
  int i=0;
  while(std::getline( inputFile,line)) {
  float value;
  std::stringstream ss(line);
  a.push_back(std::vector<float>());
  while (ss >> value) {
  a[i].push_back(value);
  i++;
  }
  }
  }

}
else {
    // show message:
    std::cout << "Error opening file";
  }
}
*/
 /* void main {


if {
   rep_counte

    traj
C
} */
//





    //

