
#include "error_list.h"
#include "sensojoint_structs.h"
#include "sensojoint_manager.h"
#include "sensojoint_trajectory_generation.h"
#include <unistd.h>
#include "file_handling/include/json_handling.h"
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
using namespace std;

/* int main() {
     {
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
};
*/
 void trajectory_gen::read_ad_file() {
  ifstream inputFile;
  std::vector<std::vector<float>> data_array;
  int n_data_to_read=8;;
 inputFile.open("Rep_data.csv",ios::in);
if (inputFile.is_open())  {
 while (inputFile.good()) {
  std::string line;
  std::string value;
  int i=0;
  int c=0;
  while(std::getline( inputFile,line)) {
   std::stringstream s (line);
   std::vector<float> row;
    while (std::getline(s,value,',')) {
    row.push_back(std::stof(value));
    if (i==n_data_to_read-1) {
     data_array.push_back(row);
     }
    i++;
    }
    c=row.size();
    if (c!=n_data_to_read) {
     PLOGI <<"Error Missing Data" ;
     }
   }
   n_points=data_array.size();
   data_repetition=data_array;
  }

}
else {
   // show message:
   std::cout << "Error opening file";
  }

}

 void trajectory_gen::discard_rep() {

    for (int j=0; j<data_repetition.size();j++) {

     std:vector<float> rep_good_list;
     bool good= true;
     if ( (static_cast <int>(data_repetition[j][2]) == 1) && (good==true)) {
        good=false;
         }
         if (good==true &&(static_cast <int>(data_repetition[j][1]) !=static_cast <int>(data_repetition[j+1][1])))  {
         rep_good_list.push_back(static_cast <int>(data_repetition[j][1]));

           }
         else if (good==false &&(static_cast <int>(data_repetition[j][1]) !=static_cast <int>(data_repetition[j+1][1]))) {
         good=false;

          }

      j++;
      std::vector<int>::iterator it;
//    it = find(data_repetition.begin(), data_repetition.end(), j+1);
//     it = find(data_repetition.begin(),data_repetition.end(),j+1);


      }

   }






