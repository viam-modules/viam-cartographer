//if include <json/value.h> line fails (latest kernels), try also:
//#include <jsoncpp/json/json.h>
//#include <json/value.h>
#include <iostream>
#include <fstream>
#include <regex>

using namespace std;

int main() {
   ifstream in;    // Create an input file stream.
  in.open("j.json");
  if ( ! in ) {
     cout << "Error: Can't open the file named data.txt.\n";
     exit(1);
  }

  string input;
  getline(in,input);  // Get the frist line from the file, if any.

  std::regex regex("\"[A-Za-z]+\":(\\d+\\.?\\d*)");
  std::smatch match;

  while (std::regex_search(input, match, regex)) {
      std::cout << match.str(1) << std::endl;
      input = match.suffix();
  }
}
