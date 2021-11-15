#include <iostream>

#include "parser/parser.hpp"

int main() {

  Parser parser("/home/yession/Code/Cpp/ycr/scene.yaml");
  parser.parse();

  return 0;
}