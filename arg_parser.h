#ifndef ARG_PARSER_H
#define ARG_PARSER_H

#include <string>
#include <vector>
#include <map>

// Struct to hold command line argument information
struct Argument {
    std::string name;
    std::string shortName;
    std::string defaultValue;
    std::string description;
};

// Vector of all valid arguments
extern std::vector<Argument> arguments;

// Function to parse command line arguments into a map
std::map<std::string, std::string> parseArgs(int argc, char* argv[]);

// Function to generate help text
std::string generateHelpText();

#endif // ARG_PARSER_H
