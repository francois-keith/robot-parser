// simple tools for parsing.

#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

std::string tolower(const std::string & word);
std::string toupper(const std::string & word);

//// File manipulation

// open an ofstream (verbose)
#define DECLARE_OFSTREAM(out, name)						\
std::ofstream out (name);								\
if (!out)												\
	std::cerr << "  Cannot open " << name << std::endl;	\
else													\
	std::cout << "  Creating  " << name << std::endl;	\
out.precision(15);


// open an ifstream (verbose)
#define DECLARE_IFSTREAM(in, name)						\
std::ifstream in (name);								\
if (!in)												\
	std::cerr << "  Cannot open " << name << std::endl;

void copyFile(const std::string & infile, const std::string & outfile);
void copyFile (const std::string & infile, std::ofstream & out);



//// Parsing

//get the element of type T between the two tags start and finish
template<class T>
T parseTableElement(const std::string & line, T defaultValue,
	const std::string & start="<td>", const std::string & finish="</td>")
{
	int found0=line.find(start);
	int found1=line.find(finish);

	std::string tmp = line.substr(found0+start.size(),found1-found0-start.size());
	std::istringstream smallData(tmp, std::ios_base::in);

	T elemt (defaultValue);
	smallData >> elemt;
	return elemt;
}

//Parse the line to get two strings.
// The line has thus the form
// start  innerBodyId   middle  outerBodyId  finish
void parseTableElement2(const std::string & line,
	std::string &innerBodyId, std::string & outerBodyId,
	std::string start, std::string middle, std::string finish);

//Parse the line to get two double.
// The line has thus the form
// start  innerBodyId   middle  outerBodyId  finish
void parseTableElement2_double(const std::string & line,
	double &innerBodyId, double & outerBodyId,
	std::string start, std::string middle, std::string finish);

#endif // TOOLS_H

