// simple tools for parsing.
#include "tools.h"

std::string tolower(const std::string & word)
{
	std::string word2(word);
	for (unsigned i=0; i<word2.size();++i)
		word2[i]=tolower(word[i]);
	return word2;
}

std::string toupper(const std::string & word)
{
	std::string word2(word);
	for (unsigned i=0; i<word2.size();++i)
		word2[i]=toupper(word[i]);
	return word2;
}

void copyFile(const std::string & infile, const std::string & outfile)
{
	DECLARE_IFSTREAM(in, infile.c_str());
	DECLARE_OFSTREAM(out, outfile.c_str());

	std::string line;
	while(getline(in, line))
		out << line << std::endl;

	in.close();
	out.close();
}

void copyFile (const std::string & infile, std::ofstream & out)
{
	// open the file.
	DECLARE_IFSTREAM(in, infile.c_str());

	// copy the file.
	std::string line;
	while (std::getline(in, line))
		out << line<< std::endl;
	in.close();
}




//Parse the line to get two strings.
// The line has thus the form
// start  innerBodyId   middle  outerBodyId  finish
void parseTableElement2(const std::string & line,
	std::string &innerBodyId, std::string & outerBodyId,
	std::string start, std::string middle, std::string finish)
{
	int found0=line.find(start);
	int found1=line.find(middle);
	int found2=line.find(finish);
	innerBodyId = line.substr(found0+start.size(),found1-found0-start.size());
	outerBodyId = line.substr(found1+middle.size(),found2-found1-middle.size());
}

//Parse the line to get two double.
// The line has thus the form
// start  innerBodyId   middle  outerBodyId  finish
void parseTableElement2_double(const std::string & line,
	double &innerBodyId, double & outerBodyId,
	std::string start, std::string middle, std::string finish)
{
	int found0=line.find(start);
	int found1=line.find(middle);
	int found2=line.find(finish);

	std::string tmp1 = line.substr(found0+start.size(),found1-found0-start.size());
	std::istringstream smallData1(tmp1, std::ios_base::in);
	smallData1 >> innerBodyId;

	std::string tmp2 = line.substr(found1+middle.size(),found2-found1-middle.size());
	std::istringstream smallData2(tmp2, std::ios_base::in);
	smallData2 >> outerBodyId;
}

