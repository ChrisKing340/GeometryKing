/*******************************************************************
 TextFileParse - tokenizes a text file and stores the tokens
*******************************************************************/
#pragma once

#include <string>
#include <sstream>// #include <strstream>
#include <fstream>
#include <iomanip>

#include <comutil.h>
#pragma comment(lib, "comsuppw.lib")

namespace King {

class TextFileParse 
{
	/* variables */
protected:
	std::wstring				nameOfFile;
	int							wordCount		= 0; // words[wordCount]
	int							currentWord		= 0; // index into words[]
	std::string					commentDes		= "//"; // igorne text on lines after this string found
	char						customSeparator = -1;
	std::string					groupStartDes	= "{";
	std::string					groupStopDes	= "}";
private:
	std::ostringstream			storage; // keeps the file contents in memory until WordParse() is performed which then clears it
	char**						words			= nullptr; // pointer array filled when WordParse() is complete
	int							findPosition	= 0; // word token instance to start searching from
	/* methods */
public:
	// life cycle
	TextFileParse() = default;
	TextFileParse(std::wstring fileName ) : TextFileParse() { Load( fileName ); }
	~TextFileParse() { if (words) delete [] words; storage.clear(); }
	// operators
	explicit operator bool() const { if (words == nullptr) return false; else return true; }
	bool operator !() const { return !words || !wordCount; }
	inline std::string operator[] (int indexIn) const { return Word(indexIn); } // accessor
	// functionality
	bool						Load(std::wstring fileName );

	int							Next() { if(currentWord<wordCount-1) currentWord++; return currentWord;}
	int							Back() { if(currentWord>0) currentWord--; return currentWord;}
	
	int							GoBegin() { currentWord=0; return currentWord;}
	int							GoEnd() { if(wordCount) currentWord=wordCount-1; return currentWord;}
	// trial
	bool						IsLast() const { if(currentWord >= wordCount-1) return true; return false;}
	// access or assign
	int							WordIndex(int indexSet = -1) { if (indexSet > -1) currentWord = indexSet; if (currentWord > wordCount - 1) currentWord= wordCount - 1; return currentWord; }
	// access
	const std::wstring &		GetFileName() const { return nameOfFile; }
	const int &					GetWordCount() const { return wordCount; }
	std::string 				Word(int index = -1) const { if (index < 0) index = currentWord; if (index > wordCount - 1) index = wordCount - 1; if (words) return words[index]; else return ""; }
	std::wstring 				WordW(int index = -1) { if (index < 0) index = currentWord; return HelperStringToWString(words[index]); }
	std::string 				NextWord() { if (currentWord < wordCount - 1) currentWord++; if (words) return words[currentWord]; else return ""; }
	std::wstring				NextWordW() { if (currentWord < wordCount - 1) currentWord++; if (words) return HelperStringToWString(words[currentWord]); else return L""; }
	// assign
	void						SetCommentDesignator(std::string comments) { commentDes = comments; }
	void						SetCustomSeparator(char s) { customSeparator = s; }
	void						SetCustomGroupStart(std::string in) { groupStartDes = in; }
	void						SetCustomGroupStop(std::string in) { groupStopDes = in; }
	// useful methods
	int							CountString(const std::string &txt);
	int							CountStringAndStopAtMarker(const std::string &txt, const std::string marker);

	bool						FindFirst(const std::string &txt);
	bool						FindNext(const std::string &txt);
	bool						FindNextFrom(const std::string &txt, const int wordPositionIN) { currentWord = wordPositionIN; return FindNext(txt); }
	
	std::wstring				HelperStringToWString(const std::string strIn) { _bstr_t	bstr1(strIn.c_str()); std::wstring wstr(bstr1); return wstr; }

protected:
    void						WordParse();
};

}