/*******************************************************************
 TextFileParse - tokenizes a text file and stores the tokens
    (c) Christopher H. King 1997

    Update 2020 - removed #include <strstream> now depreciated
        with #include <sstream> in move to c++17.  This changed
        how it gives internal memory versus a copy.  So changed
        WordParse() as well. Also removed fileName as wstring
        since I normally only ever use string and always hack
        in a conversion to wstring to call TexFilePase.  Those hacks
        now can be removed (breaking change).
*******************************************************************/
#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <comutil.h>
#pragma comment(lib, "comsuppw.lib")

namespace King {

class TextFileParse 
{
	/* variables */
protected:
	std::string				    nameOfFile;
	int							wordCount		= 0; // words[wordCount]
	int							currentWord		= 0; // index into words[]
	std::string					commentDes		= "//"; // igorne text on lines after this string found
	char						customSeparator = -1;
	std::string					groupStartDes	= "{";
	std::string					groupStopDes	= "}";
private:
	std::ostringstream			storage;// just for loading, discarded in WordParse()
    std::unique_ptr<char[]>     textBuffer;
	char**						words			= nullptr; // pointer array into ptextBuffer for each word
	int							findPosition	= 0; // word token instance to start searching from
	/* methods */
public:
	// life cycle
	TextFileParse() = default;
	TextFileParse(std::string fileName ) : TextFileParse() { Load( fileName ); }
	~TextFileParse() { if (words) delete [] words; }
	// operators
	explicit operator bool() const { if (words == nullptr) return false; else return true; }
	bool operator !() const { return !words || !wordCount; }
	inline std::string operator[] (int indexIn) const { return Word(indexIn); } // accessor
	// functionality
	bool						Load(std::string fileName );

	int							Next() { if(currentWord<wordCount-1) currentWord++; return currentWord;}
	int							Back() { if(currentWord>0) currentWord--; return currentWord;}
	
	int							GoBegin() { currentWord=0; return currentWord;}
	int							GoEnd() { if(wordCount) currentWord=wordCount-1; return currentWord;}

    std::string 				Word(int index = -1) const;
    std::wstring 				WordW(int index = -1) { if (index < 0) index = currentWord; return HelperStringToWString(words[index]); }

    std::string 				NextWord() { if (words) return words[Next()]; else return ""; }
    std::wstring				NextWordW() { if (words) return HelperStringToWString(words[Next()]); else return L""; }

    std::string 				PreviousWord() { if (words) return words[Back()]; else return ""; }
    std::wstring				PreviousWordW() { if (words) return HelperStringToWString(words[Next()]); else return L""; }
	// trial
    bool						IsFirst() const { if (!currentWord) return true; return false; }
	bool						IsLast() const { if(currentWord >= wordCount-1) return true; return false;}
	// access or assign
	int							WordIndex(int indexSet = -1) { if (indexSet > -1) currentWord = indexSet; if (currentWord > wordCount - 1) currentWord= wordCount - 1; return currentWord; }
	// access
	const std::string &		    GetFileName() const { return nameOfFile; }
	const int &					GetWordCount() const { return wordCount; }
    // assign
	void						SetCommentDesignator(std::string comments) { commentDes = comments; }
	void						SetCustomSeparator(char s) { customSeparator = s; }
	void						SetCustomGroupStart(std::string in) { groupStartDes = in; }
	void						SetCustomGroupStop(std::string in) { groupStopDes = in; }
	// useful methods
	int							CountString(const std::string &txt);
	int							CountStringAndStopAtMarker(const std::string &txt, const std::string marker);

	bool						FindFirst(const std::string &txt);
	bool						FindNext(const std::string &txt, bool onlyWithInGroupBracketLimiters = true);
	bool						FindNextFrom(const std::string &txt, const int wordPositionIN) { currentWord = wordPositionIN; return FindNext(txt); }

    bool						FindPrevious(const std::string& txt, bool onlyWithInGroupBracketLimiters = true);
    bool						FindPreviousFrom(const std::string& txt, const int wordPositionIN) { currentWord = wordPositionIN; return FindNext(txt); }

protected:
    void						WordParse();
    std::wstring				HelperStringToWString(const std::string strIn) { _bstr_t bstr1(strIn.c_str()); std::wstring wstr(bstr1); return wstr; }
};

}