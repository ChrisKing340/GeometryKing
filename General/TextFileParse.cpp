#include "TextFileParse.h"

using namespace std;

/*******************************************************************
* Load the file into a string stream while ignoring comments
*******************************************************************/
bool King::TextFileParse::Load(string fileName)
{
    string   inBuffer;
    ifstream dataFile; // file input stream

    nameOfFile = fileName;

    dataFile.open( fileName );
    if( ! dataFile ) 
    {
        string str = "\nFile " + fileName + " could not be opened.";
        return false;
    }

    wordCount = 0;
    // read contents of file into storage array and remove all comments
    while( storage && dataFile >> setw(4096) >> inBuffer )
    {
        // test for comment line
        if( inBuffer == commentDes.c_str() )
        {
            // found comment line so skip rest of words in line
            dataFile.ignore( 4096, '\n' );
        }
        else
        {
            // add word to storage array;
            ++wordCount;
            storage << inBuffer << ends;
        }
    }
    dataFile.close();
    if( ! wordCount )
    {
        string str = fileName + " is empty.";
        return false;
    }
    WordParse();

    return true;
}
/*******************************************************************
* Parse the string stream into tokens (words) and create the
*  indexing array of words
*******************************************************************/
void King::TextFileParse::WordParse()
{
    auto str = storage.str();
    char* psource = str.data();
    std::streamsize characters;
    characters = str.size();
    textBuffer = make_unique<char[]>(characters);
    auto ptextBuffer = textBuffer.get();
    memcpy(ptextBuffer, psource, characters);

    // parse out separators into separate lines
    for(int i=0;i<characters;i++)
    {
        if( ptextBuffer[i] == ',' || ptextBuffer[i] == '\t' || ptextBuffer[i] == ' ' || ptextBuffer[i] == '\n' || ptextBuffer[i] == customSeparator )
            ptextBuffer[i] = '\0';
    }

    // count the number of lines in the string which are now words since separators are removed
    wordCount = 0;
    for(int i=0;i<characters;i++)
        if( ptextBuffer[i] == '\0' )
            wordCount++;

    // create an array of pointers to the inputTextStream buffer for each parsed word
    if( words ) delete words;
    words = new char *[wordCount];
    memset(words, 0, wordCount);
    // fill the words pointer array and remove empty words
    int wordsRemoved = 0;
    for(int i=0;i<wordCount;i++)
    {
        if( strlen(ptextBuffer) == 0 ) // length to '\0'
        {
            wordsRemoved++;
        }
        else words[i-wordsRemoved] = ptextBuffer; // store the pointer to the word

        ptextBuffer += strlen(ptextBuffer)+1;
    }
    wordCount -= wordsRemoved;

    storage.str(""); // remove the contents
}
/*******************************************************************
* return the word at index location
*******************************************************************/
inline std::string King::TextFileParse::Word(int index) const
{
    if (index < 0) index = currentWord;
    else if (index > wordCount - 1) index = wordCount - 1;
    if (words)
        return words[index];
    else
        return "";
}
/*******************************************************************
* count the total number of words that are equal to the input txt
*  within the file
*******************************************************************/
int King::TextFileParse::CountString( const string &txt )
{
    int count = 0;
    int currentWordHolder = currentWord; // save the current position

    while( !IsLast() )
    {
        auto w = Word();
        if( w == txt ) count++;
        Next();
    }
    currentWord = currentWordHolder; // restore the position
    return count;
}
/*******************************************************************
* count the total number of words that are equal to the input txt
*  within the file but stop when the marker text is reached
*******************************************************************/
int King::TextFileParse::CountStringAndStopAtMarker(const string & txt, const string marker)
{
    int count = 0;
    int currentWordHolder = currentWord; // save the current position

    string w = Word();
    do 
    {
        if (w == txt) count++;
    } 
    while (!IsLast() && (w = NextWord()) != marker);
    
    currentWord = currentWordHolder; // restore the position
    return count;
}

/*******************************************************************
* Find the first instance of the word token
*******************************************************************/
bool King::TextFileParse::FindFirst(const string &txt)
{
    int temp = currentWord;
    GoBegin();
    while (!IsLast() && (Word() != txt))
        Next();
    if (Word() == txt) findPosition = currentWord; // first token of file
    else { findPosition = temp; currentWord = temp; return false; }// not found
    return true;
}
/*******************************************************************
* Find the next instance of the word token
*  Search only in the current group and any sub groupings using
*  {} to define groups.  If an } is encountered before its { pair
*  the routine stops and exits
*******************************************************************/
bool King::TextFileParse::FindNext(const string &txt, bool onlyWithInGroupBracketLimiters)
{
    int temp = currentWord;
    long countOpenBrackets = 0;

    if (onlyWithInGroupBracketLimiters)
    {
        // end of a group, skip and start next
        if (NextWord() == groupStopDes) ++currentWord;
        else --currentWord;
    
        while (!IsLast() && (Word() != txt))
        {
            if (Word() == groupStartDes) ++countOpenBrackets;
            else if (Word() == groupStopDes) --countOpenBrackets;

            if (countOpenBrackets < 0) break;
            Next();
        }
    }
    else
    {
        while (!IsLast() && (Word() != txt))
        {
            Next();
        }
    }

    if (Word() == txt)
    {
        findPosition = currentWord;
    }
    else 
    { 
        findPosition = currentWord = temp; 
        return false; 
    }
    return true;
}