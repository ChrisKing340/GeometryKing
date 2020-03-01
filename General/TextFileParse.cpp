#include "TextFileParse.h"

using namespace std;

/*******************************************************************
* Load the file into a string stream while ignoring comments
*******************************************************************/
bool King::TextFileParse::Load(wstring fileName)
{
    string   inBuffer;
    ifstream dataFile; // file input stream

    nameOfFile = fileName;

    dataFile.open( fileName );
    if( ! dataFile ) 
    {
        wstring str = L"\nFile " + fileName + L" could not be opened.";
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
        wstring str = fileName + L" is empty.";
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
    string str = storage.str();
    char *textBuffer = str.data();

    int i;
    std::streamsize characters;
    characters = str.size();

    // parse out separators into separate lines
    for(i=0;i<characters;i++)
    {
        if( textBuffer[i] == ',' || textBuffer[i] == '\t' || textBuffer[i] == ' ' || textBuffer[i] == '\n' || textBuffer[i] == customSeparator )
            textBuffer[i] = '\0';
    }

    // count the number of lines in the string which are now words since separators are removed
    wordCount = 0;
    for(i=0;i<characters;i++)
        if( textBuffer[i] == '\0' )
            wordCount++;

    // create an array of pointers to the inputTextStream buffer for each parsed word
    if( words ) delete words;
    words = new char *[wordCount];
    // fill the words pointer array
    int wordsRemoved = 0;
    for(int i=0;i<wordCount;i++)
    {
        if( strlen(textBuffer) == 0 )
        {
            wordsRemoved++;
        }
        else words[i-wordsRemoved] = textBuffer;

        textBuffer += strlen(textBuffer)+1;
    }
    wordCount -= wordsRemoved;

    storage.clear();
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
        if( Word() == txt ) count++;
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
*  Search only in the current group and any sub groupings
*******************************************************************/
bool King::TextFileParse::FindNext(const string &txt)
{
    int temp = currentWord;
    long countOpenBrackets = 0;

    // end of a group, skip and start next
    if (NextWord() == groupStopDes) ++currentWord;
    else --currentWord;

    while(!IsLast() && (Word() != txt))
    {
        if (Word() == groupStartDes) ++countOpenBrackets;
        else if (Word() == groupStopDes) --countOpenBrackets;

        if (countOpenBrackets < 0) break;
        Next();
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