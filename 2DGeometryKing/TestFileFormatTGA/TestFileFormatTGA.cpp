#include <iostream>
#include <string>
#include <vector>
#include "..\2DGeometry.h"

class InputParser 
{
    // Reference:
    // https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
private:
    std::vector <std::string> tokens;
public:
    InputParser(int& argc, char** argv) 
    {
        for (int i = 1; i < argc; ++i)
            this->tokens.push_back(std::string(argv[i]));
    }

    const std::string GetCmdOption(const std::string& option) const 
    {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end())
            return *itr;
        static const std::string empty_string("");
        return empty_string;
    }

    bool CmdOptionExists(const std::string& option) const 
    {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
            != this->tokens.end();
    }
};

using namespace King;
using namespace std;

int main(int argc, char** argv) 
{
    cout << "GeometryKing TGA file reader + writer TEST application\n\n";

    InputParser input(argc, argv);

    const string filename = input.GetCmdOption("-if");
    string ext;
    if (!filename.empty())
    {
        if ((ext = filename.substr(filename.find_last_of(".") + 1)) == "tga")
        {
            ImageTGA tga;
            std::ifstream infile(filename, std::ifstream::binary);

            if (tga.ReadTGA(infile))
            {
                cout << "  Successful file load.\n";

                {
                    if (tga.header.datatypecode == 2)
                        cout << "  Uncompressed " << (int)tga.header.bitsperpixel << " bit image file." << '\n';
                    else if (tga.header.datatypecode == 10)
                        cout << "  RLE compressed " << (int)tga.header.bitsperpixel << " bit image file." << '\n';
                    else
                        cout << "  Unsupported data format (type " << tga.header.datatypecode << ")" << '\n';

                    cout << "\n  Header Contents:\n";
                    cout << "    idlength " << (int)tga.header.idlength << '\n';
                    cout << "    colormaptype " << (int)tga.header.colormaptype << '\n';
                    cout << "    datatypecode " << (int)tga.header.datatypecode << '\n';
                    cout << "    colormaporigin " << (int)tga.header.colormaporigin << '\n';
                    cout << "    colormaplength " << (int)tga.header.colormaplength << '\n';
                    cout << "    colormapdepth " << (int)tga.header.colormapdepth << '\n';
                    cout << "    x_origin " << (int)tga.header.x_origin << '\n';
                    cout << "    y_origin " << (int)tga.header.y_origin << '\n';
                    cout << "    width " << (int)tga.header.width << '\n';
                    cout << "    height " << (int)tga.header.height << '\n';
                    cout << "    bitsperpixel " << (int)tga.header.bitsperpixel << '\n';
                    cout << "    imagedescriptor " << (int)tga.header.imagedescriptor << '\n';
                    cout << "    IdentificationFieldString " << tga.IdentificationFieldString.c_str() << '\n';
                }

                // write output parameters
                bool rle = input.CmdOptionExists("-rle") || input.CmdOptionExists("-RLE");
                if (rle)
                    tga.header.datatypecode = 10;
                else
                    tga.header.datatypecode = 2;

                string out = input.GetCmdOption("-of");
                if (out != "")
                    out = "out" + filename;

                // write file output
                std::ofstream outfile("testout.tga", std::ifstream::binary);
                tga.WriteTGA(outfile);

                cout << "\n  WROTE testout.tga with:\n";
                {
                    if (tga.header.datatypecode == 2)
                        cout << "  Uncompressed " << (int)tga.header.bitsperpixel << " bit image file." << '\n';
                    else if (tga.header.datatypecode == 10)
                        cout << "  RLE compressed " << (int)tga.header.bitsperpixel << " bit image file." << '\n';
                    else
                        cout << "  Unsupported data format (type " << tga.header.datatypecode << ")" << '\n';

                    cout << "\n  Header Contents:\n";
                    cout << "    idlength " << (int)tga.header.idlength << '\n';
                    cout << "    colormaptype " << (int)tga.header.colormaptype << '\n';
                    cout << "    datatypecode " << (int)tga.header.datatypecode << '\n';
                    cout << "    colormaporigin " << (int)tga.header.colormaporigin << '\n';
                    cout << "    colormaplength " << (int)tga.header.colormaplength << '\n';
                    cout << "    colormapdepth " << (int)tga.header.colormapdepth << '\n';
                    cout << "    x_origin " << (int)tga.header.x_origin << '\n';
                    cout << "    y_origin " << (int)tga.header.y_origin << '\n';
                    cout << "    width " << (int)tga.header.width << '\n';
                    cout << "    height " << (int)tga.header.height << '\n';
                    cout << "    bitsperpixel " << (int)tga.header.bitsperpixel << '\n';
                    cout << "    imagedescriptor " << (int)tga.header.imagedescriptor << '\n';
                    cout << "    IdentificationFieldString " << tga.IdentificationFieldString.c_str() << '\n';
                }
            }
            else
                cout << "  Error - File loading could not be completed.\n";
        }
        else
            cout << "  File name provided did not have a .tga extension (" << ext << ")\n";
    }
    else
        cout << "  Specify a .tga file to load with: -if filename.tga\n  Specify the out file name with -of outfilename.tga\n  Specify writing with run length compression with -rle\n";
    
    if (true)
    {
        // Drawing tests
        // 
        // Start with an ImageBlock (our MemoryBlock class inherited with some extra functionality)
        // File encoders/decoders inherit ImageBlock to override Read/Write methods and keep their extra header information
        ImageTGA im(512, 512 / 3, 4);

        auto white = float4(1.f, 1.f, 1.f, 1.f);
        auto black = float4(0.f, 0.f, 0.f, 1.f);
        auto red = float4(1.f, 0.f, 0.f, 1.f);
        auto green = float4(0.f, 1.f, 0.f, 1.f);
        auto blue = float4(0.f, 0.f, 1.f, 1.f);
        auto yellow = float4(1.f, 0.9f, 0.50f, 1.f);
        auto ltblue = float4(0.67f, 0.85f, 0.90f, 1.f);

        // white and opaque using MemoryBlock Fill of every byte
        im.Fill(255);
        // trianlges use initializer constructors for float2 groupings to define 3 points
        Triangle2DF t = { { 512.f / 3.f - 10.f - (512.f / 3.f - 20.f) * 0.866f, 512.f / 6.f }, { 512.f / 3.f - 10.f, 10.f}, {  512.f / 3.f - 10.f, 512.f / 3.f - 10.f} };
        im.DrawFilled(t, red);
        // rectangle use initialize constructors for float3 groupings of LT and RB points
        Rectangle2DF r = { { 512.f * 2.0f / 3.f + 10.f, 10.f }, { 512.f - 10.f, 512.f / 3.f - 10.f} };
        im.DrawFilled(r, green);
        // use default constructor for center and radius
        Circle2DF cr(float2(512.f / 2.f, 512.f / 6.f), 512.f / 6.f - 10.f);
        im.DrawFilled(cr, blue);

        // default is an uncompressed truevision file that is portable to about every paint program in existence
        std::ofstream outfile("testdraw.tga", std::ifstream::binary);
        im.WriteTGA(outfile);

        cout << "\n  WROTE testdraw.tga\n";
        cout << "    _w " << im.GetWidth() << '\n';
        cout << "    _h " << im.GetHeight() << '\n';
        cout << "    _stride " << im.GetStride() << '\n';
        cout << "    _length " << im.GetLength() << '\n';
    }

    return 0;
}
