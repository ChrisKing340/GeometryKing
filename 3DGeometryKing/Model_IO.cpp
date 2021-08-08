#include "Model_IO.h"

using namespace std;
using namespace King;

/******************************************************************************
*    Class:    Model_IO
*    Method:   Load_KNG
*    Description:    Imports King::Model definitions from native format
*
*    Inputs:   fileNameIN        name to give file
*    Outputs:  reads contents of meshes for each model from file
*    Returns:  std::vector       array of models
******************************************************************************/
std::vector<shared_ptr<King::ModelScaffold>> King::Model_IO::Load_KNG(const std::string fileNameIN)
{
    bool good = true;
    std::vector<shared_ptr<King::ModelScaffold>> models;

    // v1 for only a King::ModelScaffold
    struct KNG_v1 
    {
        uint32_t    version = 1;
        uint32_t    numModels = 1;
    } header;

    ifstream dataFile;
    dataFile.open(fileNameIN, std::ifstream::in | std::ifstream::binary);

    if (!dataFile)
    {
        string str = "\nFile " + fileNameIN + " could not be opened.";
        cout << str << '\n';
        return models;
    }

    dataFile.read(reinterpret_cast<char*>(&header), sizeof(header));

    while (dataFile && good && header.numModels)
    {
        auto model = ModelScaffold::Create();

        good = model->Read_v1(dataFile);
        models.push_back(model);

        --header.numModels;
    }
    if (!dataFile)
    {
        string str = "\nFile " + fileNameIN + " error occurred during read.";
        cout << str << '\n';
        return models;
    }

    dataFile.close();

    return models;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:   Save_KNG
*    Description:    Exports King::Model definitions from native format
*
*    Inputs:   fileNameIN        name to give file
*    Outputs:  reads contents of meshes for each model from file
*    Returns:  std::vector       array of models
******************************************************************************/
bool King::Model_IO::Save_KNG(const std::string fileNameIN, const std::vector<shared_ptr<King::ModelScaffold>>& modelsIN)
{
    bool good = true;

    ofstream dataFile;
    dataFile.open(fileNameIN, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);

    if (!dataFile)
    {
        string str = "\nFile " + fileNameIN + " could not be opened.";
        cout << str << '\n';
        return false;
    }

    // v1 for only a King::ModelScaffold
    struct KNG_v1
    {
        size_t    version = 1;
        size_t    numModels;
    } header;
    header.numModels = modelsIN.size();

    dataFile.write(reinterpret_cast<char*>(&header), sizeof(header));

    for (const auto m : modelsIN)
    {
        if (dataFile && good && header.numModels)
        {
            good = m->Write_v1(dataFile);

            --header.numModels;
        }
        if (!good) break;
    }
    if (!dataFile || !good)
    {
        string str = "\nFile " + fileNameIN + " error occurred during write.";
        cout << str << '\n';
        return false;
    }

    dataFile.close();

    return true;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:   Load_OBJ
*    Description:    Imports King::Model definitions from .obj format
*
*    Inputs:   fileNameIN        name to give file
*    Outputs:  reads contents of meshes for each model from file
*    Returns:  std::vector       array of models
******************************************************************************/
std::vector<shared_ptr<King::Model>> King::Model_IO::Load_OBJ(const std::string fileNameIN, const VertexFormat *vertexFormatIn)
{
    std::vector<shared_ptr<King::Model>> models;

    King::TextFileParse f;
    f.SetCommentDesignator("#");

    if (!f.Load(fileNameIN)) return models;

    // get number of models in file
    int numModels = 1;
    bool modelNameDefined = false;
    if (!flagGroupsAsObjects)
    {
        numModels = f.CountString(obj.prop_modelName);
        if(numModels) modelNameDefined = f.FindFirst(obj.prop_modelName);
    }
    else
    {
        numModels = f.CountString(obj.prop_meshName);
        if (numModels) modelNameDefined = f.FindFirst(obj.prop_meshName);
    }
    // no identifiers found
    if (!numModels)
    {
        numModels = 1;
        modelNameDefined = false;
    }

    // build master index and vertex data
    class vertexComposite
    {
    public:
        float xyz[3];
        float uv[2];
        float norm[3];

        vertexComposite() = default;
        vertexComposite(float x, float y, float z, float u, float v, float nx, float ny, float nz)
        {
            xyz[0] = x;
            xyz[1] = y;
            xyz[2] = z;
            uv[0] = u;
            uv[1] = v;
            norm[0] = nx;
            norm[1] = ny;
            norm[2] = nz;
        }
        // implicit convert to float*
        inline operator float* () { return reinterpret_cast<float*>(this); }
    };

    class vertexCompositePadTo56 : public vertexComposite
    {
    public:
        float t[3] = { 0.f,0.f,0.f };
        float bt[3] = { 0.f,0.f,0.f };
        vertexCompositePadTo56() = default;
        vertexCompositePadTo56(const vertexComposite& in)
        {
            xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
            uv[0] = in.uv[0]; uv[1] = in.uv[1];
            norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];
        }
        // implicit convert to float*
        inline operator float* () { return reinterpret_cast<float*>(this); }
    };
    //..............................................................................
    // Model loading (several per file separated by "o modelname"
    // vertex data is zero based for each model, so read each type from "o " to "o "
    // or end of file
    //..............................................................................
    for(int modelCount=0; modelCount < numModels; ++modelCount)
    {
        auto wpos_models = f.WordIndex() + 1; // word position after find then skip the found word
        
        models.push_back(Model::Create());
        auto model = models.back();

        if(modelNameDefined) model->SetModelName(f.NextWord());
        if (vertexFormatIn) model->SetVertexFormat(*vertexFormatIn);

        // attempt to fix format if missing type needed
        auto vertexFormat = model->GetVertexFormat();
        if (!vertexFormat.Has(King::VertexAttrib::enumDesc::position))
            model->AddVertexAttribute(King::VertexAttrib::enumDesc::position, King::VertexAttrib::enumFormat::format_float32x3);
        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::textureCoord))
            model->AddVertexAttribute(King::VertexAttrib::enumDesc::textureCoord, King::VertexAttrib::enumFormat::format_float32x2);
        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::normal))
            model->AddVertexAttribute(King::VertexAttrib::enumDesc::normal, King::VertexAttrib::enumFormat::format_float32x3);

        // must be in the order we use in the loader below for memcpy to work correctly
        VertexAttrib attr[3];
        attr[0] = model->GetVertexFormat().GetAttribute(0);
        attr[1] = model->GetVertexFormat().GetAttribute(1);
        attr[2] = model->GetVertexFormat().GetAttribute(2);
        assert(attr[0].GetDescription() == VertexAttrib::enumDesc::position &&
            attr[1].GetDescription() == VertexAttrib::enumDesc::textureCoord &&
            attr[2].GetDescription() == VertexAttrib::enumDesc::normal);

        // Load the Master Object data

        // Get verticies
        auto positions = vector<vector<float>>();
        int numVertex;
        f.WordIndex(wpos_models); // return to start of model

        numVertex = f.CountStringAndStopAtMarker(obj.prop_vertexPosition, obj.prop_modelName);
        if (numVertex)
        {
            f.FindNext(obj.prop_vertexPosition);
            do
            {
                if (f.Word() == obj.prop_vertexPosition)
                {
                    vector<float> v(3);
                    v[0] = (float)atof(f.NextWord().c_str());
                    v[1] = (float)atof(f.NextWord().c_str());
                    v[2] = (float)atof(f.NextWord().c_str());

                    positions.push_back(v);
                }
                // if vertex are not grouped together, seek out the next one
                while (f.NextWord() != obj.prop_vertexPosition && !f.IsLast()) {};
            } while (!f.IsLast());
            cout << "  numVertex " << numVertex << "\n";
            //assert(numVertex == positions.size());
        }
        // Get texture UV
        auto textureUV = vector<vector<float>>();
        int numTextureCoords;
        f.WordIndex(wpos_models); // return to start of model

        bool hasModelUV;
        numTextureCoords = f.CountStringAndStopAtMarker(obj.prop_vertexTextureCoord, obj.prop_modelName);
        if (numTextureCoords)
        {
            hasModelUV = true;
            bool search = true;
            f.FindNext(obj.prop_vertexTextureCoord);
            do
            {
                if (f.Word() == obj.prop_vertexTextureCoord)
                {
                    vector<float> uv(2);
                    uv[0] = (float)atof(f.NextWord().c_str());
                    uv[1] = -1.f * (float)atof(f.NextWord().c_str());
                    // Since we negate our y uv coordinate on loads, when we save it we will negate it back (refer to Save_OBJ)

                    textureUV.push_back(uv);
                }
                // if vertexTextureCoord are not grouped together, seek out the next one
                while (f.NextWord() != obj.prop_vertexTextureCoord && !f.IsLast()) {};
            } while (!f.IsLast());
            cout << "  numTextureCoords " << numTextureCoords << "\n";
            //assert(numTextureCoords == textureUV.size());
        }
        else
        {
            hasModelUV = false;
        }
        // Get normals
        auto normals = vector<vector<float>>();
        int numNormals;
        f.WordIndex(wpos_models); // return to start of model

        bool hasModelNormals;
        numNormals = f.CountStringAndStopAtMarker(obj.prop_vertexNormal, obj.prop_modelName);
        if (numNormals)
        {
            hasModelNormals = true;
            bool search = true;
            f.FindNext(obj.prop_vertexNormal);
            do
            {
                if (f.Word() == obj.prop_vertexNormal)
                {
                    vector<float> vn(3);
                    vn[0] = (float)atof(f.NextWord().c_str());
                    vn[1] = (float)atof(f.NextWord().c_str());
                    vn[2] = (float)atof(f.NextWord().c_str());

                    normals.push_back(vn);
                }
                // if normals are not grouped together, seek out the next one
                while (f.NextWord() != obj.prop_vertexNormal && !f.IsLast()) {};
            } while (!f.IsLast());
            cout << "  numNormals " << numNormals << "\n";
            //assert(numNormals == normals.size());
        }
        else
            hasModelNormals = false; // this will flag the composite vertex assembler code later on to generate normals for each vertex

        // Get Faces
        vector<uint32_t> indicies;
        vector<vertexComposite> verticiesComposite;

        size_t numFaces;
        f.WordIndex(wpos_models); // reset position back to start of model
        if (numFaces = f.CountStringAndStopAtMarker(obj.prop_face, obj.prop_modelName))
        {
            cout << "  numFaces " << numFaces << "\n";
            bool search = f.FindNext(obj.prop_face);
            while (!f.IsLast() && search)
            {
                if (f.Word() == obj.prop_face)
                {
                    --numFaces; // represent what is left after this read
                    vector<pair<uint32_t, vertexComposite>> loadedVertex;
                    uint32_t numVertexThisFace = 0;
                    bool stopFlag = false;
                    bool badFaceRead = false;
                    std::string w;

                    do
                    {
                        w = f.NextWord();

                        if (w != obj.prop_face && w != obj.prop_meshName && w != obj.prop_modelName && w != obj.prop_useMaterial && w != obj.prop_vertexPosition && w != obj.prop_vertexNormal && w != obj.prop_vertexTextureCoord)
                        {

                            ++numVertexThisFace;
                            // f xyz/vn/uv
                            auto tokens = Tokenize(w, "/");
                            auto numTokens = tokens.size();

                            // uses the model definition.  We could improve and search the mesh first and use that as a condition as well.  Works fine so far, so skipping
                            bool loadUV = hasModelUV && (numTokens > 2);
                            bool loadNormals = hasModelNormals && (numTokens > 1);
                            if (!hasModelNormals && hasModelUV && numTokens == 2)
                                loadUV = true;

                            vertexCompositePadTo56 vert; // build xyz/uv/vn composite from indicies
                            // POTIONS 
                            // always have positions
                            auto index = (uint32_t)atoi(tokens[0].c_str()) - 1;
                            {
                                if (index < positions.size())
                                {
                                    vert.xyz[0] = positions[index][0];
                                    vert.xyz[1] = positions[index][1];
                                    vert.xyz[2] = positions[index][2];
                                }
                                else
                                {
                                    cout << "  BAD vertex position read from file, index is " << index << '\n';
                                    badFaceRead = true;
                                }
                            }
                            // TEXTURE 
                            if (loadUV)
                            {
                                index = (uint32_t)atoi(tokens[1].c_str()) - 1;
                                if (index < textureUV.size())
                                {
                                    vert.uv[0] = textureUV[index][0];
                                    vert.uv[1] = textureUV[index][1];
                                }
                                else
                                {
                                    cout << "  BAD vertex texture coordinate read from file, index is " << index << '\n';
                                    badFaceRead = true;
                                }
                            }
                            else
                            {
                                vert.uv[0] = 0.0f;
                                vert.uv[1] = 0.0f;
                            }
                            // NORMALS 
                            if (loadNormals)
                            {
                                if (numTokens == 3) index = (uint32_t)atoi(tokens[2].c_str()) - 1;
                                else index = (uint32_t)atoi(tokens[1].c_str()) - 1;
                                if (index < normals.size())
                                {
                                    vert.norm[0] = normals[index][0];
                                    vert.norm[1] = normals[index][1];
                                    vert.norm[2] = normals[index][2];
                                }
                                else
                                {
                                    cout << "  BAD vertex normal read from file, index is " << index << '\n';
                                    badFaceRead = true;
                                }
                            }
                            else
                            {
                                vert.norm[0] = 0.0f;
                                vert.norm[1] = 1.0f;
                                vert.norm[2] = 0.0f;
                            }

                            uint32_t vi = (uint32_t)verticiesComposite.size();
                            if (badFaceRead)
                            {
                                loadedVertex.push_back(pair<uint32_t, vertexComposite>(vi, vertexComposite(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f)));
                            }
                            else
                            {
                                loadedVertex.push_back(pair<uint32_t, vertexComposite>(vi, vert));
                            }
                        }
                        else
                            stopFlag = true;
                    } while (!f.IsLast() && w != "" && !stopFlag);


                    //*****
                    //  Add vertex to verticiesComposite
                    //*****
                    if (!badFaceRead)
                    {
                        // interpret face into triagles for mesh
                        // empty
                        if (numVertexThisFace == 0ul)
                        {
                        }
                        // point
                        else if (numVertexThisFace == 1ul)
                            ;//verticiesComposite.pop_back();
                        // line
                        else if (numVertexThisFace == 2ul)
                        {
                            //verticiesComposite.pop_back();
                            //verticiesComposite.pop_back();
                        }
                        // triangle strip
                        else if (numVertexThisFace > 2ul)
                        {
                            // Add our new verticies
                            // NOTE: loader reads in 3 or more but several are duplicates
                            uint32_t vi = (uint32_t)verticiesComposite.size();
                            for (auto& v1 : loadedVertex)
                            {
                                // remove duplicates (near search)
                                //if (vi > 4)
                                //{
                                //    // search previous 5 vertex
                                //    if (!CompareVertex(v1.second, verticiesComposite[vi - 1]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 2]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 3]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 4]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 5]))
                                //    {
                                //        // new
                                //        v1.first = verticiesComposite.size();
                                //        verticiesComposite.push_back(v1.second);
                                //    }
                                //    else
                                //        v1.first = UINT32_MAX;
                                //}
                                //else if (vi > 2) // 3 or 4 loaded, early special case
                                //{
                                //    // search previous triangle
                                //    if (!CompareVertex(v1.second, verticiesComposite[vi - 3]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 2]) &&
                                //        !CompareVertex(v1.second, verticiesComposite[vi - 1]))
                                //    {
                                //        // new
                                //        v1.first = verticiesComposite.size();
                                //        verticiesComposite.push_back(v1.second);
                                //    }
                                //    else
                                //        v1.first = UINT32_MAX;
                                //}
                                 // only 1 or 2 loaded so add this vertex
                                {
                                    // new
                                    v1.first = (uint32_t)verticiesComposite.size();
                                    verticiesComposite.push_back(v1.second);
                                }
                            }

                            for (auto& v1 : loadedVertex)
                            {
                                // replace the index of the duplicates
                                if (v1.first == UINT32_MAX)
                                {
                                    if (CompareVertex(v1.second, verticiesComposite[vi - 1]))
                                        v1.first = vi - 1;
                                    else if (CompareVertex(v1.second, verticiesComposite[vi - 2]))
                                        v1.first = vi - 2;
                                    else if (CompareVertex(v1.second, verticiesComposite[vi - 3]))
                                        v1.first = vi - 3;
                                    else if (CompareVertex(v1.second, verticiesComposite[vi - 4]))
                                        v1.first = vi - 4;
                                    else if (CompareVertex(v1.second, verticiesComposite[vi - 5]))
                                        v1.first = vi - 5;
                                    else
                                        cout << "  vertex replace duplicate failed!" << '\n';
                                }
                            }

                            // CCW winding order for right hand system
                            // blender orders them 0, 1, 3,2, and as extended (lower row) 0, 1, 2, 3, ... (upper row) 7, 6, 5, 4
                            // Blender -> Reordered
                            // 3    4   3    2
                            //  -----    -----
                            //  |  /|    |  /|
                            //  | / |  ->| / |
                            //  |/  |    |/  |
                            //  -----    -----
                            // 1    2   0    1
                            //          5    4   3
                            //           ---------
                            //           |  /|  /|
                            //           | / | / |
                            //           |/  |/  |
                            //           ---------
                            //          0    1   2

                            // note only verticies that are unique were added to verticiesComposite; numVertexThisFace is still correct and unadjusted

                            //uint32_t vertNum = (uint32_t)(verticiesComposite.size() - numVertexThisFace);
                            uint32_t numTri = (numVertexThisFace - 2);
                            uint32_t upperRow = numVertexThisFace - 1; // offset since blender order faces 1 2 3 6 5 4
                            bool even = !(bool)(numTri % 2);

                            auto quads = (numTri + 1) / 2; // last quad maybe just a triangle
                            uint32_t trianglesAdded = 0;

                            if (numTri == 1)
                            {
                                // first tri
                                indicies.push_back(loadedVertex[0].first); // 0
                                indicies.push_back(loadedVertex[1].first); // 1
                                indicies.push_back(loadedVertex[2].first); // 2
                            }
                            else
                                for (uint32_t i = 0; i < quads; ++i)
                                {

                                    // first tri in quad
                                    ++trianglesAdded;
                                    indicies.push_back(loadedVertex[i].first); // 0
                                    indicies.push_back(loadedVertex[i + 1].first); // 1
                                    indicies.push_back(loadedVertex[upperRow - i].first); // if a 2 then a triangle, 3 for quad, 4 for 3 triangles, etc                                 

                                    if (trianglesAdded < numTri)
                                    {
                                        ++trianglesAdded;
                                        indicies.push_back(loadedVertex[i + 1].first); // 1
                                        indicies.push_back(loadedVertex[upperRow - i - 1].first); // 2
                                        indicies.push_back(loadedVertex[upperRow - i].first); // 3   
                                    }
                                }
                        }
                    }
                    else
                    {
                        cout << "  Bad face read during load" << '\n';
                        for (unsigned long i = 0; i < numVertexThisFace; ++i)
                        {
                            // remove data if it was loaded
                            if (verticiesComposite.size())
                                verticiesComposite.pop_back();
                        }
                    }
                }
                else
                {
                    // not a face
                    if (numFaces > 0)
                        // separated face data for object, skip until it if found again
                        search = f.FindNext(obj.prop_face);
                    else
                        search = false;
                }
            } // end faces search for this model

  //          //if(loadNormals)
  //          //auto normals = vector<vector<float>>();
  //          //int numNormals;

  //          // smooth normals
  //          if (f.FindNextFrom(obj.postprocess_smoothNormals, wpos_meshes))
  //              if ((f.NextWord() == "on" || f.Word() == "1") && hasModelNormals)
  //              {
  //                  // *** TO DO *** average verticiesComposite[i].norm[0] not normals[i]
  //                  //// post process to smooth all normals of faces
  //                  //// assumes normals are unit normals
  //                  //uint32_t triangles = (uint32_t)indicies.size() / 3;
  ////                  vector<vector<float>> smoothNormals(indicies.size());
  ////                  std::fill(smoothNormals.begin(), smoothNormals.end(), 0.0f);
  //                  //// pass 1 sum
  //                  //for (uint32_t i = 0; i < triangles; ++i)
  //                  //{
  ////                      // *** TO DO *** this is not correct, numNormals == normals.size() which are the ones load in and indexed by the face, not each vertex
  ////                      // average the 3 vertex loaded in for the triangle
  ////                      smoothNormals[i][0] += normals[i][0];
  ////                      smoothNormals[i][1] += normals[i][1];
  ////                      smoothNormals[i][2] += normals[i][2];
  //                  //}
  //                  //// pass 2 average (normailize again to complete the average)
  //                  //for (uint32_t i = 0; i < verticiesComposite.size(); ++i)
  //                  //{
  ////                      float3 n(smoothNormals[i].data());
  //                  //    n.MakeNormalize();
  //                  //    verticiesComposite[i].norm[0] = n.f[0];
  //                  //    verticiesComposite[i].norm[1] = n.f[1];
  //                  //    verticiesComposite[i].norm[2] = n.f[2];
  //                  //}
  //              }
        }
        // Check face winding matches face normals
        if (true)
        {
            // NOTE: this was added to void King::ModelScaffold::ReverseWindingsToMatchNormals()
            auto fnFaceNormal = [](float3 a, float3 b, float3 c) {return Cross((b - a), (c - a)); };
            auto fnAveNormals = [](float3 a, float3 b, float3 c) {return Normalize(a + b + c); };
            const uint32_t num = (uint32_t)indicies.size();
            uint32_t offset(0), index(0);
            uint32_t numReversed = 0;
            for (uint32_t i = 0; i < num; i += 3)
            {
                float3 v0 = verticiesComposite[indicies[i]].xyz;
                float3 v1 = verticiesComposite[indicies[i + 1]].xyz;
                float3 v2 = verticiesComposite[indicies[i + 2]].xyz;
                auto faceNormal = fnFaceNormal(v0, v1, v2);

                float3 n0 = verticiesComposite[indicies[i]].norm;
                float3 n1 = verticiesComposite[indicies[i + 1]].norm;
                float3 n2 = verticiesComposite[indicies[i + 2]].norm;
                auto aveNormal = fnAveNormals(n0, n1, n2);

                float mustBePositive = Dot(faceNormal, aveNormal);
                if (abs(mustBePositive) != mustBePositive)
                {
                    ++numReversed;
                    auto temp = indicies[i + 1];
                    indicies[i + 1] = indicies[i + 2];
                    indicies[i + 2] = temp;
                }
            }
            cout << "  Reversed winding to match average face normal: " << numReversed << '\n';
        }

        //..............................................................................
        // Place composites into the model master data
        //..............................................................................
        MemoryBlock<uint8_t> vertexBufferRaw;
        MemoryBlock<uint32_t> indexBufferRaw(indicies.size());

        if (indexBufferRaw)
        {
            indexBufferRaw = indicies.data(); // copy
        }
        if (vertexFormatIn->GetByteSize() == sizeof(vertexComposite)) // position, uv, normals
        {
            vertexBufferRaw.Initialize(verticiesComposite.size() * sizeof(vertexComposite));
            vertexBufferRaw.SetStride(sizeof(vertexComposite));
            if (vertexBufferRaw) vertexBufferRaw = reinterpret_cast<uint8_t*>(verticiesComposite.data()); // copy
        }
        else if (vertexFormatIn->GetByteSize() == sizeof(vertexCompositePadTo56)) // position, uv, normals, tangents, bitangents
        {
            vertexBufferRaw.Initialize(verticiesComposite.size() * sizeof(vertexCompositePadTo56));
            vertexBufferRaw.SetStride(sizeof(vertexCompositePadTo56));
            // copy and pad
            if (vertexBufferRaw)
            {
                auto ptrFrom = verticiesComposite.data();
                for (size_t i = 0; i < verticiesComposite.size(); ++i)
                {
                    auto ptrTo = reinterpret_cast<vertexCompositePadTo56*>(&(vertexBufferRaw[i]));

                    *ptrTo = vertexCompositePadTo56(*ptrFrom); // copy
                    ++ptrFrom;
                }
            }
        }
        // append
        model->GetVertexBufferMaster() += vertexBufferRaw;
        model->GetIndexBufferMaster() += indexBufferRaw;
        // all the model data is loaded, so this is done only once, so the pointer does not
        // change and mesh loading is done next.

        //..............................................................................
        // Mesh loading (several per object separated by "g meshname" or "usemtl mtlname"
        //..............................................................................
        // Get number of meshes for model
        bool meshNameDefined;
        int numMeshes;

        f.WordIndex(wpos_models);
        if (!(numMeshes = f.CountStringAndStopAtMarker(obj.prop_meshName, obj.prop_modelName)))
        {
            numMeshes = 1;
            meshNameDefined = false;
            if (modelNameDefined)
            {
                f.FindFirst(obj.prop_modelName); // models are used to delimit (each is unique) data in the file
            }
        }
        else
        {
            meshNameDefined = f.FindFirst(obj.prop_meshName);
        }
        cout << "  numMeshes " << numMeshes << " with " << indicies.size() << " indicies\n";

        for (int meshCount = 0; meshCount < numMeshes; ++meshCount)
        {
            auto wpos_meshes = f.WordIndex() + 1; // word position after find then skip the found word
            string meshName;

            if (meshNameDefined)
            {
                meshName = f.NextWord();
            }
            else
            {
                f.FindNext(obj.prop_useMaterial);
                meshName = f.NextWord();
                //meshName = "mesh" + std::to_string(meshCount);
            }

            // material to use
            string mtlName;
            f.WordIndex(wpos_meshes);
            if (f.CountString(obj.prop_useMaterial))
            {
                f.FindNextFrom(obj.prop_useMaterial, wpos_meshes);
                mtlName = f.NextWord();
                models.back()->AddMaterial(std::make_shared<King::Material>(mtlName));
            }

            f.WordIndex(wpos_meshes);

            if (meshNameDefined)
            {
                // jump in file to the mesh name, otherwise take the next set of face definitions
                f.FindNext(meshName);
                cout << "  meshName " << meshName << '\n';
            }

            // create the mesh
            if(indicies.size()>=3)
            {
                TriangleMesh triMesh((uint32_t)indicies.size()/3, model->GetVertexFormat(), 0, 0, &model->GetVertexBufferMaster().GetData(), &model->GetIndexBufferMaster().GetData());
                triMesh.Set_name(meshName);
                triMesh.Set_materialName(mtlName);
                model->AddMesh(triMesh);
            }

            f.FindNextFrom(obj.prop_meshName, wpos_meshes);
        } // next mesh

        //model->_indexBufferMaster.WriteText("IB.txt");

        model->CalculateBoundingBox();

        if (!hasModelNormals) model->CalculateNormals();
        if (hasModelUV) model->CalculateTangentsAndBiTangents();

        f.FindNextFrom(obj.prop_modelName, wpos_models);

    } // next model

    //
    // Get materials
    //
    int numMaterials;
    if (!(numMaterials = f.CountString(obj.prop_useMaterial))) 
        return models;

    std::map<std::string, shared_ptr<King::Material>> materials;
    TextFileParse mtlFile;
    if (f.FindFirst(obj.prop_materialDefinitionFile))
    {
        auto name = f.NextWord();
        materials = Load_MTL(name);
    }

    // copy each mesh material into the master model material buffer and discard any unused materials
    for (auto & model : models)
    {
        for (auto & mesh : model->_meshes)
        {
            for (auto & mtl : materials)
            {
                if (mesh->Get_materialName() == mtl.second->Get_name())
                {
                    model->AddMaterial(mtl.second); // add the mesh's material to the model master definitions
                    break;
                }
            }
        }
    }

    // 
    for (auto& model : models)
    {
        cout << "  duplicate verts ";
        model->RemoveDuplicateVerticies();
        model->RemoveUnusedVerticies();
    }
    

    // an advantage of .obj files is by their nature they are vertex order optimized
    bool optimize(false);

    if (optimize)
        for (auto & model : models)
            model->OptimizeVertexBuffer();

    return models;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:    Save_OBJ
*    Description:    Exports King::Model in .obj format writing a material file
*                    and model file.  The model file may include one or more
*                    models and each model may define one or more meshes.
*
*    Inputs:        fileNameIN        name to give file
*                modelsIN        vector of models to save in this file
*    Outputs:    writes contents of models to file
*    Returns:    bool            false if error writing file
******************************************************************************/
bool King::Model_IO::Save_OBJ(const std::string fileNameIN, const std::vector<shared_ptr<King::Model>> &modelsIN)
{
    bool extendExport = true; // custom file format (for tangents and bi-tangent saves) that extends the .obj file format to .objx.  Set to false for standard export.
    float3 epsilon(0.00005f); // difference in which two numbers are considered equal

    string ext;
    if (extendExport)
        ext = ".objx";
    else
        ext = ".obj";

    std::time_t time = std::time(nullptr);
    std::string fileName(fileNameIN.begin(), fileNameIN.end()); // truncates and mostly only works for latin alpha

    stringstream ss(fileName);
    string prefixName;
    string postfixName;
    getline(ss, prefixName, '.');
    getline(ss, postfixName);

    ofstream of(prefixName + ext, std::ofstream::trunc);
    if (of.fail()) return false;

    cout << "Saving..." << prefixName << ext << '\n';

    // stats
    size_t totalMeshes = 0;
    for (const auto & model : modelsIN)
    {
        totalMeshes += model->GetNumMeshes();
    }

    if (extendExport)
    {
        of << "#" << " " << "EXTENDED File format" << '\n';
        of << "#" << " " << "OBJx file: " << fileName << '\n';
    }
    else
        of << "#" << " " << "OBJ file: " << fileName << '\n';

    char str[50];
    auto t = std::time(nullptr);
    tm tm;
    localtime_s(&tm, &t);
    asctime_s(str, sizeof str, &tm);

    of << "#" << " " << "Created: " << str;
    of << "#" << " " << "By: King Game Engine" << '\n';
    of << "#" << " " << "Models: " << std::to_string(modelsIN.size()) << '\n';
    of << "#" << " " << "Meshes: " << std::to_string(totalMeshes) << '\n';
    of << '\n';
    of.precision(6);

    if (extendExport)
        of << obj.prop_materialDefinitionFile << " " << prefixName << ".mtlx" << '\n' << '\n';
    else
        of << obj.prop_materialDefinitionFile << " " << prefixName << ".mtl" << '\n' << '\n';

    // compress per model
    for (const auto & model : modelsIN)
    {
        of << obj.prop_modelName << " " << model->GetModelName() << '\n';

        uint32_t numModelVerticies = 0;
        for (const auto & mesh : model->_meshes)
        {
            numModelVerticies += mesh->GetNumVerticies();
        }
        of << '#' << " Verticies in model (o object) " << numModelVerticies << '\n';

        const auto vertexFormat = model->GetVertexFormat();
        const auto vertexStride = model->GetVertexStride();

        bool hasPos(false), hasUV(false), hasNorm(false), hasTangent(false), hasBiTangent(false);

        // retrieves all for the model
        // .obj
        vector<vector<float>> locations;
        vector<vector<float>> normals;
        vector<vector<float>> textureUV;
        // .objx
        vector<vector<float>> tangents;
        vector<vector<float>> bitangents;

        if (hasPos = vertexFormat.Has(VertexAttrib::enumDesc::position))
            locations = model->GetPositions();
        if (hasNorm = vertexFormat.Has(VertexAttrib::enumDesc::normal))
            normals = model->GetNormals();
        if (hasUV = vertexFormat.Has(VertexAttrib::enumDesc::textureCoord))
            textureUV = model->GetTextureCoordinates();
        // reverse y of uv to be consistent with loads
        for (auto & uv : textureUV)
        {
            uv[1] *= -1.0f;
        }
        if (extendExport)
        {
            if (hasTangent = vertexFormat.Has(VertexAttrib::enumDesc::tangent))
                tangents = model->GetTangents();
            if (hasBiTangent = vertexFormat.Has(VertexAttrib::enumDesc::bitangent))
                bitangents = model->GetBiTangents();
        }

        // cycle each mesh
        //for (const auto & mesh : model->_meshes)
        // master data, not each mesh 8/16/2020
        {
            // NOTE: could retrieve attributes sets by mesh, not sure if I want to or like above, whole model

            /*
            we could build a hash table and only compare the vertex in the hash table.  If match,
            set the index in indexLocations[current->index] = k; 
            struct vertexHashEntry
            {
                XMFLOAT3            v;
                uint32_t            index;
                vertexHashEntry *   next; // list of entries with the same hash value
            };
            size_t hashSize = nVerts / 3;
            std::unique_ptr<vertexHashEntry*[]> hashTable(new (std::nothrow) vertexHashEntry*[hashSize]);
            if (!hashTable)
                return E_OUTOFMEMORY;
            memset(hashTable.get(), 0, sizeof(vertexHashEntry*) * hashSize);

            // would need to build the hasTable first by hashing and referencing vertex with index

            for (size_t vert = 0; vert < nVerts; ++vert)
            {
                auto px = reinterpret_cast<const uint32_t*>(&positions[vert].x);
                auto py = reinterpret_cast<const uint32_t*>(&positions[vert].y);
                auto pz = reinterpret_cast<const uint32_t*>(&positions[vert].z);
                uint32_t hashKey = (*px + *py + *pz) % uint32_t(hashSize);
                
                auto previous = hashTable[hashKey];
                for (auto current = previous; current != nullptr; current = current->next)
                {
                    float3 base(locations[k].data());
                    float3 test(locations[s].data());

                    if (DirectX::XMVector3NearEqual(base, test, epsilon))
                    {
                        // duplicate as me, set index the same
                        indexLocations[current->index] = k; 
                        // then remove from the hash table
                        previous->next = current->next;
                    }
                    previous = current;
                }
            */

            // *** locations *** remove duplicates
            vector<uint32_t> indexLocations(locations.size(), UINT32_MAX);
            if(indexLocations.size())
            {
                vector<vector<float>> uniqueLocations;
                uniqueLocations.reserve(locations.size());
                {
                    const auto ls = indexLocations.size();
                    // first find duplicates and give the same index
                    for (size_t k = 0; k < ls - 1; ++k)
                    {
                        cout << "  " << "Evaluating position " << k+2 << " of " << ls << '\r';
                        // unique
                        if (indexLocations[k] == UINT32_MAX) indexLocations[k] = static_cast<unsigned int>(k);
                        // search for matches to this index by location value
                        for (size_t s = k + 1; s < ls; ++s)
                        {
                            if (indexLocations[s] == UINT32_MAX) // not stored yet
                            {
                                float3 base(locations[k].data());
                                float3 test(locations[s].data());

                                if (DirectX::XMVector3NearEqual(base, test, epsilon))
                                {
                                    // duplicate as me, set index the same
                                    indexLocations[s] = static_cast<unsigned int>(k);
                                }
                                else
                                {
                                    // do nothing
                                }
                            }
                        }
                    }
                    cout << '\n';


                    // now we need to create a new list of uniques values in the order of remapped indicies
                    size_t last = 0; // last substitution
                    uniqueLocations.push_back(locations[0]); // first always unique
                    const auto ls2 = locations.size();
                    for (size_t k = 1; k < ls2; ++k)
                    {
                        cout << "  " << "Remapping positions " << (k + 1) / ls2 * 100.f << "%" << '\r';
                        //cout << "  " << "Remapping position " << k+1 << " of " << ls2 << '\r';
                        // unique test
                        auto u = indexLocations[k];
                        if (u > last)
                        {
                            last = u;
                            uniqueLocations.push_back(locations[k]);
                            // go forward and find all the duplicates and substitute the new index
                            for (size_t s = k; s < ls2; ++s) // starts with current of indexLocations[k]
                            {
                                if (indexLocations[s] == u) // duplicate reference
                                    indexLocations[s] = static_cast<unsigned int>(uniqueLocations.size() - 1);
                            }
                        }
                    }
                    cout << '\n';
                    cout << "  " << "Unique positions: " << uniqueLocations.size() << '\n';
                }
                // v
                of << "#" << " " << "Unique positions: " << std::to_string(uniqueLocations.size()) << '\n';
                for (const auto & p : uniqueLocations)
                    of << obj.prop_vertexPosition << " " << std::fixed << p[0] << " " << p[1] << " " << p[2] << " " << '\n';
            }
            // *** texture coordinates *** remove duplicates
            vector<uint32_t> indexTextureUV(textureUV.size(), UINT32_MAX);
            if (textureUV.size())
            {
                vector<vector<float>> uniqueTextureUV;
                uniqueTextureUV.reserve(textureUV.size());
                {
                    auto ts = textureUV.size();
                    for (size_t k = 0; k < ts-1; ++k)
                    {
                        cout << "  " << "Evaluating texture coordinate " << k+2 << " of " << ts << '\r';
                        // unique
                        if (indexTextureUV[k] == UINT32_MAX) indexTextureUV[k] = static_cast<unsigned int>(k);
                        // search
                        for (size_t s = k + 1; s < textureUV.size(); ++s)
                        {
                            if (indexTextureUV[s] == UINT32_MAX)
                            {
                                float2 base(textureUV[k].data());
                                float2 test(textureUV[s].data());
                                if (DirectX::XMVector3NearEqual(base, test, epsilon))
                                {
                                    // duplicate
                                    indexTextureUV[s] = (unsigned int)k;
                                }
                            }
                        }
                    }
                    cout << '\n';

                    // now we need to create a new list of uniques in the data values and remap the indicies to the unique data set
                    size_t last = 0; // last substitution
                    uniqueTextureUV.push_back(textureUV[0]);
                    const auto tc2 = textureUV.size();
                    for (size_t k = 1; k < tc2; ++k)
                    {
                        cout << "  " << "Remapping textures " << (k+1) / tc2 * 100.f << "%" << '\r';
                        //cout << "  " << "Remapping texture coordinate " << k+1 << " of " << tc2 << '\r';
                        // unique test
                        auto u = indexTextureUV[k];
                        if (u > last)
                        {
                            last = u;
                            uniqueTextureUV.push_back(textureUV[k]);
                            // go forward and find all the duplicates and substitute the new index
                            for (size_t s = k; s < tc2; ++s)
                            {
                                if (indexTextureUV[s] == u)
                                    indexTextureUV[s] = static_cast<unsigned int>(uniqueTextureUV.size() - 1);
                            }
                        }
                    }
                    cout << '\n';
                    cout << "  " << "Unique textures: " << uniqueTextureUV.size() << '\n';
                }
                // vt
                of << "#" << " " << "Unique texture UV: " << std::to_string(uniqueTextureUV.size()) << '\n';
                for (const auto & p : uniqueTextureUV)
                    of << obj.prop_vertexTextureCoord << " " << std::fixed << p[0] << " " << p[1] << " " << '\n';
            }
            // *** normals *** remove duplicates
            vector<uint32_t> indexNormals(normals.size(), UINT32_MAX);
            if (normals.size())
            {
                vector<vector<float>> uniqueNormals;
                uniqueNormals.reserve(normals.size());
                {
                    auto ns = normals.size();
                    for (size_t k = 0; k < ns-1; ++k)
                    {
                        cout << "  " << "Evaluating normal " << k + 2 << " of " << ns << '\r';
                        // unique
                        if (indexNormals[k] == UINT32_MAX) indexNormals[k] = static_cast<unsigned int>(k);
                        // search
                        for (size_t s = k + 1; s < normals.size(); ++s)
                        {
                            if (indexNormals[s] == UINT32_MAX)
                            {
                                float3 base(normals[k].data());
                                float3 test(normals[s].data());
                                if (DirectX::XMVector3NearEqual(base, test, epsilon))
                                {
                                    // duplicate
                                    indexNormals[s] = static_cast<unsigned int>(k);
                                }
                            }
                        }
                    }
                    cout << '\n';

                    // now we need to create a new list of uniques in the data values and remap the indicies to the unique data set
                    size_t last = 0; // last substitution
                    uniqueNormals.push_back(normals[0]);
                    const auto ns2 = normals.size();
                    for (size_t k = 1; k < ns2; ++k)
                    {
                        cout << "  " << "Remapping normals " << (k + 1) / ns2 * 100.f<< "%" << '\r';
                        //cout << "  " << "Remapping normal " << k + 1 << " of " << ns2 << '\r';
                        // unique test
                        auto u = indexNormals[k];
                        if (u > last)
                        {
                            last = u;
                            uniqueNormals.push_back(normals[k]);
                            // go forward and find all the duplicates and substitute the new index
                            for (size_t s = k; s < ns2; ++s)
                            {
                                if (indexNormals[s] == u)
                                    indexNormals[s] = static_cast<unsigned int>(uniqueNormals.size() - 1);
                            }
                        }
                    }
                    cout << '\n';
                    cout << "  " << "Unique normals: " << uniqueNormals.size() << '\n';
                }
                //vn
                of << "#" << " " << "Unique normals: " << std::to_string(uniqueNormals.size()) << '\n';
                for (const auto & p : uniqueNormals)
                    of << obj.prop_vertexNormal << " " << std::fixed << p[0] << " " << p[1] << " " << p[2] << " " << '\n';
            }
            if (extendExport)
            {
                // *** TO DO *** tangents and bitangents
                // vet veb
            }
            // write grouping (mesh) name
            //of << obj.prop_meshName << " " << mesh.Get_name() << '\n';
            //of << '#' << " Verticies in mesh (g group) " << mesh.GetNumVerticies() << '\n';
            // write material to use
            //of << obj.prop_useMaterial << " " << mesh.Get_materialName() << '\n';
            // write smoothing (we don't store, so make default no
            //of << obj.postprocess_smoothNormals << " " << "off" << '\n';
            // f
            of << "#" << " " << "Triangle faces: " << std::to_string(indexLocations.size()/3) << '\n';
            for (size_t it = 0; it < indexLocations.size(); it += 3)
            {
                // triangles
                of << obj.prop_face;
                of << " " << to_string(indexLocations[it] + 1) << "/" << to_string(indexTextureUV[it] + 1) << "/" << to_string(indexNormals[it] + 1);
                of << " " << to_string(indexLocations[it + 1] + 1) << "/" << to_string(indexTextureUV[it + 1] + 1) << "/" << to_string(indexNormals[it+1] + 1);
                of << " " << to_string(indexLocations[it + 2] + 1) << "/" << to_string(indexTextureUV[it + 2] + 1) << "/" << to_string(indexNormals[it+2] + 1) << '\n';
            }
        }
        std::time_t time2 = std::time(nullptr);
        auto dt = (double)(time2 - time); // secs
        dt /= 60.; // min
        of << '#' << " Time processing export: " << dt << " min" << '\n';
    }
    of.close();
    if (of.fail()) return false;
    else return true;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:    Load_M3D
*    Description:    Imports King::Model in .M3D format 
*                    
*                    
*
*    Inputs:        fileNameIN        name to give file
*    Outputs:    reads contents of meshes for each model from file
*    Returns:    std::vector        array of models
******************************************************************************/
//std::vector<shared_ptr<King::SkinnedModel>> King::Model_IO::Load_M3D(const std::string fileNameIN, const VertexFormat * vertexFormatIn)
//{
//    std::vector<shared_ptr<King::SkinnedModel>> models;
//
//    // M3D only contains one model per file
//    int numModels = 1;
//    {
//        models.push_back(SkinnedModel::Create());
//        auto model = models.back();
//        model->SetModelName(string(fileNameIN.begin(), fileNameIN.end()));
//
//        // read contents of file
//        M3DLoader loader;
//
//        std::string filename = model->GetModelName();
//        std::vector<struct M3DLoader::SkinnedVertex> skinVertices;
//        std::vector<USHORT> indices;
//        std::vector<struct M3DLoader::Subset> subsets;
//        std::vector<struct M3DLoader::M3dMaterial> mats;
//        SkinnedData skinInfo;
//
//        if (!loader.LoadM3d(filename, skinVertices, indices, subsets, mats, skinInfo))
//        {
//            models.clear();
//            return models;
//        }
//
//        // check contents
//        bool hasModelMaterials = mats.size() > 0 ? true : false;
//        bool hasModelSkeleton = skinInfo.BoneCount() > 0 ? true : false;
//        bool hasModelAnimationClips = skinInfo.AnimationCount() > 0 ? true : false;
//        bool hasModelBoneIndicies = indices.size() > 0 ? true : false;
//        bool hasModelMeshes = subsets.size() > 0 ? true : false;
//
//        // Define vertex format
//        if (vertexFormatIn) model->SetVertexFormat(*vertexFormatIn);
//        // build master index and vertex data
//        // attempt to fix format if missing type needed
//        auto vertexFormat = model->GetVertexFormat();
//        if (!vertexFormat.Has(King::VertexAttrib::enumDesc::position))
//        {
//            model->AddVertexAttribute(King::VertexAttrib::enumDesc::position, King::VertexAttrib::enumFormat::format_float32x3);
//        }
//        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::textureCoord))
//        {
//            model->AddVertexAttribute(King::VertexAttrib::enumDesc::textureCoord, King::VertexAttrib::enumFormat::format_float32x2);
//        }
//        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::normal))
//        {
//            model->AddVertexAttribute(King::VertexAttrib::enumDesc::normal, King::VertexAttrib::enumFormat::format_float32x3);
//        }
//        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::tangent))
//        {
//            model->AddVertexAttribute(King::VertexAttrib::enumDesc::tangent, King::VertexAttrib::enumFormat::format_float32x3);
//        }
//        if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::bitangent))
//        {
//            model->AddVertexAttribute(King::VertexAttrib::enumDesc::bitangent, King::VertexAttrib::enumFormat::format_float32x3);
//        }
//
//        // skinned
//        if (hasModelSkeleton)
//        {
//            model->_boneHierarchy = std::make_unique<BoneHierarchy>();
//
//            if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::boneWeights))
//            {
//                model->AddVertexAttribute(King::VertexAttrib::enumDesc::boneWeights, King::VertexAttrib::enumFormat::format_float32x3);
//            }
//            if (!model->GetVertexFormat().Has(King::VertexAttrib::enumDesc::boneIndicies))
//            {
//                model->AddVertexAttribute(King::VertexAttrib::enumDesc::boneIndicies, King::VertexAttrib::enumFormat::format_byte8x4);
//            }
//        }
//
//        // must be in the order we use in the loader below for memcpy to work correctly
//        VertexAttrib attr[7];
//        for(int i=0;i<7;++i)
//            attr[i] = model->GetVertexFormat().GetAttribute(i);
//        assert(
//            attr[0].GetDescription() == VertexAttrib::enumDesc::position &&
//            attr[1].GetDescription() == VertexAttrib::enumDesc::textureCoord &&
//            attr[2].GetDescription() == VertexAttrib::enumDesc::normal &&
//            attr[3].GetDescription() == VertexAttrib::enumDesc::tangent &&
//            attr[4].GetDescription() == VertexAttrib::enumDesc::bitangent
//        );
//
//        if(hasModelSkeleton)
//            assert(
//                attr[5].GetDescription() == VertexAttrib::enumDesc::boneWeights &&
//                attr[6].GetDescription() == VertexAttrib::enumDesc::boneIndicies
//            );
//
//        // vertex composite data
//        vector<std::shared_ptr<vertexComposite>> verticiesComposite;
//        for (const auto & v : skinVertices)
//        {
//            std::shared_ptr<vertexComposite> vert;
//            if (hasModelSkeleton)
//                vert = std::make_shared<vertexCompositePadTo72>();
//            else
//                vert = std::make_shared<vertexCompositePadTo56>();
//
//            vert->xyz[0] = v.Pos.x;
//            vert->xyz[1] = v.Pos.y;
//            vert->xyz[2] = v.Pos.z;
//
//            vert->uv[0] = v.TexC.x;
//            vert->uv[1] = v.TexC.y;
//            
//            vert->norm[0] = v.Normal.x;
//            vert->norm[1] = v.Normal.y;
//            vert->norm[2] = v.Normal.z;
//
//            if (auto p = std::dynamic_pointer_cast<vertexCompositePadTo56> (vert))
//            {
//                p->t[0] = v.TangentU.x;
//                p->t[1] = v.TangentU.y;
//                p->t[2] = v.TangentU.z;
//                // format does not store bi-tangent, must calculate it
//                p->bt[0] = 0.f;
//                p->bt[1] = 0.f;
//                p->bt[2] = 0.f;
//            }
//
//            if (auto p = std::dynamic_pointer_cast<vertexCompositePadTo72> (vert))
//            {
//                p->bw[0] = v.BoneWeights.x;
//                p->bw[1] = v.BoneWeights.y;
//                p->bw[2] = v.BoneWeights.z;
//
//                p->bi[0] = v.BoneIndices[0];
//                p->bi[1] = v.BoneIndices[1];
//                p->bi[2] = v.BoneIndices[2];
//                p->bi[3] = v.BoneIndices[3];
//            }
//
//            verticiesComposite.push_back(vert);
//        }
//        // index data
//        vector<uint32_t> indicies;
//        for (const auto & i : indices)
//        {
//            indicies.push_back(i);
//        }
//
//        // place our composites into the model master data
//        // *** store model master data ***
//        MemoryBlock<uint8_t> vertexBufferRaw;
//        MemoryBlock<uint32_t> indexBufferRaw(indicies.size());
//
//        if (indexBufferRaw)
//        {
//            indexBufferRaw = indicies.data(); // copy
//        }
//
//        /* Start new inprogress code */
//        if (model->GetVertexFormat().GetByteSize() == 56) // position, uv, normals, tangents, bitangents
//        {
//            vertexBufferRaw.Initialize(verticiesComposite.size() * sizeof(vertexCompositePadTo56));
//            vertexBufferRaw.SetStride(sizeof(vertexCompositePadTo56));
//            // copy
//            if (vertexBufferRaw)
//            {
//                for (size_t i = 0; i < verticiesComposite.size(); ++i)
//                {
//                    auto ptrTo = reinterpret_cast<vertexCompositePadTo56*>(&(vertexBufferRaw[i]));
//
//                    *ptrTo = vertexCompositePadTo56(*(std::dynamic_pointer_cast<vertexCompositePadTo56>(verticiesComposite[i]))); // copy
//                }
//            }
//        }
//        else if (model->GetVertexFormat().GetByteSize() == 72) // position, uv, normals, tangents, bitangents
//        {
//            vertexBufferRaw.Initialize(verticiesComposite.size() * sizeof(vertexCompositePadTo72));
//            vertexBufferRaw.SetStride(sizeof(vertexCompositePadTo72));
//            // copy
//            if (vertexBufferRaw)
//            {
//                for (size_t i = 0; i < verticiesComposite.size(); ++i)
//                {
//                    auto ptrTo = reinterpret_cast<vertexCompositePadTo72*>(&(vertexBufferRaw[i]));
//
//                    *ptrTo = vertexCompositePadTo72(*(std::dynamic_pointer_cast<vertexCompositePadTo72>(verticiesComposite[i]))); // copy
//                }
//            }
//        }
//        auto s1 = sizeof(vertexComposite);
//        auto s2 = sizeof(vertexCompositePadTo56);
//        auto s3 = sizeof(vertexCompositePadTo72);
//
//        auto vertStart = model->GetVertexBufferMaster().Size(); // vertex number to start our new indicies in the new master so indicies in a mesh are zero based for that mesh
//
//        // append
//        model->GetVertexBufferMaster() += vertexBufferRaw;
//        model->GetIndexBufferMaster() += indexBufferRaw;
//
//        // create the mesh
//        //if (indicies.size() >= 3)
//        //{
//        //    TriangleMesh triMesh(indicies.size() / 3, model->GetVertexFormat(), vertStart, 0, &model->GetVertexBufferMaster().GetData(), &model->GetIndexBufferMaster().GetData());
//        //    triMesh.Set_name(meshName);
//        //    triMesh.Set_materialName(mtlName);
//        //    model->AddMesh(triMesh);
//        //}
//        /* end new inprogress code */
//        subsets;
//        mats;
//        skinInfo;
//
//    } // end of model loading
//
//    // setup meshes
//    //for (UINT i = 0; i < (UINT)mSkinnedSubsets.size(); ++i)
//    //{
//    //    SubmeshGeometry submesh;
//    //    std::string name = "sm_" + std::to_string(i);
//
//    //    submesh.IndexCount = (UINT)mSkinnedSubsets[i].FaceCount * 3;
//    //    submesh.StartIndexLocation = mSkinnedSubsets[i].FaceStart * 3;
//    //    submesh.BaseVertexLocation = 0;
//
//    //    geo->DrawArgs[name] = submesh;
//    //}
//
//    // Calculate model information
//    for (auto & model : models)
//    {
//        model->CalculateBoundingBox();
//        model->CalculateTangentsAndBiTangents();
//    }
//    //
//    // Get materials (only the ones being used by models)
//    //
//    //int numMaterials;
//    //if (!(numMaterials = f.CountString(obj.prop_useMaterial)))
//    //    return models;
//
//    //std::map<std::string, shared_ptr<King::Material>> materials;
//    //TextFileParse mtlFile;
//    //if (f.FindFirst(obj.prop_materialDefinitionFile))
//    //{
//    //    auto name = f.NextWordW();
//    //    materials = Load_MTL(name);
//    //}
//
//    //// copy each mesh material into the master model material buffer and discard any unused materials
//    //for (auto & model : models)
//    //{
//    //    for (auto & mesh : model->_meshes)
//    //    {
//    //        for (auto & mtl : materials)
//    //        {
//    //            if (mesh.Get_materialName() == mtl.second->Get_name())
//    //            {
//    //                model->AddMaterial(mtl.second); // add the mesh's material to the model master definitions
//    //                break;
//    //            }
//    //        }
//    //    }
//    //}
//
//    return models;
//}
/******************************************************************************
*    Class:    Model_IO
*    Method:    Save_MTL
*    Description:    Exports .m3d model format
*
*    Inputs:        fileNameIN        name to give file
*                materialsIN        vector of materials to save in this file
*    Outputs:    writes contents of materials for each model to file
*    Returns:    bool            false if error writing file
******************************************************************************/
//bool King::Model_IO::Save_M3D(const std::string fileNameIN, shared_ptr<King::SkinnedModel>& modelIn, std::map<std::string, std::shared_ptr<Material>>& materialsIN)
//{
//    ofstream of(fileNameIN + ".m3d", std::ofstream::trunc);
//    if (of.fail()) return false;
//
//    // stats
//    std::time_t result = std::time(nullptr);
//    std::string fileName(fileNameIN.begin(), fileNameIN.end()); // truncates and mostly only works for latin alpha
//
//    of << "***" << "File:" << fileName;
//    of << "***" << "CreatedByKingEngineClassKing::Model_IO" << "***" << '\n';
//
//    of << "#" << " " << "Materials " << std::to_string(modelIn->GetNumMaterials()) << '\n';
//    of << "#" << " " << "Vertices " << std::to_string(modelIn->GetVertexCount()) << '\n';
//    of << "#" << " " << "Triangles " << std::to_string(modelIn->GetIndexCount()/3) << '\n';
//    of << "#" << " " << "Bones " << std::to_string(modelIn->GetBoneCount()) << '\n';
//    of << "#" << " " << "AnimationClips " << std::to_string(0) << '\n' << '\n';
//    of.precision(6);
//
//    of << "***************Materials*********************" << '\n';
////    Name: soldier_head
////    Diffuse : 1 1 1
////    Fresnel0 : 0.05 0.05 0.05
////    Roughness : 0.5
////    AlphaClip : 0
////    MaterialTypeName : Skinned
////    DiffuseMap : head_diff.dds
////    NormalMap : head_norm.dds
//
//    of << "***************SubsetTable*******************" << '\n';
////    SubsetID: 0 VertexStart : 0 VertexCount : 3915 FaceStart : 0 FaceCount : 7230
////    SubsetID : 1 VertexStart : 3915 VertexCount : 2984 FaceStart : 7230 FaceCount : 4449
////    SubsetID : 2 VertexStart : 6899 VertexCount : 4270 FaceStart : 11679 FaceCount : 6579
////    SubsetID : 3 VertexStart : 11169 VertexCount : 2305 FaceStart : 18258 FaceCount : 3807
////    SubsetID : 4 VertexStart : 13474 VertexCount : 274 FaceStart : 22065 FaceCount : 442
//    of << '\n';
//    of << "***************Vertices**********************" << '\n';
//    of << '\n';
////    Position: -0.9983482 67.88861 4.354969
////    Tangent : -0.257403 0.5351538 - 0.8045831 1
////    Normal : 0.5368453 0.7715151 0.3414111
////    Tex - Coords : 0.695366 0.9909569
////    BlendWeights : 0.7470379 0.2529621 0 0
////    BlendIndices : 7 9 0 0
//    of << '\n';
//    of << "***************Triangles*********************" << '\n';
//    //    0 1 2
//    //    3 4 5
//    //    6 7 8
//    //    9 10 11
//    //    12 13 14
//    //    15 16 17
//    of << '\n';
//    of << "***************BoneOffsets*******************" << '\n';
//    of << '\n';
//    of << "***************BoneHierarchy*****************" << '\n';
//    of << '\n';
//    of << "***************AnimationClips****************" << '\n';
//    of << '\n';
//
//    of.close();
//    if (of.fail()) return false;
//    else return true;
//}
/******************************************************************************
*    Class: Model_IO
*    Method: Load_MTL
*    Description: Imports King::Model material definitions from .mtl format
*
*    Inputs:     fileNameIN        name to give file
*    Outputs:    reads contents of materials for each model to file
*    Returns:    map             of materials by name
*   reference PBR extensions at http://exocortex.com/blog/extending_wavefront_mtl_to_support_pbr
******************************************************************************/
std::map<std::string, std::shared_ptr<Material>> King::Model_IO::Load_MTL(const std::string fileNameIN)
{
    std::map<std::string, std::shared_ptr<Material>> materials;
    TextFileParse mtlFile;
    {
        bool loaded = mtlFile.Load(fileNameIN);
        if (!loaded)
            loaded = mtlFile.Load("Models/" + fileNameIN);
        if (!loaded)
            loaded = mtlFile.Load("Materials/" + fileNameIN);
        if (!loaded)
            loaded = mtlFile.Load("Textures/" + fileNameIN);
        if (!loaded)
            loaded = mtlFile.Load("Resources/" + fileNameIN);
        if (!loaded)
            loaded = mtlFile.Load("Resources/Models/" + fileNameIN);
        if (!loaded)
        {
            cout << "Material file could not be located in defined paths: " << fileNameIN << '\n';
            assert(false);
        }
    }
    if (!mtlFile.FindFirst(obj.prop_defineMaterial))
        return materials;

    int numMaterials = mtlFile.CountString(obj.prop_defineMaterial);
    for (int i=0; i<numMaterials; ++i)
    {
        if (mtlFile.FindNext(obj.prop_defineMaterial))
        {
            const auto mtlName = mtlFile.NextWord();
            shared_ptr<King::Material> mtl = nullptr;
            for (auto & m : materials)
            {
                if (m.second->Get_name() == mtlName)
                    mtl = m.second; // allocated already
            }
            if (mtl == nullptr)
            {
                mtl = materials[mtlName] = std::make_shared<King::Material>(mtlName);
            }

            auto props = mtl->Get_properties();
            auto fileNames = mtl->Get_fileNames();
            auto ReadFloat = [&mtlFile]() { return (float)atof(mtlFile.NextWord().c_str()); };
            auto startPos = mtlFile.WordIndex();

            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.prop_indexOfRefraction, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.prop_indexOfRefraction, startPos);
                props.indexOfRefraction = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.prop_specularStrength, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.prop_specularStrength, startPos);
                props.specularStrength = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.color_ambient, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.color_ambient, startPos);
                props.color.ambient[0] = ReadFloat();
                props.color.ambient[1] = ReadFloat();
                props.color.ambient[2] = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.color_diffuse, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.color_diffuse, startPos);
                props.color.diffuse[0] = ReadFloat();
                props.color.diffuse[1] = ReadFloat();
                props.color.diffuse[2] = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.color_specular, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.color_specular, startPos);
                props.color.specular[0] = ReadFloat();
                props.color.specular[1] = ReadFloat();
                props.color.specular[2] = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.color_emissive, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.color_emissive, startPos);
                props.color.emissive[0] = ReadFloat();
                props.color.emissive[1] = ReadFloat();
                props.color.emissive[2] = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.prop_dissolved, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.prop_dissolved, startPos);
                props.dissolved = min(1.f, max(0.f, ReadFloat())); // alpha
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.prop_transparentInverse, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.prop_transparentInverse, startPos);
                props.dissolved = 1.0f - ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.prop_transmissionFilter, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.prop_transmissionFilter, startPos);
                props.transmissionFilter = ReadFloat();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.shader_method, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.shader_method, startPos);
                auto v = ReadFloat();
                props.isAmbient = (v > 0);
                props.isShiny = (v > 1);
                props.isCutOut = (v == 4 ? true : false);

            }
            // file names
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_diffuse, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_diffuse, startPos);
                fileNames.diffuse = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_specular, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_specular, startPos);
                fileNames.specular = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_specular_strength, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_specular_strength, startPos);
                fileNames.specular_strength = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_normal, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_normal, startPos);
                fileNames.normal = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_transparency, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_transparency, startPos);
                fileNames.transparency = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_ambient, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_ambient, startPos);
                fileNames.light = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_displacement, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_displacement, startPos);
                fileNames.displacement = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_stencil, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_stencil, startPos);
                fileNames.stencil = mtlFile.NextWord();
            }
            mtlFile.WordIndex(startPos);
            if (mtlFile.CountStringAndStopAtMarker(obj.map_reflection, obj.prop_defineMaterial))
            {
                mtlFile.FindNextFrom(obj.map_reflection, startPos);
                fileNames.reflection = mtlFile.NextWord();
            }
            // store values
            mtl->Set_name(mtlName);
            mtl->Set_properties(props);
            mtl->Set_fileNames(fileNames);
        }
    }
    return materials;
}
/******************************************************************************
*    Class:      Model_IO
*    Method:     Save_MTL
*    Description:    Exports King::Model material definitions in .mtl format
*
*    Inputs:     of              ofstream to save contents too
*                materialsIN     vector of materials to save in this file
*    Outputs:    writes contents of materials for each material to file
*    Returns:    bool            false if error writing file
******************************************************************************/
bool King::Model_IO::Save_MTL(ofstream & of, std::map<std::string, std::shared_ptr<Material>> & materialsIN)
{
    if(of.good()) 
        for (const auto & mtl : materialsIN)
        {
            of << '\n';
            of << obj.prop_defineMaterial << " " << mtl.second->Get_name() << '\n';
            of << obj.prop_specularStrength << " " << std::fixed << mtl.second->Get_properties().specularStrength << '\n';
            of << obj.color_ambient << " " << std::fixed << mtl.second->Get_properties().color.ambient[0] << " " << mtl.second->Get_properties().color.ambient[1] << " " << mtl.second->Get_properties().color.ambient[2] << '\n';
            of << obj.color_diffuse << " " << std::fixed << mtl.second->Get_properties().color.diffuse[0] << " " << mtl.second->Get_properties().color.diffuse[1] << " " << mtl.second->Get_properties().color.diffuse[2] << '\n';
            of << obj.color_specular << " " << std::fixed << mtl.second->Get_properties().color.specular[0] << " " << mtl.second->Get_properties().color.specular[1] << " " << mtl.second->Get_properties().color.specular[2] << '\n';
            of << obj.color_emissive << " " << std::fixed << mtl.second->Get_properties().color.emissive[0] << " " << mtl.second->Get_properties().color.emissive[1] << " " << mtl.second->Get_properties().color.emissive[2] << '\n';
            of << obj.prop_indexOfRefraction << " " << std::fixed << mtl.second->Get_properties().indexOfRefraction << '\n';
            of << obj.prop_dissolved << " " << std::fixed << mtl.second->Get_properties().dissolved << '\n';
            of << obj.prop_transmissionFilter << " " << std::fixed << mtl.second->Get_properties().transmissionFilter << '\n';
            of << "# " << "NOTE: illum 1 is no specular & no alpha, 2 is specular on & no alpha, 4 is specular on & alpha on" << '\n';
            of << obj.shader_method << (mtl.second->Get_properties().isCutOut ? " 4" : " 2") << '\n';
            of << obj.map_diffuse << " " << mtl.second->Get_fileNames().diffuse << '\n';
            of << obj.map_specular << " " << mtl.second->Get_fileNames().specular << '\n';
            of << obj.map_specular_strength << " " << mtl.second->Get_fileNames().specular_strength << '\n';
            of << obj.map_transparency << " " << mtl.second->Get_fileNames().transparency << '\n';
            of << obj.map_ambient << " " << mtl.second->Get_fileNames().light << '\n';
            of << obj.map_normal << " " << mtl.second->Get_fileNames().normal << '\n';
            of << obj.map_displacement << " " << mtl.second->Get_fileNames().displacement << '\n';
            of << obj.map_stencil << " " << mtl.second->Get_fileNames().stencil << '\n';
            of << obj.map_reflection << " " << mtl.second->Get_fileNames().reflection << '\n';
        }
    if (of.fail()) return false;
    else return true;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:    Save_MTL
*    Description:    Exports King::Model material definitions in .mtl format
*
*    Inputs:        fileNameIN        name to give file
*                materialsIN        vector of materials to save in this file
*    Outputs:    writes contents of materials for each model to file
*    Returns:    bool            false if error writing file
******************************************************************************/
bool King::Model_IO::Save_MTL(const std::string fileNameIN, std::map<std::string, std::shared_ptr<Material>>& materialsIN)
{
    std::string fileName(fileNameIN.begin(), fileNameIN.end()); // truncates and mostly only works for latin alpha

    stringstream ss(fileName);
    string prefixName;
    string postfixName;
    getline(ss, prefixName, '.');
    getline(ss, postfixName);

    ofstream of(prefixName + ".mtlx", std::ofstream::trunc);
    if (of.fail()) return false;

    // stats
    char str[50];
    auto t = std::time(nullptr);
    tm tm;
    localtime_s(&tm, &t);
    asctime_s(str, sizeof str, &tm);

    of << "#" << " " << "MTL file: " << fileName << '\n';
    of << "#" << " " << "Created: " << str;
    of << "#" << " " << "By: King Game Engine" << '\n';
    of << "#" << " " << "Materials: " << std::to_string(materialsIN.size()) << '\n';
    of.precision(6);

    Save_MTL(of, materialsIN);

    of.close();
    if (of.fail()) return false;
    else return true;
}
/******************************************************************************
*    Class:    Model_IO
*    Method:    Save_MTL
*    Description:    Exports King::Model material definitions in .mtl format 
*
*    Inputs:        fileNameIN        name to give file
*                modelsIN        vector of models to save in this file
*    Outputs:    writes contents of materials for each model to file
*    Returns:    bool            false if error writing file
******************************************************************************/
bool King::Model_IO::Save_MTL(const std::string fileNameIN, const std::vector<shared_ptr<King::Model>>& modelsIN)
{
    std::string fileName(fileNameIN.begin(), fileNameIN.end()); // truncates and mostly only works for latin alpha

    stringstream ss(fileName);
    string prefixName;
    string postfixName;
    getline(ss, prefixName, '.');
    getline(ss, postfixName);

    ofstream of(prefixName + ".mtlx", std::ofstream::trunc);
    if (of.fail()) return false;

    // stats
    char str[50];
    auto t = std::time(nullptr);
    tm tm;
    localtime_s(&tm, &t);
    asctime_s(str, sizeof str, &tm);

    of << "#" << " " << "MTL file: " << fileName << '\n';
    of << "#" << " " << "Created: " << str;
    of << "#" << " " << "By: King Game Engine" << '\n';

    size_t numMtl = 0;
    for (const auto & model : modelsIN)
    {
        numMtl += model->_materials.size();
    }
    of << "#" << " " << "Materials: " << std::to_string(modelsIN.size()) << '\n';
    of.precision(6);

    for (const auto & model : modelsIN)
    {
        Save_MTL(of, model->_materials);
    }
    of.close();
    if (of.fail()) return false;
    else return true;
}
/******************************************************************************
*    Helper - Tokenize
******************************************************************************/
vector<string> King::Model_IO::Tokenize(const string & str, const string & delimiters)
{
    vector<string> tokens;

    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    return tokens;
}

bool King::Model_IO::CompareVertex(const float * vertex1In, const float * vertex2In)
{
    float3 epsilon(0.00005f); // difference in which two numbers are considered equal
    array<float,8> vertex1;
    copy(vertex1In, vertex1In + 8, vertex1.data());
    array<float, 8> vertex2;
    copy(vertex2In, vertex2In + 8, vertex2.data());

    // positions
    float3 p1(vertex1.data());
    float3 p2(vertex2.data());
    if (DirectX::XMVector3NearEqual(p1, p2, epsilon))
    {
        // uv
        float2 uv1, uv2;
        uv1.Set(vertex1[3], vertex1[4]);
        uv2.Set(vertex2[3], vertex2[4]);
        if (DirectX::XMVector2NearEqual(uv1, uv2, epsilon))
        {
            // normals
            float3 norm1, norm2;
            norm1.Set(vertex1[5], vertex1[6], vertex1[7]);
            norm2.Set(vertex2[5], vertex2[6], vertex2[7]);
            if (DirectX::XMVector3NearEqual(norm1, norm2, epsilon))
                return true;
        }
    }
    return false;
}


