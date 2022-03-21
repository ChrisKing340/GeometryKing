/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:			Model_IO	

Description:	Model extended for loading and saving

Usage:			

Contact:		ChrisKing340@gmail.com

(c) Copyrighted 2018 Christopher H. King all rights reserved.

References:		https://en.wikipedia.org/wiki/Wavefront_.obj_file

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#pragma once // needed for C++17 which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif
// std namespace
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <iostream>
// King namespace
#include "..\3DGeometryKing\3DGeometry.h"
#include "..\General\MemoryBlock.h"
#include "..\General\TextFileParse.h"
// M3D
//#include "SkinnedData.h"
//#include "LoadM3d.h"

namespace King {

	/******************************************************************************
	*	Vertex formats used by loaders
	******************************************************************************/
	class vertexComposite
	{
	public:
		float xyz[3]; // 12 position
	   	float uv[2]; // 8 texture coordinate
		float norm[3]; // 12 normal
		
		vertexComposite() = default;
		vertexComposite(const vertexComposite &in) = default;
		virtual ~vertexComposite() = default;

		virtual size_t size() { return 32; } // sizeof(vertexComposite) returns 40
	};

	class vertexCompositePadTo56 : public vertexComposite
	{
	public:
		float t[3] = { 0.f,0.f,0.f }; // 12 tangent
		float bt[3] = { 0.f,0.f,0.f }; // 12 bi-tangent

		vertexCompositePadTo56() = default;
		vertexCompositePadTo56(const vertexComposite &in)
		{
			xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
			uv[0] = in.uv[0]; uv[1] = in.uv[1];
			norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];
		}
		vertexCompositePadTo56(const vertexCompositePadTo56 &in)
		{
			xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
			uv[0] = in.uv[0]; uv[1] = in.uv[1];
			norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];

			t[0] = in.t[0]; t[1] = in.t[1]; t[2] = in.t[2];
			bt[0] = in.bt[0]; bt[1] = in.bt[1]; bt[2] = in.bt[2];
		}
		virtual ~vertexCompositePadTo56() = default;

		virtual size_t size() override { return 56; } // sizeof(vertexCompositePadTo56) returns 64
	};

	class vertexCompositePadTo72 : public vertexCompositePadTo56 // 80 in lieu of 72; why?
	{
	public:
		float bw[3] = { 0.f,0.f,0.f }; // 12 bone weights // 68
		unsigned char bi[4] = { 0,0,0,0 }; // 4 bone indicies // 72

		vertexCompositePadTo72() = default;
		vertexCompositePadTo72(const vertexComposite &in)
		{
			xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
			uv[0] = in.uv[0]; uv[1] = in.uv[1];
			norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];
		}
		vertexCompositePadTo72(const vertexCompositePadTo56 &in)
		{
			xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
			uv[0] = in.uv[0]; uv[1] = in.uv[1];
			norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];

			t[0] = in.t[0]; t[1] = in.t[1]; t[2] = in.t[2];
			bt[0] = in.bt[0]; bt[1] = in.bt[1]; bt[2] = in.bt[2];
		}
		vertexCompositePadTo72(const vertexCompositePadTo72 &in)
		{
			xyz[0] = in.xyz[0]; xyz[1] = in.xyz[1]; xyz[2] = in.xyz[2];
			uv[0] = in.uv[0]; uv[1] = in.uv[1];
			norm[0] = in.norm[0]; norm[1] = in.norm[1]; norm[2] = in.norm[2];

			t[0] = in.t[0]; t[1] = in.t[1]; t[2] = in.t[2];
			bt[0] = in.bt[0]; bt[1] = in.bt[1]; bt[2] = in.bt[2];

			bw[0] = in.bw[0]; bw[1] = in.bw[1]; bw[2] = in.bw[2];
			bi[0] = in.bi[0]; bi[1] = in.bi[1]; bi[2] = in.bi[2];
		}
		virtual ~vertexCompositePadTo72() = default;

		virtual size_t size() override { return 72; } // sizeof(vertexCompositePadTo72) returns 80
	};

	/******************************************************************************
	*	Model_IO
	******************************************************************************/
	class alignas(16) Model_IO
	{
		/* variables */
	public:
	private:
		struct OBJTokens {
			// tokens key words
			string prop_modelName			= "o"; // object model
			string prop_meshName			= "g"; // group meshes are subsets of object models
			string prop_vertexPosition		= "v";
			string prop_vertexNormal		= "vn";
			string prop_vertexTextureCoord	= "vt";
			string prop_face				= "f";
			string prop_vertexComponentSep	= "/";
			string prop_materialDefinitionFile	= "mtllib";
			string prop_defineMaterial		= "newmtl";
			string prop_useMaterial			= "usemtl";
			string prop_line				= "l";
        
			string postprocess_smoothNormals = "s";
        
			string modifer_orignOffset		= "-o";
			string modifer_scale			= "-s";
			string modifer_type				= "-type";

			// .mtl format tokens
            // reference PBR extensions at http ://exocortex.com/blog/extending_wavefront_mtl_to_support_pbr
			string prop_specularStrength	= "Ns"; // exponent 0 to 1000
			string color_ambient			= "Ka"; // ambient color
			string color_diffuse			= "Kd"; // diffuse color
			string color_specular			= "Ks"; // specular color
			string color_emissive			= "Ke"; // emissive color, extended format
			string prop_indexOfRefraction	= "Ni"; // 0.001 to 10; 1.0 means that light does not bend, glass has value 1.5
			string prop_dissolved			= "d"; // 0 transparent to 1 opaque (eg. alpha)
			string prop_transparentInverse	= "Tr"; // 0 opaque to 1 transparent 
			string prop_transmissionFilter	= "Tf"; // 0 to 1
        
			string shader_method			= "illum"; // 1 is default, 2 is specular on, 4 is cutouts

			string map_ambient				= "map_Ka"; // light map
			string map_diffuse				= "map_Kd"; // diffuse map
			string map_specular				= "map_Ks"; // specular map
			string map_specular_strength	= "map_Ns"; // specular highlight component
			string map_transparency			= "map_d";
			string map_normal				= "map_Bump";
			string map_displacement			= "map_disp";
			string map_stencil				= "map_decal";
			string map_reflection			= "map_refl";
		} const obj;
		
		struct M3DTokens {
			// tokens key words
			string prop_vertexPosition = "Position:";
			string prop_vertexNormal = "Normal:";
			string prop_vertexTextureCoord = "Tex-Coords:";
			string prop_vertexTangent = "Tangent:";
			string prop_vertexBlendWeights = "BlendWeights:";
			string prop_vertexBlendIndices = "BlendIndices:";

			// material tokens
			string prop_specularStrength = ""; // exponent 0 to 1000
			string color_ambient = ""; // ambient color
			string color_diffuse = ""; // diffuse color
			string color_specular = ""; // specular color
			string color_emissive = ""; // emissive color, extended format
			string prop_indexOfRefraction = ""; // 0.001 to 10; 1.0 means that light does not bend, glass has value 1.5
			string prop_dissolved = ""; // 0 transparent to 1 opaque
			string prop_transparentInverse = ""; // 0 opaque to 1 transparent 
			string prop_transmissionFilter = ""; // 0 to 1

			string shader_method = "";

			string map_ambient = "";
			string map_diffuse = "";
			string map_specular = "";
			string map_specular_strength = "";
			string map_transparency = "";
			string map_normal = "";
			string map_displacement = "";
			string map_stencil = "";
			string map_reflection = "";
		} const m3d;

		/* methods */
	public:
		// Creation/Life cycle
		static std::shared_ptr<Model_IO> Create() { return std::make_shared<Model_IO>(); }
		Model_IO() { ; }
		Model_IO(const Model_IO &in) { *this = in; } // forward to copy assignment
		Model_IO(Model_IO &&in) noexcept { *this = std::move(in); } // forward to move assignment

		virtual ~Model_IO() { Destroy(); }

		// Operators 
		void * operator new (size_t size) { return _aligned_malloc(size, 16); }
		void   operator delete (void *p) { _aligned_free(static_cast<Model_IO*>(p)); }
		inline Model_IO & operator= (const Model_IO &other) { if (this != &other) { ; } return *this; } // copy assign
		inline Model_IO & operator= (Model_IO &&other) noexcept { if (this != &other) { ; } return *this; } // move assign
		// Conversions
		// Functionality
		void												Destroy() { ; }
        // wavefront format supported by every popular modeling software for exporting data.  Lacks skeleton animation and inconsistentcies on modeling exports implementations (winding order, encoding of quads and larger polygons).
        //  our format is tested with with modeling software Blender.
		std::vector<shared_ptr<King::Model>>				Load_OBJ(const std::string fileNameIN, const VertexFormat *vertexFormatIn = nullptr);
		bool												Save_OBJ(const std::string fileNameIN, const std::vector<shared_ptr<King::ModelScaffold>> &modelsIN);
        // our native format in binary
        std::vector<shared_ptr<King::ModelScaffold>>		Load_KNG(const std::string fileNameIN);
        bool												Save_KNG(const std::string fileNameIN, const std::vector<shared_ptr<King::ModelScaffold>>& modelsIN);
		// https://en.wikipedia.org/wiki/Quake_III_Arena popular format, text based, and contains skelton animation data.  
		//std::vector<shared_ptr<King::SkinnedModel>>		Load_M3D(const std::string fileNameIN, const VertexFormat *vertexFormatIn = nullptr);
		//bool												Save_M3D(const std::string fileNameIN, shared_ptr<King::SkinnedModel>& modelIn, std::map<std::string, std::shared_ptr<Material>>& materialsIN);
        // material format used with wavefront files and our native format as accompany file
		std::map<std::string, std::shared_ptr<Material>>	Load_MTL(const std::string fileNameIN);
		bool												Save_MTL(const std::string fileNameIN, const std::vector<shared_ptr<King::ModelScaffold>> &modelsIN);
		bool 												Save_MTL(const std::string fileNameIN, std::map<std::string, std::shared_ptr<Material>> &materialsIN);
		bool												Save_MTL(std::ofstream & of, std::map<std::string, std::shared_ptr<Material>>& materialsIN);
		// Accessors
		// Assignments
    private:
		// Helpers		
		std::vector<std::string>							Tokenize(const std::string& str, const std::string& delimiters = " ");
        bool                                                CompareVertex(const float * vertex1In, const float * vertex2In);
	};

}
