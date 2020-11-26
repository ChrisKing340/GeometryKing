
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          GeometryKing

Description:    File to easily include all library headers
                https://github.com/ChrisKing340/GeometryKing

                2D and 3D geometry fundamentals and advanced modeling. C++ generic 
                Classes with no rendering dependencies.

                GeometryKing contains the base SIMD data types class wrappers to 
                seamlessly accelerate your code using the popular DirectXMath 
                library (not dependent on DirectX). From this base, GeometryKing 
                defines geometry types, classes and methods that make the basis of
                2D and 3D games, simulators, and engineering applications.

                Compiled with Visual Studio 2019, inteded for 64 Bit Windows 10 but
                may work just fine on 32 Bit Windows 10 or lesser machine.

                This code is a small part of a fully functional DirectX 12 game 
                engine and physics simulator of mine. Typical usage is load, store, 
                manipulate, and interact with geometry constructs and models. My 
                game engine, not included here, uses DirectX12 to define graphics 
                render classes that inherit from GeometryKing model class, 
                for example, to add methods that copy data into the DirectX 12 GPU 
                buffers ready for rendering, among other functionality. I hope you 
                enjoy this work which is free to you and covered under the MIT license.

Contact:        ChrisKing340@gmail.com

References:     json input and output utilize https://github.com/nlohmann/json
                SIMD math functions utilize https://github.com/microsoft/DirectXMath

MIT License

Copyright (c) 2020 Christopher H. King

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
#pragma once

// Math foundation
#include "MathSIMD\MathSIMD.h"
// General utilities
#include "General\MemoryBlock.h"
#include "General\TextFileParse.h"
// Physics
#include "Physics\Physics.h"
// Geometry
#include "2DGeometryKing\2DGeometry.h"
#include "3DGeometryKing\3DGeometry.h"
// Input/Output
#include "3DGeometryKing\Model_IO.h"
