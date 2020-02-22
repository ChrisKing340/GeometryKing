# GeometryKing
2D and 3D geometry fundamentals and advanced modeling.  C++ generic Classes with no rendering dependentcies.

GeometryKing contains the base SIMD data types class wrappers to seamlessly accelerate your code using the popular DirectXMath library (not dependent on DirectX). From this base, GeometryKing defines geometry types, classes and methods that make the basis of 2D and 3D games, simulators, and engineering applications.  

Compiled with Visual Studio 2019, inteded for 64 Bit Windows 10 but may work just fine on 32 Bit Windows 10 or lesser machine.

This code is a small part of a fully functional DirectX 12 game engine and physics simulator of mine. Typical usage is load, store, manipulate, and interact with geometry constructs and models.  My game engine, not included here, uses DirectX12 to define graphics render classes that inherit from GeometryKing model class, for example, to add methods that copy data into the DirectX 12 GPU buffers ready for rendering, among other functionality.  I hope you enjoy this work which is free to you and covered under the MIT license.

For the latest version of DirectXMath, visit:
<https://github.com/Microsoft/DirectXMath>
