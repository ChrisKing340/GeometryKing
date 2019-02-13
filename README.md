# GeometryKing
C++ Classes for 2D and 3D geometry with SIMD support using DirectXMath

For the latest version of DirectXMath, visit:
<https://github.com/Microsoft/DirectXMath>

GeometryKing contains the base SIMD data types class wrappers to seamlessly accelerate your code while working with legacy windows multi data types (think POINT) that are not accelerated.  From this base, GeometryKing defines geometry types and methods that make the basis of 2D and 3D games, simulators, and engineering applications.  Code base is intended to work on modern, 64-bit operating systems and tested on Windows 10.  Therefore, 16 byte alignment is required.  For use on 32-bit, the classes have been aligned to 16 byte so the code base will work, just note that I rarely build and test to 32 bit and you have to maintain 16 byte alignment if you extend my work by deriving from GeometryKing classes.

This code base builds into a fully functional DirectX 12 game engine and physics simulator of mine.  The classes here are not dependent on any graphics API and intended to be generic without rendering methods.  My graphics render classes inherit from my model class, for example, to add that functionality.

I hope you enjoy this work which is free to you and covered under the MIT license for your distribution and use, including in commercial work.  If you do, a citation in the credits would be appreciated :).  Enjoy!
