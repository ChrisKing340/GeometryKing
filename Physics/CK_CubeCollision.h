/***************************************************************************/
/*                                                                         */
/*  File: CubeCollision.h                                                  */
/*  Date: 10-11-07                                                         */
/*                                                                         */
/***************************************************************************/
/*
	Because of the complexity of doing OBB-OBB collision detection, with
	the added determination of collision features, it was put into its own
	file.

	Basically, you pass in two OBB cubes, and the code will determine the
	collision points and the penetration depth and the direction of the
	collision to push the cubes appart.

	Its probably more complicated than it should be, and definetly needs a
	rewrite at some later date.
*/
/***************************************************************************/
#ifndef CUBECOLLISION_H
#define CUBECOLLISION_H

#include "..\MathSIMD\MathSIMD.h"

#include "CK_Cube.h"

namespace King {

    // Collision information
    struct stPoint
    {
        float3 point;
        float3 normal;
        float  pen;
        float3 pos0;
        float3 pos1;
    };
    // Collision information
    struct stCollisionPoints
    {
        Cube* box0;
        Cube* box1;
        stPoint	points[100];
        int	numPoints;
    };
    struct stCollisions
    {
        void Clear() { g_numCols = 0; }
        void Add(Cube* box0, Cube* box1, float3& point, float3& normal, float pen);
    };

    bool CubeCubeCollisionCheck(const Cube& box0,
        const Cube& box1,
        float3* hitPoints,
        int* numHitPoints,
        float* penetration,
        float3* hitNormal);
}

#endif //CUBECOLLISION_H

