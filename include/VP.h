#ifndef VP_H_INCLUDED
#define VP_H_INCLUDED

#define GLEW_STATIC
#include <GL/glew.h>

#include <GL/glfw.h>

//---CONSTANTS---

//Interleaved Array Names
enum
{
    VP_VERTEX_ARRAY,
    VP_NORMAL_ARRAY,
    VP_TEXCOORD_ARRAY,
    VP_COLOR_ARRAY,
    VP_MATRIX_ARRAY
};

//Render Passes
enum
{
    VP_RENDER_NONE,
    VP_RENDER_GEOMETRY,
    VP_RENDER_SKY
};

//Collision Groups
enum
{
    VP_COLLISION_NONE = 1,
    VP_COLLISION_PLAYER_BULLET = 2,
    VP_COLLISION_PLAYER_SHIP = 4,
    VP_COLLISION_ENEMY_BULLET = 8,
    VP_COLLISION_ENEMY_SHIP = 16,
    VP_COLLISION_SHIELD = 32
};

//Mesh Types
enum
{
    VP_MESH_TEXTURED,
    VP_MESH_COLORED
};

#endif // VP_H_INCLUDED
