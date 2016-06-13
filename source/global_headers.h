#ifndef _COMMON_HEADERS_H_
#define _COMMON_HEADERS_H_

// precision
#define HIGH_PRECISION

// matlab debugger
// NOTE: if you don't have Matlab R2010a or higher versions installed on your computer, please comment the next line.
// #define ENABLE_MATLAB_DEBUGGING

// single or double presicion
#ifdef HIGH_PRECISION
    typedef double ScalarType;
    #define TW_TYPE_SCALAR_TYPE TW_TYPE_DOUBLE
#else
    typedef float ScalarType;
    #define TW_TYPE_SCALAR_TYPE TW_TYPE_FLOAT
#endif

// small number and large number
#ifdef HIGH_PRECISION
    #define EPSILON 1e-15
#else
    #define EPSILON 1e-6
#endif

#define LARGER_EPSILON 1e-6

// use open mp
#define OPEN_MP

// default values
#define DEFAULT_SCREEN_WIDTH 1024
#define DEFAULT_SCREEN_HEIGHT 768

// selection radius
#define DEFAULT_SELECTION_RADIUS 0.1

// file localtions
#define DEFAULT_VERT_SHADER_FILE "./shaders/vert.glsl"
#define DEFAULT_FRAG_SHADER_FILE "./shaders/frag.glsl"
#define DEFAULT_SCENE_FILE "./scenes/test_scene.xml"
#define DEFAULT_CONFIG_FILE "./config/config.txt"
#define DEFAULT_CLOTH_STATE_FILE "./config/cloth.txt"
#define DEFAULT_TEXTURE_FILE "./textures/chinese_porcelain_1.png"
#define DEFAULT_SCREEN_SHOT_FILE "./output/ScreenShot.png"
#define DEFAULT_OUTPUT_CLOTH_STATE_FILE "./output/cloth.txt"
#define DEFAULT_OUTPUT_OBJ_FILE "./output/cloth.obj"
#define DEFAULT_MODEL "./mesh_models/frog_reg.mesh"
#define DEFAULT_OBJ_MODEL "./obj_models/bunny.obj"

// disallow copy and assign
#define DISALLOW_COPY_AND_ASSIGN(TypeName)	\
	TypeName(const TypeName&) = delete;		\
	void operator=(const TypeName&) = delete	

#endif