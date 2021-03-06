#pragma warning( disable : 4996)

#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "simulation.h"

//----------Events Related Variables--------------------//
static int g_old_screen_width;
static int g_old_screen_height;
static ScalarType g_old_timestep;
static ScalarType g_old_stretch_stiffness;
static ScalarType g_old_bending_stiffness;
static ScalarType g_old_attachment_stiffness;
static bool g_old_enable_bending_constrints;
static bool g_old_enable_collision_handling;

//----------Global Parameters----------------//
extern int g_screen_width;
extern int g_screen_height;

//----------State Control--------------------//
extern bool g_only_show_sim;
extern bool g_record;
extern bool g_pause;
extern bool g_show_wireframe;
extern bool g_show_texture;
extern bool g_enable_bending_constrints;
extern bool g_enable_collision_handling;

//----------anttweakbar handlers----------//
extern void TW_CALL reset_simulation(void*);
extern void TW_CALL step_through(void*);

//----------key components--------------//
extern Mesh* g_mesh;
extern Simulation* g_simulation;

AntTweakBarWrapper::AntTweakBarWrapper()
{
}

AntTweakBarWrapper::~AntTweakBarWrapper()
{
    SaveSettings(NULL);
    Cleanup();
}

void AntTweakBarWrapper::Init()
{
    TwInit(TW_OPENGL, NULL);

    //Control Panel bar
    m_control_panel_bar = TwNewBar("Control Panel");
    TwDefine(" 'Control Panel' size='200 710' position='814 10' color='255 255 255' text=dark ");
    char control_bar_pos_string [255];
    sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
    TwDefine(control_bar_pos_string);
    // state control
    TwAddVarRW(m_control_panel_bar, "Pause", TwType(sizeof(bool)), &(g_pause), "group='State Control'");
    TwAddButton(m_control_panel_bar, "Step Once", step_through, NULL, "group='State Control' ");
    TwAddVarRW(m_control_panel_bar, "Record", TwType(sizeof(bool)), &(g_record), "group='State Control'");
    TwAddSeparator(m_control_panel_bar, NULL, "");
    // visualization
    TwAddVarRW(m_control_panel_bar, "Wireframe", TwType(sizeof(bool)), &(g_show_wireframe), "group='Visualization'");
    TwAddVarRW(m_control_panel_bar, "Texture", TwType(sizeof(bool)), &(g_show_texture), "group='Visualization'");
    TwAddVarRW(m_control_panel_bar, "Width", TW_TYPE_INT32, &(g_screen_width), "min=640 group='Screen Resolution'");
    TwAddVarRW(m_control_panel_bar, "Height", TW_TYPE_INT32, &(g_screen_height), "min=480 group='Screen Resolution'");
    TwAddSeparator(m_control_panel_bar, NULL, "");
    // buttons
    TwAddButton(m_control_panel_bar, "Save Settings", SaveSettings, this, " ");
    TwAddButton(m_control_panel_bar, "Load Settings", LoadSettings, this, " ");
    TwAddButton(m_control_panel_bar, "Default Settings", SetDefaultSettings, this, " ");
    TwAddSeparator(m_control_panel_bar, NULL, "");
    TwAddButton(m_control_panel_bar, "Reset Simulation", reset_simulation, NULL, " ");
    //!Control Panel bar

    // mesh settings bar
    m_mesh_bar = TwNewBar("Mesh Settings");
    TwDefine(" 'Mesh Settings' size='200 250' position='10 10' color='210 240 255' text=dark ");
    // mesh type
    TwEnumVal meshTypeStyleEV[1] =  {
                                     {kMeshTypeTriangle, "Tet Mesh"}
									 // Please add another mesh type here !!
									};
    TwType meshTypeStyle = TwDefineEnum("MeshType", meshTypeStyleEV, 1);
    TwAddVarRW(m_mesh_bar, "Mesh Type", meshTypeStyle, &g_mesh->mesh_type_, " ");
    TwAddVarRW(m_mesh_bar, "Total Mass", TW_TYPE_SCALAR_TYPE, &(g_mesh->total_mass_), " ");
    // tet settings
    TwAddVarRW(m_mesh_bar, "Tet File", TW_TYPE_CSSTRING(sizeof(g_mesh->model_file_path_)), &(g_mesh->model_file_path_), " group='Tet Settings' ");
    TwAddVarRW(m_mesh_bar, "Tet Scaling", TW_TYPE_SCALAR_TYPE, &(g_mesh->mesh_scaling_), " min=0.01 group='Tet Settings' ");
    // !mesh settings bar

    // simulation settings bar
    m_sim_bar = TwNewBar("Simulation Settings");
    TwDefine(" 'Simulation Settings' size='200 450' position='10 270' color='255 216 224' text=dark ");
    // integration
    TwAddVarRW(m_sim_bar, "Time Step", TW_TYPE_SCALAR_TYPE, &g_simulation->m_h, " min=0.0001 step=0.0001 ");
    TwEnumVal integrationStyleEV[INTEGRATION_TOTAL_NUM] =  {
															{INTEGRATION_LOCAL_GLOBAL, "Local Global"}
															// Please add another integration type here !!
															};
    TwType integrationStyle = TwDefineEnum("Integration Method", integrationStyleEV, INTEGRATION_TOTAL_NUM);
    TwAddVarRW(m_sim_bar, "Method", integrationStyle, &g_simulation->m_integration_method, " group='Integration' ");
    TwAddVarRW(m_sim_bar, "Iterations/Frame", TW_TYPE_INT32, &g_simulation->m_iterations_per_frame, " group='Integration' ");
    // constants
    TwAddVarRW(m_sim_bar, "Attachment Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_attachment, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Stretch Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_stretch, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Bending Stiffness", TW_TYPE_SCALAR_TYPE, &g_simulation->m_stiffness_bending, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Gravity", TW_TYPE_SCALAR_TYPE, &g_simulation->m_gravity_constant, " group='Constants' ");
    TwAddVarRW(m_sim_bar, "Damping Coefficient", TW_TYPE_SCALAR_TYPE, &g_simulation->m_damping_coefficient, " min=0 step=0.001 group='Constants' ");
	// constraints control
	TwAddVarRW(m_sim_bar, "Enable Bending Constraint", TwType(sizeof(bool)), &(g_enable_bending_constrints), "group='Constraints Control'");
	// collision control
	TwAddVarRW(m_sim_bar, "Enable Collision Handling", TwType(sizeof(bool)), &(g_enable_collision_handling), "group='Collision Control'");

    // !simulation settings bar

    TwDefine(" TW_HELP visible=false ");
}

void AntTweakBarWrapper::Cleanup()
{
    m_control_panel_bar = NULL;
    m_mesh_bar = NULL;
    m_sim_bar = NULL;

    TwTerminate();
}

void AntTweakBarWrapper::Reset()
{
    Cleanup(); 
    Init();
}

void AntTweakBarWrapper::Hide()
{
    TwDefine(" 'Control Panel' visible=false ");
    TwDefine(" 'Mesh Settings' visible=false ");
    TwDefine(" 'Simulation Settings' visible=false ");
}

void AntTweakBarWrapper::Show()
{
    TwDefine(" 'Control Panel' visible=true ");
    TwDefine(" 'Mesh Settings' visible=true ");
    TwDefine(" 'Simulation Settings' visible=true ");
}

int AntTweakBarWrapper::Update()
{
    // update

    // control panel pos
    char control_bar_pos_string [255];
    sprintf(control_bar_pos_string, "'Control Panel' position='%d 10'", g_screen_width-210);
    TwDefine(control_bar_pos_string);

    // mesh settings display
    switch (g_mesh->mesh_type_)
    {
    case kMeshTypeTriangle:
        TwDefine(" 'Mesh Settings'/'Tet Settings' visible=true");
        break;
    }

    // simulation settings display
    switch(g_simulation->m_integration_method)
    {
    case INTEGRATION_LOCAL_GLOBAL:
        TwDefine(" 'Simulation Settings'/'Iterations/Frame' visible=true");
        break;
    }

    // give feed back
	ATBFeedBack atb_feedback = ATB_DEFAULT;
    if (g_old_screen_width!=g_screen_width || g_old_screen_height!=g_screen_height)
    {
        g_old_screen_width = g_screen_width;
        g_old_screen_height = g_screen_height;
		atb_feedback = ATB_RESHAPE_WINDOW;
    }
    if (g_simulation->m_h != g_old_timestep)
    {
        g_old_timestep = g_simulation->m_h;
		atb_feedback = ATB_CHANGE_TIME_STEP;
    }
    if (g_old_stretch_stiffness != g_simulation->m_stiffness_stretch ||\
        g_old_bending_stiffness != g_simulation->m_stiffness_bending ||\
        g_old_attachment_stiffness != g_simulation->m_stiffness_attachment)
    {
        g_old_stretch_stiffness = g_simulation->m_stiffness_stretch;
        g_old_bending_stiffness = g_simulation->m_stiffness_bending;
        g_old_attachment_stiffness = g_simulation->m_stiffness_attachment;
		atb_feedback = ATB_CHANGE_STIFFNESS;
    }
	if (g_old_enable_bending_constrints != g_enable_bending_constrints ||\
		g_old_enable_collision_handling != g_enable_collision_handling)
	{
		g_old_enable_bending_constrints = g_enable_bending_constrints;
		g_old_enable_collision_handling = g_enable_collision_handling;
		atb_feedback = ATB_NEED_RESET;
	}
    
    return atb_feedback;
}

void AntTweakBarWrapper::SaveSettings()
{
    std::ofstream outfile;
    outfile.open(DEFAULT_CONFIG_FILE, std::ifstream::out);
    if (outfile.is_open())
    {
        // TODO: change it to memory dump.
        // global settings:
        outfile << "Wireframe           " << g_show_wireframe << std::endl;
        outfile << "Texture             " << g_show_texture << std::endl;
        outfile << "ScreenWidth         " << g_screen_width << std::endl;
        outfile << "ScreenHeight        " << g_screen_height << std::endl;
        outfile << std::endl;

        // mesh settings:
        outfile << "MeshType            " << g_mesh->mesh_type_ << std::endl;
        outfile << "MeshMass            " << g_mesh->total_mass_ << std::endl;
        outfile << "TetFilePath         " << g_mesh->model_file_path_<< std::endl;
        outfile << "TetScaling          " << g_mesh->mesh_scaling_ << std::endl;
        outfile << std::endl;

        // simulation settings:
        outfile << "SimMethod           " << g_simulation->m_integration_method << std::endl;
        outfile << "Timestep            " << g_simulation->m_h << std::endl;

        outfile << "AttachmentStiffness " << g_simulation->m_stiffness_attachment << std::endl;
        outfile << "StretchStiffness    " << g_simulation->m_stiffness_stretch << std::endl;
        outfile << "BendingStiffness    " << g_simulation->m_stiffness_bending << std::endl;
        outfile << "GravityConstant     " << g_simulation->m_gravity_constant << std::endl;
        outfile << "DampingCoefficient  " << g_simulation->m_damping_coefficient << std::endl;

        outfile << "IterationsPerFrame  " << g_simulation->m_iterations_per_frame << std::endl;

        outfile.close();
    }
    else
    {
        std::cerr << "Warning: Can not write config file. Settings not saved." << std::endl; 
    }
}

void AntTweakBarWrapper::LoadSettings()
{
    bool successfulRead = false;

    //read file
    std::ifstream infile;
    infile.open(DEFAULT_CONFIG_FILE, std::ifstream::in);
    if (successfulRead = infile.is_open())
    {
        int tempEnum;
        char ignoreToken[256];

        // global settings:
        infile >> ignoreToken >> g_show_wireframe;
        infile >> ignoreToken >> g_show_texture;
        infile >> ignoreToken >> g_screen_width;
        infile >> ignoreToken >> g_screen_height;

        // mesh settings:
        infile >> ignoreToken >> tempEnum; g_mesh->mesh_type_ = MeshType(tempEnum);
        infile >> ignoreToken >> g_mesh->total_mass_;
        infile >> ignoreToken >> g_mesh->model_file_path_;
        infile >> ignoreToken >> g_mesh->mesh_scaling_;                         

        // simulation settings:
        infile >> ignoreToken >> tempEnum; g_simulation->m_integration_method = IntegrationMethod(tempEnum);
        infile >> ignoreToken >> g_simulation->m_h;

        infile >> ignoreToken >> g_simulation->m_stiffness_attachment;
        infile >> ignoreToken >> g_simulation->m_stiffness_stretch;
        infile >> ignoreToken >> g_simulation->m_stiffness_bending;
        infile >> ignoreToken >> g_simulation->m_gravity_constant;
        infile >> ignoreToken >> g_simulation->m_damping_coefficient;

        infile >> ignoreToken >> g_simulation->m_iterations_per_frame;

        infile.close();
    }

    // setup default values
    if (!successfulRead)
    {
        std::cerr << "Waning: failed loading settings, set to defaults." << std::endl;
        DefaultSettings();
    }

    // init event related variables
    g_old_screen_width = g_screen_width;
    g_old_screen_height = g_screen_height;
}

void AntTweakBarWrapper::DefaultSettings()
{
    // global settings
    g_show_wireframe = false;
    g_show_texture = false;
    g_screen_width = 1024;
    g_screen_height = 768;

    // mesh settings
    g_mesh->mesh_type_ = kMeshTypeTriangle;
    g_mesh->total_mass_ = 1.0;
    // tet
    strcpy(g_mesh->model_file_path_, DEFAULT_MODEL);
    g_mesh->mesh_scaling_ = 1.0;

    //simulation settings
    g_simulation->m_integration_method = INTEGRATION_LOCAL_GLOBAL;
    g_simulation->m_h = 0.0333;

    g_simulation->m_stiffness_attachment = 120;
    g_simulation->m_stiffness_stretch = 80;
    g_simulation->m_stiffness_bending = 20;
    g_simulation->m_gravity_constant = 100;
    g_simulation->m_damping_coefficient = 0.001;

    g_simulation->m_iterations_per_frame = 10;
}

void TW_CALL AntTweakBarWrapper::SaveSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->SaveSettings();
}

void TW_CALL AntTweakBarWrapper::LoadSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->LoadSettings();
    //resetSimulation(NULL);
}

void TW_CALL AntTweakBarWrapper::SetDefaultSettings(void* atb_wrapper)
{
    AntTweakBarWrapper* atb_wrapper_ref = (AntTweakBarWrapper*) atb_wrapper;
    atb_wrapper_ref->DefaultSettings();
    //resetSimulation(NULL);
}