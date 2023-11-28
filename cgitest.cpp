/*
    CGITEST OpenSceneGraph Version 3.6.5
    Adapted by DJA from EFS cgi.cpp

    stand alone test to generate an image from a save file
	to compile: make -f Makefile.cgitest cgitest

	Note: HUD, asi, alt, compass, glib removed
	DJA 11 Nov 2023
*/

/*
    System headers
*/
#ifdef WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <winsock2.h>
  #include <windows.h>
  #include <process.h>
#else // use elif __linux__ if moving back to linux at some point
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
#endif
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <iostream>

/*
    OpenSceneGraph Headers
*/
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Fog>
#include <osg/StateSet>
#include <osg/ClearNode>
#include <osg/Depth>
#include <osg/PositionAttitudeTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>
#include <osg/CullSettings>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Billboard>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/os_utils>
#include <osgDB/FileNameUtils>

/*
    Flt Sim headers. Note, change default packing on Windows for UDP structs arriving from Linux.
*/
#ifdef WIN32
#pragma pack(push,2)
#endif
#include <SIM/iosdefn.h>
#ifdef WIN32
#pragma pack(pop)
#endif

/*
    CGI Application definitions
*/
#define LeftChannel   0
#define CentreChannel 1
#define RightChannel  0

#define EarthRadius  6378137.0

#if LeftChannel
  //#define ChannelOffset 43.45f
  #define ChannelOffset 44.8f
#elif RightChannel
  //#define ChannelOffset -43.45f
  #define ChannelOffset -44.8f
#else
  #define ChannelOffset 0.0f
#endif

typedef struct {
    float              x;
    float              y;
    float              z;
    float              roll;
    float              pitch;
    float              yaw;
} Target_Info;


const double AcHeight    = -8.76;
const double CGHeight    = -5.1816;  /* 17 ft. previous values: (-4.05) (-4.78) */
const double Aero_EyeXStation = 25.19;
const double Aero_EyeYStation = 0.51;
const double Aero_EyeZStation = AcHeight - CGHeight;

/*
    Prototypes
*/
void               setname(char const *str);
void               setpath(char *str);
int                SelectDatabase ( char *argv[] );
void               SetTime ( float hour, float angle );
float              degrees ( float r );
float              normalise ( float x );
unsigned char      GetByte ( void );
int                GetInt ( void );
unsigned short int GetCard ( void );
int                GetBoolean ( void );
float              GetReal ( void );
void               GetFilename ( char *Str );
void               set_vis ( float vis );
void               CheckVisibility ( float h );
void               DecodeIosPkt ( void );
int                socket_get_data ( void );
int                socket_init ( void );
double             Bearing (double, double, double, double);
double             Distance (double, double, double, double);
float              rads (float);
float              Metres(float x);
void               SetDCM();
void               SetEyePosition();
void               Model_SetQuaternions(float pitch, float roll, float yaw);

/*
    Global data
*/

const double TWOPI  = (M_PI * 2.0);
const double ONERAD = (180.0 / M_PI);

// Database coordinates
static double RunwayX;
static double RunwayY;
static double RunwayZ;
static double RunwayRotation;

static float  visibility = 35000.0f;
static Target_Info Targets[2];

osgViewer::Viewer viewer;
// Model positions and orientations. To get comps. vec.x(), vec.y() etc. To set - vec.set(x,y,z)
osg::Vec3f vecPosAircraft;
osg::Vec3f vecAttAircraft;

// Global scene graph objects/nodes
osg::ref_ptr<osg::Fog> fog = new osg::Fog();
osg::ref_ptr<osg::LightSource> sunLight;
osg::ref_ptr<osg::PositionAttitudeTransform> TargetXForm = new osg::PositionAttitudeTransform();
osg::ref_ptr<osg::Group> SceneRoot = new osg::Group();

// File paths
osgDB::FilePathList pathList = osgDB::getDataFilePathList();
static char               filename[64];
static char               filepath[256];

static IosDefn_IosDataPkt IosPkt;

static float              Model_A11, Model_A12, Model_A13;
static float              Model_A21, Model_A22, Model_A23;
static float              Model_A31, Model_A32, Model_A33;
static float              Model_e0,  Model_e1,  Model_e2, Model_e3;
static float              Model_Ex,  Model_Ey,  Model_Ez;

/* ---------------------------------- */
void LoadRestoreFile(char Filename[])
{
    FILE *f;

    f = fopen(Filename, "rb");
    if (f == NULL)
    {
        printf("Error opening file %s\n", Filename);
        exit(-1);
    }
    fread(&IosPkt.RestoreVector, sizeof(IosDefn_RestoreVectorRecord), 1, f);
    fclose(f);
}

/* ---------------------------------------------------- */

/*
    Temporary function to set up a light source
*/
osg::ref_ptr<osg::LightSource> createSunLight(void)
{
    /*
    will have to create a function to dynamically alter the light levels
    dependent on the TimeOfDay. Could just do something similar to this func.

    Or use sunLight as declared in main and adjust its properites - better.
    eg. osg::Light* dynamicSunLight = sunLight->getLight()
    dynamicSunLight->setPosition
    dynamicSunLight->setAmbient
    dynamicSunLight->setDiffuse
    sunLight->setLight(dynamicSunLight);

    Same sort of thing for overall ambient light intensity in LightModel
    */

    osg::ref_ptr<osg::LightSource> sunLightSource = new osg::LightSource;
    osg::ref_ptr<osg::Light> sunLight = sunLightSource->getLight();
    sunLight->setPosition( osg::Vec4( -10000.0f, -10000.0f, 10000.0f, 1.0f ) );  /* not directly overhead */
    sunLight->setAmbient( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
    sunLight->setDiffuse( osg::Vec4( 0.8f, 0.8f, 0.8f, 1.0f ) );
    sunLightSource->setLight( sunLight.get() );
    sunLightSource->setLocalStateSetModes( osg::StateAttribute::ON );

    // Forcing OVERRIDE for lighting - otherwise trees in terrain remain unlit!
    // The trees are actually "extern" refs from main terrain db, so it is
    // likely that this is the source of unlit trees
    sunLightSource->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setAmbientIntensity(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    sunLightSource->getOrCreateStateSet()->setAttribute( lightModel.get() );

    return sunLightSource;
}

/* ---------------------------------------------------- */
int main (int argc, char* argv[])
{
    int s;

    if (argc <= 1)
    {
       printf("Error: No file name\n");
       exit(1);
    }

    if ( SelectDatabase(argv) )
    {
        printf("Error: No visual database\n");
        exit(1);
    }

    LoadRestoreFile(argv[2]);


	// Setting up a file paths for databases/models - and appending to the file path list
    osgDB::setDataFilePathList(pathList);

    // Load the 3d models. Uses "smart" pointers that manage memory better.
    osg::ref_ptr<osg::Node> TerrainNode = osgDB::readNodeFile(filename);
    if (!TerrainNode)
    {
        std::cerr << "Failed to load terrain database!\n";
        exit(1);
    }

    // Load the sky model - the thin cloud layer
    osg::ref_ptr<osg::Node> SkyNode = osgDB::readNodeFile("skydome.osgt");
    if (!SkyNode)
    {
        std::cerr << "Failed to load sky database!\n";
        exit(1);
    }

    // Using a sky box until I get fog working on clear buffer properly
    osg::ref_ptr<osg::Node> SkyBoxNode = osgDB::readNodeFile("models/skyb.ac");
    if (!SkyBoxNode)
    {
        std::cerr << "Failed to load skybox database!\n";
        exit(1);
    }

    // Set up a transform node to move the surrounding skybox a little lower
    // This stops the band of different colour (clear colour) showing through.
    osg::ref_ptr<osg::PositionAttitudeTransform> SkyBoxXForm = new osg::PositionAttitudeTransform();
    SkyBoxXForm->addChild(SkyBoxNode.get());
    osg::Vec3 SkyBoxPos(RunwayX, RunwayY, -1500.0);
    SkyBoxXForm->setPosition( SkyBoxPos );
    SkyBoxXForm->setScale(osg::Vec3(5.0,5.0,5.0));

    // Create blue sky - sets the clear buffer bits
    osg::ref_ptr<osg::ClearNode> backdrop = new osg::ClearNode;
    backdrop->setClearColor(osg::Vec4(0.8f,0.8f,1.0f,1.0f));

    // Enable fogging
    fog->setMode(osg::Fog::EXP);
    fog->setDensity(0.00005f);
    fog->setColor(osg::Vec4d(0.8,0.8,0.8,1.0));
    fog->setStart(0.0f);     // only for LINEAR fog
    fog->setEnd(visibility); // only for LINEAR fog

    // Fog node state
    osg::ref_ptr<osg::StateSet> fogStateSet = new osg::StateSet();
    fogStateSet->setAttribute(fog.get(), osg::StateAttribute::ON);
    fogStateSet->setMode(GL_FOG, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    //
    // Populate the scene graph. Could layout the graph a little more thoughfully.
    //
    SceneRoot->setStateSet(fogStateSet.get());

    sunLight = createSunLight();
    osg::ref_ptr<osg::Group> LitObjects = new osg::Group();
    LitObjects->addChild(sunLight.get());
    sunLight->addChild(TerrainNode.get());

    SceneRoot->addChild(SkyBoxXForm.get());
    SceneRoot->addChild(backdrop.get());
    SceneRoot->addChild(SkyNode.get());
    SceneRoot->addChild(LitObjects.get());

    // Initialise the camera with some default settings
    osg::Matrixd myCameraMatrix;
    osg::Matrixd cameraRotation;
    osg::Matrixd cameraOffsetRotation;
    osg::Matrixd cameraTrans;

    // Camera offset attitude
    cameraOffsetRotation.makeRotate(
        osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
        osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
        osg::DegreesToRadians(ChannelOffset), osg::Vec3(0,0,1) ); // heading

    // Configure UDP comms
    s = socket_init();
    if (s == EXIT_FAILURE)
    {
      printf("Unable to initialise socket\n");
      exit(1);
    }

    // Point the scene graph root to the viewer

    viewer.setSceneData( SceneRoot.get() );

    viewer.realize();

    // Many viewer settings only work once viewer.realize() has been called

    // switch off mouse cursor for all windows (we only use one)

    osgViewer::Viewer::Windows windows; // vector of GraphicsWindow
    viewer.getWindows(windows); // in ViewerBase (which Viewer inherits), get the list of GWs
    windows[0]->useCursor(false);
    //for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
    //  (*itr)->useCursor(false);
    //}

    // Field Of View settings

    // In Performer the near and far clipping planes = 5.0 and 50000.0 resp.
    // Performer Hfov and Vfov = 63.1 and 43.0 resp.
    // For OSG far set to be 200000 so sky box is drawn. This area needs improvement.
    // OSG 2.0 + doesnt provide a way to set fovx and fovy, so we use standare OpenGL interface.
    // see gluPerspective. fovy, aspect ratio, near, far
    // So OSG aspect ratio of wants tan(fovx/2)/tan(fovy/2) = 1.55877  1.599070

    viewer.getCamera()->setProjectionMatrixAsPerspective( 40.0, 1.599070, 1.0, 200000.0 );

    // Stop OSG optimising near and far clipping planes

    viewer.getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    // Disable small feature culling so we can see airfield lights from a distance

    osg::CullSettings::CullingMode cm = viewer.getCamera()->getCullingMode();
    cm &= ~(osg::CullSettings::SMALL_FEATURE_CULLING);
    viewer.getCamera()->setCullingMode(cm);
    viewer.addEventHandler(new osgViewer::StatsHandler); // ***

    //
    // Enter rendering loop
    //
    while (!viewer.done())
    {
        // read pkt from flight model
        socket_get_data();

        // Code to manually position and orient the camera. Move to function for neatness
        cameraTrans.makeTranslate( vecPosAircraft.x(), vecPosAircraft.y(), vecPosAircraft.z() ); // x = +ve RIGHT : y = +ve FORWARDS : Z = +ve UP
        cameraRotation.makeRotate(
            osg::DegreesToRadians(vecAttAircraft.z()), osg::Vec3(0,1,0),   // roll
            osg::DegreesToRadians(vecAttAircraft.y()), osg::Vec3(1,0,0) ,  // pitch
            osg::DegreesToRadians(vecAttAircraft.x()), osg::Vec3(0,0,1) ); // yaw
        myCameraMatrix = (cameraOffsetRotation * cameraRotation) * cameraTrans;
        osg::Matrixd i = myCameraMatrix.inverse(myCameraMatrix);
        osg::Matrixd xxx = osg::Matrixd::rotate( -M_PI*0.5, osg::Vec3(1,0,0) );
        viewer.getCamera()->setViewMatrix(i * xxx);

        // Position the skybox at the same position as the aircraft, but without any rotation
        SkyBoxPos.set( vecPosAircraft.x(), vecPosAircraft.y(), -1500);
        SkyBoxXForm->setPosition( SkyBoxPos );

        // Fire off the cull and draw traversals of the scene
        viewer.frame();
    }

    return 0;
}

/* ---------------------------------------------------- */

void setname(char const *str)
{
    strcpy(filename, str);
}

/* ---------------------------------------------------- */

void setpath(char *str)
{
    strcpy(filepath, str);
}

/* ---------------------------------------------------- */

int SelectDatabase ( char *argv[] )
{
    std::string databasePrefix = osg::getEnvVar("SIMDATA_PATH");
    if (databasePrefix.empty())
    {
        #ifdef WIN32
        databasePrefix = "c:\\SIM-DATA\\";
        #else
        databasePrefix = "/usr/share/SIM-DATA/";
        #endif
    }

    if ( !strcmp ( argv[1], "bristol" ) )
    {
        RunwayX  =        0.0;
        RunwayY  =        0.0;
        RunwayZ  =        0.0;
        RunwayRotation =  0.0;			/* 360 degrees */

        setname("long_bristol.flt");
        pathList.push_back(osgDB::concatPaths(databasePrefix, "databases/long_bristol/flt_files"));
        pathList.push_back(osgDB::concatPaths(databasePrefix, "databases/long_bristol/models"));
        pathList.push_back(osgDB::concatPaths(databasePrefix, "databases/long_bristol/texture"));
    }
    else if ( !strcmp ( argv[1], "hong_kong" ) )
    {
        RunwayX  =   -18552.0;
        RunwayY  =    -3003.0;
        RunwayZ  =        0.0;
        RunwayRotation =  1.815142422;		/* 104 degrees */
        setname("hk_sgi_demo.flt");
        pathList.push_back(osgDB::concatPaths(databasePrefix, "databases/hong_kong"));
    }
    else if ( !strcmp ( argv[1], "manchester" ) )
    {
        RunwayX        = 0.0;
        RunwayY        = -1091.0;
        RunwayZ        = -7.26;
        RunwayRotation = 0.0;

        setname("aerodrome-terrain.osgt");
        pathList.push_back(osgDB::concatPaths(databasePrefix, "databases/manchester"));
    }
    else
    {
        return -1;
    }

    pathList.push_back(osgDB::concatPaths(databasePrefix, "multigen"));
    pathList.push_back(osgDB::concatPaths(databasePrefix, "multigen/models"));
    pathList.push_back(osgDB::concatPaths(databasePrefix, "multigen/texture"));

#if 1
    std::cout<<"SelectDatabase() databasePrefix = "<<databasePrefix<<std::endl;
    for(auto& path : pathList)
    {
        std::cout<<" pathList "<<path<<std::endl;
    }
#endif
    return 0;
}


/* ---------------------------------------------------- */
/*                                                      */
/*  SetTime( Hour, Angle )                              */
/*  calculate the elevation and direction of the sun    */
/*                                                      */
/* ---------------------------------------------------- */

void SetTime( float hour, float angle )
{
    float d;

    if (hour < 6.0)
      d = 0.0;
    else
      if (hour > 18.0)
        d = 0.0;
      else
        d = 1.0 - fabs(12.0 - hour) / 6.0;


    osg::ref_ptr<osg::Light> UpdatedLight = sunLight->getLight();

    // Alter position of sun?
    //UpdatedLight->setPosition( osg::Vec4( 0.0f, 0.0f, 10000.0f, 1.0f ) );

    UpdatedLight->setAmbient( osg::Vec4( d, d, 0.8f*d, 1.0f ) );
    UpdatedLight->setDiffuse( osg::Vec4( d, d, 0.8f*d, 1.0f ) );

    sunLight->setLight( UpdatedLight.get() );
    sunLight->setLocalStateSetModes( osg::StateAttribute::ON );
    sunLight->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    fog->setColor(osg::Vec4d(d,d,d,1.0f));
}

/* ---------------------------------------------------- */

float degrees( float r )     /* radians -> degrees */
{
    return r * ONERAD;
}

/* ---------------------------------------------------- */

float rads( float d )     /* degrees -> radians */
{
    return d / ONERAD;
}

/* ------------------------------------------------------ */
float normalise(float a)
{
    double x = (double) a;
    double r = fmod(x + M_PI, TWOPI);

    if (r < 0.0)
    {
        r += TWOPI;
    }
    return (float) (r - M_PI);
}

/* ---------------------------------------------------- */

void set_vis( float vis )
{
    float    density;

/*  For linear fog.....
    fog->setEnd(vis);    */

/*  For exp fog....    */
    density = 3.2f/vis;
    fog->setDensity(density);

/*   For exp2 fog....
    density = 2.6f/vis;
    fog->setDensity(density);    */

}

/* ---------------------------------------------------- */

void CheckVisibility( float h )
{
}

/* ---------------------------------------------------- */

void DecodeIosPkt( void )
{
}

/* ---------------------------------------------------- */
double Distance(double Lat1, double Long1, double Lat2, double Long2)
{
    double dLat;
    double dLong;
    double d;

    dLat = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d = EarthRadius * sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    return d;
}

/* ---------------------------------------------------- */
double Bearing(double Lat1, double Long1, double Lat2, double Long2)
{
    double x, y;
    double ax, ay;
    double dLat;
    double dLong;
    double d;
    double psi;

    dLat = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d = sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    x = sin(Lat2) - sin(Lat1) * cos(d);
    ax = fabs(x);
    y = cos(Lat2) * sin(dLong) * cos(Lat1);
    ay = fabs(y);
    psi = atan2(ay, ax);
    if (x < 0.0)
    {
        psi = M_PI - psi;
    }
    if (y < 0.0)
    {
        return -psi;
    }
    else
    {
        return psi;
    }
}

/* ----------------------------------------- */
int socket_init(void)
{
    return EXIT_SUCCESS;
}

/* ---------------------------------------------------- */
int socket_get_data(void)
{
    float q;

    double x;
    double y;
    double z;
    double Pitch;
    double Roll;
    double Yaw;

    double a;
    double d;
    double b;

    double sina;
    double cosa;

    double px, py, pz;
    double h, p, r;

    IosDefn_RestoreVectorRecord rvr = IosPkt.RestoreVector;

    Pitch = rvr.Pitch;
    Roll = rvr.Roll;
    Yaw = rvr.Yaw;

    Model_SetQuaternions(Pitch, Roll, Yaw);
	SetDCM();
    SetEyePosition();

	//printf("lat=%f long=%f pitch=%f roll=%f yaw=%f\n", degrees(rvr.Latitude), degrees(rvr.Longitude), degrees(rvr.Pitch), degrees(rvr.Roll), degrees(rvr.Yaw)); // ***

    //rvr.CurrentRunway = 0; // ***

    if (rvr.CurrentRunway > 0)
    {
        double rqdm = rads(79.0);
        double rlat = rads(51.145861);
        double rlon = rads(-0.206831);

        q = normalise(rqdm + (double) (rads(-2.0)) );
        d = Distance(rvr.Latitude, rvr.Longitude, (double) rlat, (double) rlon);
        b = Bearing(rvr.Latitude, rvr.Longitude, (double) rlat, (double) rlon);
        x = -d * sin(b) + (double) Model_Ey;
        y = -d * cos(b) + (double) Model_Ex;
        z = -(rvr.Pz + Model_Ez) - Metres(195.0);
    }
    else
    {
        Pitch = 0.0;
		Roll = 0.0;
		Yaw = 0.0;
		x = 0.0;
        y = 0.0;
        z = -Model_Ez;
        q = 0.0;
    }

    Targets[0].x = x;
    Targets[0].y = y;
    Targets[0].z = z;
    Targets[0].pitch = Pitch;
    Targets[0].roll = Roll;
    Targets[0].yaw = Yaw;

    a = normalise(TWOPI - q - RunwayRotation);
    sina = sin(a);
    cosa = cos(a);

    /* position the aircraft */

    px = Targets[0].x * cosa + Targets[0].y * sina + RunwayX;
    py = Targets[0].y * cosa - Targets[0].x * sina + RunwayY;
    pz = Targets[0].z + RunwayZ;
    p = degrees(Targets[0].pitch);
    h = degrees(RunwayRotation - Targets[0].yaw + q);
    r = degrees(Targets[0].roll);
    //printf("px=%f py=%f pz=%f a=%f h=%f q=%f RunwayRotation=%f\n", px, py, pz, degrees(a), h, degrees(q), degrees(RunwayRotation)); // ***
    vecPosAircraft.set(px, py, pz);
    vecAttAircraft.set(h, p, r);

    SetTime(12.0, 0.0);  /* midday */
    set_vis(35000.0);    /* 35 Km */

    return EXIT_SUCCESS;
}

/* ---------------------------------------------------- */
float Metres(float x)
{
    return x * 0.30479999;
}

/* ----------------------------------------------- */
void SetDCM()
{
    float e00, e11, e22, e33;

    e00       = Model_e0 * Model_e0;
    e11       = Model_e1 * Model_e1;
    e22       = Model_e2 * Model_e2;
    e33       = Model_e3 * Model_e3;
    Model_A11 = e00 + e11 - e22 - e33;
    Model_A12 = 2.0 * (Model_e1 * Model_e2 - Model_e0 * Model_e3);
    Model_A13 = 2.0 * (Model_e0 * Model_e2 + Model_e1 * Model_e3);
    Model_A21 = 2.0 * (Model_e1 * Model_e2 + Model_e0 * Model_e3);
    Model_A22 = e00 - e11 + e22 - e33;
    Model_A23 = 2.0 * (Model_e2 * Model_e3 - Model_e0 * Model_e1);
    Model_A31 = 2.0 * (Model_e1 * Model_e3 - Model_e0 * Model_e2);
    Model_A32 = 2.0 * (Model_e2 * Model_e3 + Model_e0 * Model_e1);
    Model_A33 = e00 - e11 - e22 + e33;
}

/* ----------------------------------------------- */
void SetEyePosition()
{
    Model_Ex = Model_A11 * Aero_EyeXStation + Model_A12 * Aero_EyeYStation + Model_A13 * Aero_EyeZStation;
    Model_Ey = Model_A21 * Aero_EyeXStation + Model_A22 * Aero_EyeYStation + Model_A23 * Aero_EyeZStation;
    Model_Ez = Model_A31 * Aero_EyeXStation + Model_A32 * Aero_EyeYStation + Model_A33 * Aero_EyeZStation;
}

/* ----------------------------------------------- */
void Model_SetQuaternions(float pitch, float roll, float yaw)
{
    float p, r, y;
    float sp, cp, sr, cr, sy, cy;

    p        = pitch * 0.5;
    r        = roll * 0.5;
    y        = yaw * 0.5;
    sp       = sin(p);
    cp       = cos(p);
    sr       = sin(r);
    cr       = cos(r);
    sy       = sin(y);
    cy       = cos(y);
    Model_e0 = cr * cp * cy + sr * sp * sy;
    Model_e1 = sr * cp * cy - cr * sp * sy;
    Model_e2 = cr * sp * cy + sr * cp * sy;
    Model_e3 = cr * cp * sy - sr * sp * cy;
}
