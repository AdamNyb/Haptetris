/*
 *
 *
 *    DH2660 Haptic Programming spring 2017
 *    Lab 2 Template directly based on Chai3D Example 21.
 *
 *    Distribution license: BSD (e.g. free to use for
 *    most purposes, see end of file)
 *
 */

//------------------------------------------------------------------------------
#include "chai3d.h"

//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"
//#include "ode/ode.h"
//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a virtual object
//cMultiMesh* object;

cMultiMesh* object1;
///////////////////////////////////////////////////
// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEBlock;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

////////////////////////////////////////////
// a pointer to the current haptic device
//cGenericHapticDevicePtr hapticDevice;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// display options
bool showEdges = true;
bool showTriangles = true;
bool showNormals = false;

// display level for collision tree
//int collisionTreeDisplayLevel = 0;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;




// Game level boundaries
struct boxBorders {
    float left = -0.05;
    float right = 0.05;
    float top = 0.1;
    float bottom = -0.07;
    float back = 0.04;
    float front = 0.042;
};
boxBorders boundaries;

/*struct wallBorders {
    float left = -0.05;
    float right = 0.05;
    float top = 0.05;
    float bottom = -0.05;
};
wallBorders visualBoundaries;*/


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback to render graphic scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:   21-object.cpp

    This demonstration loads a 3D mesh file by using the file loader
    functionality of the \ref csh class. A finger-proxy algorithm is
    used to render the forces. Take a look at this example to understand the
    different functionalities offered by the tool force renderer.
    object->setMaterial(m);

    // disable culling so that faces are rendered on both sides
    object->setUseCulling(false);

    // compute a boundary box
    object->computeBoundaryBox(true);

    // show/hide boundary box
    object->setShowBoundaryBox(false);

    // compute collision detection algorithm
    object->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    object->setStiffness(0.2 * maxStiffness, true);

    // define some haptic friction properties
    object->setFriction(0.1, 0.2, true);

    // enable display list for faster graphic rendering
    object->setUseDisplayList(true);

    // center object in scene
    object->setLocalPos(-1.0 * object->getBoundaryCenter());

    // rotate object in scene
    //object->rotateExtrinsicEulerAnglesDeg(0, 0, 90, C_EULER_ORDER_XYZ);


    // compute all edges of object for which adjacent triangles have more than 40 degree angle
    object->computeAllEdges(0);

    // set line width of edges and color
    cColorf colorEdges;
    colorEdges.setBlack();
    object->setEdgeProperties(1, colorEdges);

    // set normal properties for display
    cColorf colorNormals;
    colorNormals.setOrangeTomato();
    object->setNormalsProperties(0.01, colorNormals);

    // display options
    object->setShowTriangles(showTriangles);
    object->setShowEdges(showEdges);
    object->setShowNormals(showNormals);
    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
    Finally, a simple dynamics model is used to simulate the behavior
    of the object.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 21-object" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Texture   (ON/OFF)" << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[3] - Collision tree (ON/OFF)" << endl;
    cout << "[4] - Increase collision tree display depth" << endl;
    cout << "[5] - Decrease collision tree display depth" << endl;
    cout << "[s] - Save screenshot to file" << endl;
    cout << "[e] - Enable/Disable display of edges" << endl;
    cout << "[t] - Enable/Disable display of triangles" << endl;
    cout << "[n] - Enable/Disable display of normals" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << boundaries.right << endl;
    cout << endl << endl;


    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);


    /*
    // define a basis in spherical coordinates for the camera
    camera->setSphericalReferences(cVector3d(0.8, 0.0, 0.5),    // origin
                                   cVector3d(0.0, 0.0, 1.0),    // zenith direction
                                   cVector3d(1.0, 0.0, 0.0));   // azimuth direction

    camera->setSphericalDeg(1.0,    // spherical coordinate radius
                            65,     // spherical coordinate polar angle
                            20);    // spherical coordinate azimuth angle
    */
    // position and orient the camera
    camera->set( cVector3d (0.4, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 100);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(1.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.6f, 0.6f, 0.6f);
    light->m_diffuse.set(0.8f, 0.8f, 0.8f);
    light->m_specular.set(1.0f, 1.0f, 1.0f);

    //WALLS
    /*cMaterial wallMats;
    wallMats.setShininess(100);

    cShapeBox *leftWall = new cShapeBox(5,0.0001,0.3);
    //leftWall->translate(0, -0.06, 0);
    world -> addChild(leftWall);
    leftWall->setLocalPos(0,boundaries.left,0);
    //leftWall->setLocalPos(0,0,0);
    leftWall->setMaterial(wallMats,true);

    cShapeBox *rightWall = new cShapeBox(5,0.0001,0.3);
    world -> addChild(rightWall);
    rightWall-> setLocalPos(0,boundaries.right,0);

    cShapeBox *ceiling = new cShapeBox(5,0.3,0.000001);
    world-> addChild(ceiling);
    ceiling->setLocalPos(0,0,boundaries.top);

    cShapeBox *floor = new cShapeBox(5,0.3,0.000001);
    world -> addChild(floor);
    floor->setLocalPos(0,0,boundaries.bottom);

    for (int i = 0; i < 15; i++) {
        cShapeLine *borderRight = new cShapeLine(cVector3d(i*0.01,boundaries.right,1), cVector3d(i*0.01,boundaries.right,-1));
        world -> addChild(borderRight);
        cShapeLine *borderLeft = new cShapeLine(cVector3d(i*0.01,boundaries.left,1), cVector3d(i*0.01,boundaries.left,-1));
        world -> addChild(borderLeft);
        cShapeLine *borderTop = new cShapeLine(cVector3d(i*0.01,-1,boundaries.top), cVector3d(i*0.01,1,boundaries.top));
        world -> addChild(borderTop);
        cShapeLine *borderBottom = new cShapeLine(cVector3d(i*0.01,-1,boundaries.bottom), cVector3d(i*0.01,1,boundaries.bottom));
        world -> addChild(borderBottom);
    }
    cShapeLine *borderRight = new cShapeLine(cVector3d(0,boundaries.right,1), cVector3d(0,boundaries.right,-1));
    world -> addChild(borderRight);
    cShapeLine *borderLeft = new cShapeLine(cVector3d(0,boundaries.left,1), cVector3d(0,boundaries.left,-1));
    world -> addChild(borderLeft);
    cShapeLine *borderTop = new cShapeLine(cVector3d(0,-1,boundaries.top), cVector3d(0,1,boundaries.top));
    world -> addChild(borderTop);
    cShapeLine *borderBottom = new cShapeLine(cVector3d(0,-1,boundaries.bottom), cVector3d(0,1,boundaries.bottom));
    world -> addChild(borderBottom);*/

    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    shared_ptr<cGenericHapticDevice> hapticDevice;
    handler->getDevice(hapticDevice, 0);
//cMultiMesh* object = new cMultiMesh();
    cout << "AAAAAAAAAAAAAAAAAAAAH: " << boundaries.right << endl;

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the haptic device has a gripper, enable it as a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.1);

    // define the radius of the tool (sphere)
    double toolRadius = 0.005;

    // define a radius for the tool
    tool->setRadius(toolRadius, toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    //tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // oriente tool with camera
    //tool->setLocalRot(camera->getLocalRot());

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECT
    //--------------------------------------------------------------------------

    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);
    ////////////////////////////////////////////////////
    ODEBlock = new cODEGenericBody(ODEWorld);


    // create a virtual mesh
    cMultiMesh* object = new cMultiMesh();
    //object1 = new cMultiMesh();
    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body

    // add object to world
    //world->addChild(object);
    //world->addChild(object1);


//    world->addChild(object);


    // create a new ODE object that is automatically added to the ODE world
  //  ODEblock = new cODEGenericBody(ODEWorld);
    //cMultiMesh* object = new cMultiMesh();


    // load an object file
    bool fileload;
    //bool fileload1;
    //fileload = object->loadFromFile("L-block-2.obj");
    fileload = object->loadFromFile(RESOURCE_PATH("L-block3.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object->loadFromFile("L-block3.obj");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly" << endl;
        close();
        return (-1);
    }
    object->createAABBCollisionDetector(toolRadius);

/*
    // get dimensions of object
    object->computeBoundaryBox(true);
    double size = cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0.001)
    {
        object->scale(1.0 / size);
    }
*/

    cMaterial m;
    m.setStiffness(0.5*maxStiffness);
    m.setDynamicFriction(0.7);
    m.setStaticFriction(1.0);
    m.setBlueCadet();
    object->setMaterial(m, true);

    ODEBlock -> setImageModel(object);
    ODEBlock -> createDynamicMesh(true);

    ODEBlock -> setMass(0.03);



    // disable culling so that faces are rendered on both sides
    //object->setUseCulling(false);

    // compute a boundary box
    object->computeBoundaryBox(true);

    // show/hide boundary box
    object->setShowBoundaryBox(false);

    // compute collision detection algorithm
    //object->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    //object->setStiffness(0.5 * maxStiffness, true);

    // define some haptic friction properties
    //object->setFriction(0.0, 5.0, true);

    // enable display list for faster graphic rendering
    //object->setUseDisplayList(true);

    // center object in scene
    object->setLocalPos(-3.0 * object->getBoundaryCenter());

    // rotate object in scene
    object->rotateExtrinsicEulerAnglesDeg(0, 90, 0, C_EULER_ORDER_XYZ);


    // compute all edges of object for which adjacent triangles have more than 40 degree angle
    //object->computeAllEdges(0);

    // set line width of edges and color
    cColorf colorEdges;
    colorEdges.setBlack();
    object->setEdgeProperties(1, colorEdges);

    // set normal properties for display
    cColorf colorNormals;
    colorNormals.setOrangeTomato();
    object->setNormalsProperties(0.01, colorNormals);

    // display options
    object->setShowTriangles(showTriangles);
    object->setShowEdges(showEdges);
    object->setShowNormals(showNormals);





    //========================================================================================================================================
    //TETRIS
    //===================================================
   /* object1->setMaterial(m);

    // disable culling so that faces are rendered on both sides
    object1->setUseCulling(false);

    // compute a boundary box
    object1->computeBoundaryBox(true);

    // show/hide boundary box
    object1->setShowBoundaryBox(false);

    // compute collision detection algorithm
    object1->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    object1->setStiffness(0.2 * maxStiffness, true);

    // define some haptic friction properties
    object1->setFriction(50.0, 0.0, true);

    // enable display list for faster graphic rendering
    object1->setUseDisplayList(true);

    // center object in scene
    object1->setLocalPos(-3.0 * object1->getBoundaryCenter());

    // rotate object in scene
    object1->rotateExtrinsicEulerAnglesDeg(0, 90, 0, C_EULER_ORDER_XYZ);


    // compute all edges of object for which adjacent triangles have more than 40 degree angle
    object1->computeAllEdges(0);

    // set line width of edges and color
    //cColorf colorEdges;
    colorEdges.setBlack();
    object1->setEdgeProperties(1, colorEdges);

    // set normal properties for display
   // cColorf colorNormals;
    colorNormals.setOrangeTomato();
    object1->setNormalsProperties(0.01, colorNormals);

    // display options
    object1->setShowTriangles(showTriangles);
    object1->setShowEdges(showEdges);
    object1->setShowNormals(showNormals);
    */


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(0.95f, 0.95f, 0.95f),
                                cColorf(0.95f, 0.95f, 0.95f),
                                cColorf(0.80f, 0.80f, 0.80f),
                                cColorf(0.80f, 0.80f, 0.80f));


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    /*else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - show/hide texture
    else if (a_key == GLFW_KEY_1)
    {
        bool useTexture = object->getUseTexture();
        object->setUseTexture(!useTexture);
    }

    // option - enable/disable wire mode
    else if (a_key == GLFW_KEY_2)
    {
        bool useWireMode = object->getWireMode();
        object->setWireMode(!useWireMode, true);
    }

    // option - show/hide collision detection tree
    else if (a_key == GLFW_KEY_3)
    {
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        bool show = object->getShowCollisionDetector();
        object->setShowCollisionDetector(!show, true);
    }

    // option - decrease depth level of collision tree
    else if (a_key == GLFW_KEY_4)
    {
        collisionTreeDisplayLevel--;
        if (collisionTreeDisplayLevel < 0) { collisionTreeDisplayLevel = 0; }
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        object->setShowCollisionDetector(true, true);
    }

    // option - increase depth level of collision tree
    else if (a_key == GLFW_KEY_5)
    {
        collisionTreeDisplayLevel++;
        cColorf color = cColorf(1.0, 0.0, 0.0);
        object->setCollisionDetectorProperties(collisionTreeDisplayLevel, color, true);
        object->setShowCollisionDetector(true, true);
    }

    // option - save screenshot to file
    else if (a_key == GLFW_KEY_S)
    {
        cImagePtr image = cImage::create();
        camera->copyImageBuffer(image);
        image->saveToFile("screenshot.png");
        cout << "> Saved screenshot to file.       \r";
    }

    // option - show/hide triangles
    else if (a_key == GLFW_KEY_T)
    {
        showTriangles = !showTriangles;
        object->setShowTriangles(showTriangles);
    }

    // option - show/hide edges
    else if (a_key == GLFW_KEY_E)
    {
        showEdges = !showEdges;
        object->setShowEdges(showEdges);
    }

    // option - show/hide normals
    else if (a_key == GLFW_KEY_N)
    {
        showNormals = !showNormals;
        object->setShowNormals(showNormals);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }*/
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    cVector3d position;
    cVector3d axis1;
    double angle1;
    hapticDevice->getPosition(position);
    cTransform rotation;
  //  qDebug() << "Widget" << widget << "at position" << widget->pos();
    hapticDevice->getTransform(rotation);
  //  CHAI_DEBUG_PRINT("%s\n",rotation.getLocalRot());

    rotation.getLocalRot().toAxisAngle(axis1, angle1);

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz / x: " +
        cStr(position.x()) + " y: " + cStr(position.y()) + " z: " +
        cStr(position.z())+ "/ rotation: " +
        cStr(angle1*(180/M_PI)) + " | rot-x: " + cStr(axis1.x()) + " rot-y: " + cStr(axis1.y()) + " rot-z: " + cStr(axis1.z()));

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

/*enum cMode
{
    IDLE,
    SELECTION
};*/

void updateHaptics(void)
{
    //cMode state = IDLE;
    //cGenericObject* selectedObject = NULL;
    //cTransform tool_T_object;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        //////////////////////////////////////77
        // sim time
        /////////////////////////////////////7
        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();


        /////////////////////////////////////////////////////////////////////////
        // MANIPULATIONcMultiMesh* object = new cMultiMesh();
        /////////////////////////////////////////////////////////////////////////

        // compute transformation from world to tool (haptic device)
        //cTransform world_T_tool = tool->getDeviceGlobalTransform();

        // get status of user switch
        //bool button = tool->getUserSwitch(0);

        //
        // STATE 1: ONLY CLICK
        // Idle mode - user presses the user switch
        //
        /*if ((state == IDLE) && (button == true))
        {
            // check if at least one contact has occurred
            if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
            {
                // get contact event
                cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

                // get object from contact event
                selectedObject = collisionEvent->m_object;

                //tool->setDeviceGlobalForce(0.0,0.0,-50.82);
            }
            else
            {
                cout << "DED?" << endl;
                selectedObject = object;
            }

            // get transformation from object
            cTransform world_T_object = selectedObject->getGlobalTransform();

            // compute inverse transformation from contact point to object
            cTransform tool_T_world = world_T_tool;
            tool_T_world.invert();

            // store current transformation tool
            tool_T_object = tool_T_world * world_T_object;

            // update state
            state = SELECTION;
        }


        //
        // STATE 2: CLICK & HOLD
        // Selection mode - operator maintains user switch enabled and moves object
        //
        else if ((state == SELECTION) && (button == true))
        {
            // compute new transformation of object in global coordinates
            cTransform world_T_object = world_T_tool * tool_T_object;

            // compute new transformation of object in local coordinates
            cTransform parent_T_world = selectedObject->getParent()->getLocalTransform();
            //parent_T_world.setLocalRot(cMatrix3d (1, 0, 0, 0, 1, 0, 0, 0, 1));
            parent_T_world.invert();
            cTransform parent_T_object = parent_T_world * world_T_object;

            // ROTATIONAL MATRICES
            cMatrix3d yRot (cos(M_PI/2), 0, -sin(M_PI/2), 0, 1, 0, sin(M_PI/2), 0, cos(M_PI/2));

            //parent_T_object.setLocalRot(cMatrix3d (1, 0, 0, 0, 1, 0, 0, 0, 1));

            // assign new local transformation to object
            selectedObject->setLocalTransform(parent_T_object);

            // set zero forces when manipulating objects
            //tool->setDeviceGlobalForce(0.0, 0.0, -9.82);



            tool->initialize();
        }

        //
        // STATE 3:
        // Finalize Selection mode - operator releases user switch.
        //
        else
        {
            state = IDLE;
        }
        */
        if (graspActive)
                {
                    // check if button pressed
                    if (tool->getUserSwitch(0))
                    {
                        // position grasp point in world coordinates
                        //cVector3d posA = graspObject->getLocalTransform() * graspPos;
                        cVector3d posA = graspObject->getLocalTransform() * graspPos;

                        // position of tool
                        cVector3d posB = tool->getHapticPoint(0)->getGlobalPosGoal();

                        // update line
                        graspLine->m_pointA = posA;
                        graspLine->m_pointB = posB;
                        graspLine->setShowEnabled(true);

                        // compute force
                        cVector3d force = 1.0 * (posB - posA);

                        // apply force
                        graspObject->addExternalForceAtPoint(force, posA);

                        // apply reaction force to haptic device
                        tool->addDeviceGlobalForce(-force);
                    }
                    else
                    {
                        graspLine->setShowEnabled(false);
                        graspActive = false;
                        graspObject = NULL;
                    }
                }
        else
            {
                // get pointer to next interaction point of tool
                cHapticPoint* interactionPoint = tool->getHapticPoint(0);

                // check primary contact point if available
                if (interactionPoint->getNumCollisionEvents() > 0)
                {
                    cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

                    // given the mesh object we may be touching, we search for its owner which
                    // could be the mesh itself or a multi-mesh object. Once the owner found, we
                    // look for the parent that will point to the ODE object itself.
                    cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                    // cast to ODE object
                    cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                    // if ODE object, we apply interaction forces
                    if (ODEobject != NULL)
                    {
                        ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                            collisionEvent->m_globalPos);

                        // check if button pressed
                        if (tool->getUserSwitch(0))
                        {
                            graspObject = ODEobject;
                            graspPos = collisionEvent->m_localPos;
                            graspActive = true;
                        }
                    }
                }
            }
        ODEWorld -> updateDynamics(timeInterval);

        tool-> applyToDevice();
    }


        //WALL BOUNDARIES
        //Read the current position of the haptic device
       /* cVector3d newPosition;
        hapticDevice->getPosition(newPosition);

        cVector3d force(0,0,0);
        cVector3d proxyPos;

        if (newPosition.y() > boundaries.right){
            force.y(-1000*(newPosition.y()-boundaries.right));*/
            /*proxyPos.x(newPosition.x());
            proxyPos.y(boundaries.right);  //Proxy-failure /Spansk
            proxyPos.z(newPosition.z());
            tool->setLocalPos(proxyPos);*/
        /*}
        if (newPosition.y() < boundaries.left){
            force.y(-1000*(newPosition.y()-boundaries.left));
        }
        if (newPosition.z() > boundaries.top){
            force.z(-1000*(newPosition.z()-boundaries.top));
        }
        if (newPosition.z() < boundaries.bottom){
            force.z(-1000*(newPosition.z()-boundaries.bottom));
        }
        if (newPosition.x() < boundaries.back) {
            force.x(-1000*(newPosition.x()-boundaries.back));
        }
        if (newPosition.x() > boundaries.front) {
            force.x(-1000*(newPosition.x()-boundaries.front));
        }*/



        //tool->setDeviceGlobalForce(force);


        //ODEWorld->updateDynamics(timeInterval);
        /////////////////////////////////////////////////////////////////////////
        // FINALIZE
        /////////////////////////////////////////////////////////////////////////

        // send forces to haptic device
        //tool->applyToDevice();


    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------



//==============================================================================
/*
 *
 *  Example code borrowed from Chai3D library:

    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the followi== true))ng disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2049 $
*/
//==============================================================================







