
//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

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
    \version   3.2.0 $Rev: 2007 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"
#include "ode/ode.h"

#include "system/CGlobals.h"
#include "timers/CPrecisionClock.h"

#define _USE_MATH_DEFINES
//------------------------------------------------------------------------------

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

// tool radius
double toolRadius = 0.075;

int blockCounter = 1;

double spawnTime = 20;

double gameTimer = 0;

cPrecisionClock precisionClock;



//------------------------------------------------------------------------------
// CHAI3D VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;
cDirectionalLight *light2;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

//SCORE
cLabel* scoreText;

//Countdown
cLabel* timerText;

//end
cLabel* endText;

// is game ended?
bool gameEnded = false;

//------------------------------------------------------------------------------
// ODE MODULE VARIABLES
//------------------------------------------------------------------------------
 double cubeSize = 0.15;
// ODE world
cODEWorld* ODEWorld;



/*void cODEWorld::nearCallback (void *a_data, dGeomID a_object1, dGeomID a_object2)
{
    // retrieve body ID for each object. This value is defined unless the object
    // is static.
    dBodyID b1 = dGeomGetBody(a_object1);
    dBodyID b2 = dGeomGetBody(a_object2);

    // exit without doing anything if the two bodies are connected by a joint
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

    dContact contact[MAX_CONTACTS_PER_BODY];
    int n = dCollide (a_object1, a_object2, MAX_CONTACTS_PER_BODY,&(contact[0].geom),sizeof(dContact));
    if (n > 0)
    {
        for (int i=0; i<n; i++)
        {
            // define default collision properties (this section could be extended to support some ODE material class!)
            contact[i].surface.slip1 = 0.7;
            contact[i].surface.slip2 = 0.7;
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            contact[i].surface.mu = 50;
            contact[i].surface.soft_erp = 0.90;
            contact[i].surface.soft_cfm = 0.10;

            // get handles on each body
            cODEGenericBody* ode_body = NULL;
            if (b1 != NULL)
            {
                ode_body = (cODEGenericBody*)dBodyGetData (b1);
            }
            else if (b2 != NULL)
            {
                // if first object is static, use the second one. (both objects can not be static)
                ode_body = (cODEGenericBody*)dBodyGetData (b2);
            }

            // create a joint following collision
            if (ode_body != NULL)
            {
                dJointID c = dJointCreateContact (ode_body->m_ODEWorld->m_ode_world,
                                                    ode_body->m_ODEWorld->m_ode_contactgroup,
                                                    &contact[i]);
                dJointAttach (c,
                            dGeomGetBody(contact[i].geom.g1),
                            dGeomGetBody(contact[i].geom.g2));
            }

        }
    }
}*/

// ODE object
cODEGenericBody* ODEBodyT1;
cODEGenericBody* ODEBodyT2;
cODEGenericBody* ODEBodyZ1;
cODEGenericBody* ODEBodyZ2;
cODEGenericBody* ODEBodyO1;
cODEGenericBody* ODEBodyO2;
cODEGenericBody* ODEBodyS1;
cODEGenericBody* ODEBodyS2;
cODEGenericBody* ODEBodyL1;
cODEGenericBody* ODEBodyL2;
cODEGenericBody* ODEBodyJ1;
cODEGenericBody* ODEBodyJ2;
cODEGenericBody* ODEBodyI;
cODEGenericBody* ODEBodyGround;
cODEGenericBody* ODEBodyEndGame;


cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

// vector of block objects
vector<cODEGenericBody*> blocks;

//maxstiffness
double maxStiffness;


// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = true;

//
int clicked = 0;

// a small line used to display a grasp
cShapeLine* graspLine;

//Joints
dJointGroupID jointGroupIDT;
dJointID pointT;

dJointGroupID jointGroupIDZ;
dJointID pointZ;

dJointGroupID jointGroupIDO;
dJointID pointO;

dJointGroupID jointGroupIDS;
dJointID pointS;

dJointGroupID jointGroupIDL;
dJointID pointL;

dJointGroupID jointGroupIDJ;
dJointID pointJ;


//------------------------------------------------------------------------------
// GENERAL VARIABLES
//------------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


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

// this function renders the scene
void updateGraphics(void);


// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// create new block
void createNewBlock(vector<cODEGenericBody*>& blocks, double& maxStiffness);

// create block shape
void newBlockShape(char shape, double& sizeModifier1, double& sizeModifier2, double& yPosModifier, double& zPosModifier);

// The end-screen
void endGame();

//==============================================================================
/*
    DEMO:    ODE-tehtris.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 3
    spherical meshes that we individually attach to ODE bodies. Haptic interactions
    are computer by using the finger-proxy haptic model and forces are
    propagated to the ODE representation.
 */
//==============================================================================




int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    //Random generator fix
    srand(time(0));

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 02-ODE-trickyTris" << endl;
    cout << "Copyfight 2017" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[g] - Enable/Disable gravity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
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

    // initialize GLEW library
#ifdef GLEW_VERSION
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
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set(cVector3d(3.0, 0.0, 0.3),    // camera position (eye)
                cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light sources
    light = new cSpotLight(world);
    light2 = new cDirectionalLight(world);

    // attach lights to camera
    world->addChild(light);
    //world->addChild(light2);

    // enable light sources
    light->setEnabled(true);
    //light2->setEnabled(true);

    // position the light source
    light->setLocalPos( 0, 0, 1.2);

    // define the direction of the light beams
    light->setDir(0,0,-1.0);
    //light2->setDir(-2,0,0);

    // set uniform concentration level of light
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    shared_ptr<cGenericHapticDevice> hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // emulate button if device has a force gripper
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the tool (graphical display)
    //double toolRadius = 0.075;
    tool->setRadius(toolRadius, toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    //TEXT
    cFontPtr betterFont = NEW_CFONTCALIBRI20();

    //end it bitch
    cFontPtr bestFont = NEW_CFONTCALIBRI72();

    scoreText = new cLabel(betterFont);
    timerText = new cLabel(betterFont);
    endText = new cLabel(bestFont);

    scoreText->m_fontColor.setBlack();
    timerText->m_fontColor.setBlack();
    endText->m_fontColor.setBlack();


    camera->m_frontLayer->addChild(scoreText);
    camera->m_frontLayer->addChild(timerText);
    camera->m_frontLayer->addChild(endText);

    //--------------------------------------------------------------------------
    // CREATE ODE WORLD AND OBJECTS
    //--------------------------------------------------------------------------

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    cout << "MAX STIFLER: " << maxStiffness << endl;

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.1);
    ODEWorld->setLinearDamping(0.1);


    //////////////////////////////////////////////////////////////////////////
    // 3 ODE SPHERES
    //////////////////////////////////////////////////////////////////////////
    double cubeSize = 0.15;


    // create three ODE objects that are automatically added to the ODE world
   /* ODEBodyT1 = new cODEGenericBody(ODEWorld);
    ODEBodyT2 = new cODEGenericBody(ODEWorld);

    ODEBodyZ1 = new cODEGenericBody(ODEWorld);
    ODEBodyZ2 = new cODEGenericBody(ODEWorld);

    ODEBodyO1 = new cODEGenericBody(ODEWorld);
    ODEBodyO2 = new cODEGenericBody(ODEWorld);

    ODEBodyS1 = new cODEGenericBody(ODEWorld);
    ODEBodyS2 = new cODEGenericBody(ODEWorld);

    ODEBodyL1 = new cODEGenericBody(ODEWorld);
    ODEBodyL2 = new cODEGenericBody(ODEWorld);

    ODEBodyJ1 = new cODEGenericBody(ODEWorld);
    ODEBodyJ2 = new cODEGenericBody(ODEWorld);

    ODEBodyI = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMesh* objectT1 = new cMesh();
    cMesh* objectT2 = new cMesh();
    cMesh* objectZ1 = new cMesh();
    cMesh* objectZ2 = new cMesh();
    cMesh* objectO1 = new cMesh();
    cMesh* objectO2 = new cMesh();
    cMesh* objectS1 = new cMesh();
    cMesh* objectS2 = new cMesh();
    cMesh* objectL1 = new cMesh();
    cMesh* objectL2 = new cMesh();
    cMesh* objectJ1 = new cMesh();
    cMesh* objectJ2 = new cMesh();
    cMesh* objectI = new cMesh();


    cCreateBox(objectT1, cubeSize, cubeSize*3, cubeSize);
    cCreateBox(objectT2, cubeSize, cubeSize, cubeSize);
    cCreateBox(objectZ1, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectZ2, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectO1, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectO2, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectS1, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectS2, cubeSize, cubeSize*2, cubeSize);
    cCreateBox(objectL1, cubeSize, cubeSize, cubeSize*3);
    cCreateBox(objectL2, cubeSize, cubeSize, cubeSize);
    cCreateBox(objectJ1, cubeSize, cubeSize, cubeSize*3);
    cCreateBox(objectJ2, cubeSize, cubeSize, cubeSize);
    cCreateBox(objectI, cubeSize, cubeSize, cubeSize*4);

    // define some material properties for each sphere
    cMaterial mat;
    mat.setStiffness(0.5 * maxStiffness);
    mat.setDynamicFriction(5.0);
    mat.setStaticFriction(5.0);
    mat.setBlue();
    objectT1->setMaterial(mat, true);
    objectT2->setMaterial(mat, true);
    objectZ1->setMaterial(mat, true);
    objectZ2->setMaterial(mat, true);
    objectO1->setMaterial(mat, true);
    objectO2->setMaterial(mat, true);
    objectS1->setMaterial(mat, true);
    objectS2->setMaterial(mat, true);
    objectL1->setMaterial(mat, true);
    objectL2->setMaterial(mat, true);
    objectJ1->setMaterial(mat, true);
    objectJ2->setMaterial(mat, true);
    objectI->setMaterial(mat, true);

    cout << "BBBBBBBBBBBBBBBBBBBBBB" << endl;

    // add mesh model to ODE object
    ODEBodyT1->setImageModel(objectT1);
    ODEBodyT2->setImageModel(objectT2);
    ODEBodyZ1->setImageModel(objectZ1);
    ODEBodyZ2->setImageModel(objectZ2);
    ODEBodyO1->setImageModel(objectO1);
    ODEBodyO2->setImageModel(objectO2);
    ODEBodyS1->setImageModel(objectS1);
    ODEBodyS2->setImageModel(objectS2);
    ODEBodyL1->setImageModel(objectL1);
    ODEBodyL2->setImageModel(objectL2);
    ODEBodyJ1->setImageModel(objectJ1);
    ODEBodyJ2->setImageModel(objectJ2);
    ODEBodyI->setImageModel(objectI);

    cout << "AAAAAAAAAAAAAAAAAAAAAA" << endl;

    objectT1->createAABBCollisionDetector(toolRadius);
    objectT2->createAABBCollisionDetector(toolRadius);
    objectZ1->createAABBCollisionDetector(toolRadius);
    objectZ2->createAABBCollisionDetector(toolRadius);
    objectO1->createAABBCollisionDetector(toolRadius);
    objectO2->createAABBCollisionDetector(toolRadius);
    objectS1->createAABBCollisionDetector(toolRadius);
    objectS2->createAABBCollisionDetector(toolRadius);
    objectL1->createAABBCollisionDetector(toolRadius);
    objectL2->createAABBCollisionDetector(toolRadius);
    objectJ1->createAABBCollisionDetector(toolRadius);
    objectJ2-object>createAABBCollisionDetector(toolRadius);
    objectI->createAABBCollisionDetector(toolRadius);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    objectT1->computeBoundaryBox(true);
    double sphereRadius = 0.5 * (objectT1->getBoundaryMax().x() - objectT1->getBoundaryMin().x());

    ODEBodyT1->createDynamicBox(cubeSize, cubeSize*3, cubeSize);
    ODEBodyT2->createDynamicBox(cubeSize, cubeSize, cubeSize);

    ODEBodyZ1->createDynamicBox(cubeSize, cubeSize*2, cubeSize);
    ODEBodyZ2->createDynamicBox(cubeSize, cubeSize*2, cubeSize);

    ODEBodyO1->createDynamicBox(cubeSize, cubeSize*2, cubeSize);
    ODEBodyO2->createDynamicBox(cubeSize, cubeSize*2, cubeSize);

    ODEBodyS1->createDynamicBox(cubeSize, cubeSize*2, cubeSize);
    ODEBodyS2->createDynamicBox(cubeSize, cubeSize*2, cubeSize);

    ODEBodyL1->createDynamicBox(cubeSize, cubeSize, cubeSize*3);
    ODEBodyL2->createDynamicBox(cubeSize, cubeSize, cubeSize);

    ODEBodyJ1->createDynamicBox(cubeSize, cubeSize, cubeSize*3);
    ODEBodyJ2->createDynamicBox(cubeSize, cubeSize, cubeSize);

    ODEBodyI->createDynamicBox(cubeSize, cubeSize, cubeSize*4);


    int g = 1;
    // define some mass properties for each cube
    ODEBodyT1->setMass(0.015*g);
    ODEBodyT2->setMass(0.005*g);
    ODEBodyZ1->setMass(0.01*g);
    ODEBodyZ2->setMass(0.01*g);
    ODEBodyO1->setMass(0.01*g);
    ODEBodyO2->setMass(0.01*g);
    ODEBodyS1->setMass(0.01*g);
    ODEBodyS2->setMass(0.01*g);
    ODEBodyL1->setMass(0.015*g);
    ODEBodyL2->setMass(0.005*g);
    ODEBodyJ1->setMass(0.015*g);
    ODEBodyJ2->setMass(0.005*g);
    ODEBodyI->setMass(0.02*g);


    // set initial position of each cube
    ODEBodyT1->setLocalPos(0.0,0.0,0);
    ODEBodyT2->setLocalPos(0.0, 0.0,cubeSize);
    ODEBodyZ1->setLocalPos(0.0,cubeSize*1.5,cubeSize);
    ODEBodyZ2->setLocalPos(0.0, cubeSize*2.5,0.0);
    ODEBodyO1->setLocalPos(0.0,0.0,cubeSize);
    ODEBodyO2->setLocalPos(0.0, 0.0,0.0);
    ODEBodyS1->setLocalPos(0.0,-cubeSize*1.5,cubeSize);
    ODEBodyS2->setLocalPos(0.0,-cubeSize*2.5,0.0);
    ODEBodyL1->setLocalPos(0.0, 0.0,0.0);
    ODEBodyL2->setLocalPos(0.0,cubeSize,-cubeSize);
    ODEBodyJ1->setLocalPos(0.0, 0.0,0.0);
    ODEBodyJ2->setLocalPos(0.0,-cubeSize,-cubeSize);
    ODEBodyI->setLocalPos(0.0, cubeSize*4,0.0);*/



    // we create 6 static walls to contains the 3 spheres within a limited workspace
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    double groundLevel = -1.0;
    //ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  20 *cubeSize), cVector3d(0.0, 0.0 ,-1.0));
    //ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -1.0), cVector3d(0.0, 0.0 , 1.0));
    //ODEGPlane2->createStaticPlane(cVector3d(0.0, 5*cubeSize, 0.0), cVector3d(0.0,-1.0, 0.0));
    //ODEGPlane3->createStaticPlane(cVector3d( 0.0,-5.0*cubeSize, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d(cubeSize, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(0.0, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));

    /*ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -1.0), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0, size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d( 0.0,-size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d(cubeSize*1.01, 0.0, 0.0), cVector3d(groundLevel, 0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(0.0, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));*/

    cMesh* wallMesh = new cMesh();
    world->addChild(wallMesh);

    cMaterial wallMat;
    wallMat.setStiffness(0);
    wallMat.setDynamicFriction(0.0);
    wallMat.setStaticFriction(0.0);


    ODEGPlane1->setMaterial(wallMat);
    ODEGPlane2->setMaterial(wallMat);
    ODEGPlane3->setMaterial(wallMat);
    ODEGPlane4->setMaterial(wallMat);
    ODEGPlane5->setMaterial(wallMat);

    ODEGPlane4 -> setUseMaterial(true);
    ODEGPlane5 -> setUseMaterial(true);



/*
    ///////////////////////////////////////////////////////////
    /// Joints
    //////////////////////////////////////////////////////////
    jointGroupIDT = dJointGroupCreate(0);
    pointT = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDT);
    dJointAttach(pointT, ODEBodyT1->m_ode_body, ODEBodyT2->m_ode_body);
    dJointSetFixed(pointT);

    jointGroupIDZ = dJointGroupCreate(0);
    pointZ = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDZ);
    dJointAttach(pointZ, ODEBodyZ1->m_ode_body, ODEBodyZ2->m_ode_body);
    dJointSetFixed(poODEPlane1->createAABBCollisionDetector(toolRadius);intZ);

    jointGroupIDO = dJointGroupCreate(0);
    pointO = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDO);
    dJointAttach(pointO, ODEBODEBodyT1odyO1->m_ode_body, ODEBodyO2->m_ode_body);
    dJointSetFixed(pointO);

    jointGroupIDS = dJointGroupCreate(0);
    pointS = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDS);
    dJointAttach(pointS, ODEBodyS1->m_ode_body, ODEBodyS2->m_ode_body);
    dJointSetFixed(pointS);

    jointGroupIDL = dJointGroupCreate(0);
    pointL = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDL);
    dJointAttach(pointL, ODEBodyL1->m_ode_body, ODEBodyL2->m_ode_body);
    dJointSetFixed(pointL);

    jointGroupIDJ = dJointGroupCreate(0);
    pointJ = dJointCreateFixed(ODEWorld-> m_ode_world, jointGroupIDJ);
    dJointAttach(pointJ, ODEBodyJ1->m_ode_body, ODEBodyJ2->m_ode_body);
    dJointSetFixed(pointJ);*/



    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////
    ODEBodyGround = new cODEGenericBody(ODEWorld);
    cMesh* objectGround = new cMesh();
    cCreateBox(objectGround, cubeSize, cubeSize*6, cubeSize*2);
    cMaterial matGround;
    matGround.setStiffness(maxStiffness);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3);
    ODEBodyGround->setMaterial(matGround);
    ODEBodyGround->setImageModel(objectGround);
    ODEBodyGround->createDynamicBox(cubeSize, cubeSize*6, cubeSize*2,true);
    cout<<"hit kom vi"<<endl;
    ODEBodyGround->setLocalPos(cubeSize/2,0.0,-cubeSize*10);




    // create a mesh that represents the ground
    //cMesh* ground = new cMesh();
    //world->addChild(ground);

    // create a plane
    double groundSize = 3.0;
    //cCreatePlane(ground, groundSize, groundSize);

    // position ground in world where the invisible ODE plane is located (ODEGPlane1)
    //ground->setLocalPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    //cMaterial matGround;
    //matGround.setStiffness(maxStiffness);
    //matGround.setDynamicFriction(0.2);
    //matGround.setStaticFriction(0.0);
    //matGround.setWhite();
    //matGround.m_emission.setGrayLevel(0.3);
    //ground->setMaterial(matGround);

    // setup collision detector
    //ground->createAABBCollisionDetector(toolRadius);

    createNewBlock(blocks, maxStiffness);

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
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
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
    }
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

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);



    if (gameEnded) {
        scoreText->setText(" ");
        timerText->setText(" ");
    } else {

        timerText->setText("Next block in: "+cStr(spawnTime-gameTimer, 0)+" seconds!");

        // update position of label
        timerText->setLocalPos((int)(0.1*(width - timerText->getWidth())), height-50);
        scoreText->setText("You got "+cStr(blockCounter)+" pointers!");

        // update position of label
        scoreText->setLocalPos((int)(0.1*(width - timerText->getWidth())), height-100);
    }



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

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    gameTimer = precisionClock.start();


    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d newPosition;

        cVector3d desiredPosition;
        newPosition = tool->getHapticPoint(0)->getGlobalPosGoal();
        desiredPosition.set(toolRadius, newPosition.y(), newPosition.z());
        // variables for forces
        cVector3d force (0,0,0);
        // compute linear force
        double Kp = 25; // [N/m]
        cVector3d forceField = Kp * (desiredPosition - newPosition);
        force.add(forceField);



        // send computed force, torque, and gripper force to haptic device

        tool->addDeviceGlobalForce(force);

        // *****************************************************************
        /*cVector3d newPosition;


        newPosition = tool->getHapticPoint(0)->getGlobalPosGoal();
        cout<<"enter the if"<<endl;
        cVector3d force(0,0,0);
        force = cVector3d(0,0,0);

        if((newPosition.x()-toolRadius-0.01)<0.0){
            force.x(-10*(newPosition.x()));
            cVector3d proxyPos;
            proxyPos.x(toolRadius);
            proxyPos.y(newPosition.y());
            proxyPos.z(newPosition.z());
            tool->setDeviceLocalPos(proxyPos);

        }

        else if ((newPosition.x()+toolRadius+0.01)>cubeSize){
            force.x(-10*(newPosition.x()-cubeSize));
            cVector3d proxyPos;
            proxyPos.x(toolRadius);
            proxyPos.y(newPosition.y());
            proxyPos.z(newPosition.z());
            tool->setDeviceLocalPos(proxyPos);
        }

        else {
            cout << "x: " << newPosition.x() << " | Force: " << force.x() << ", " << force.y() << ", " << force.z() << endl;
        }



        tool->addDeviceGlobalForce(force);*/

        //cout << "x: " << newPosition.x() << " | Force: " << force.x() << ", " << force.y() << ", " << force.z() << endl;

        //if (graspActive)
        //{
        gameTimer = precisionClock.getCurrentTimeSeconds();
        //cout<<gameTimer<<endl;

        if (gameTimer > spawnTime){
            createNewBlock(blocks, maxStiffness);
            if (spawnTime > 3)
                spawnTime = spawnTime - 1;
            precisionClock.reset();
            precisionClock.start();
            blockCounter++;
            cout <<"Poäng:" << blockCounter << endl;
        }


           /* if (spawnTime < 0.2) {
                spawnTime = spawnTime + clock.getCurrentTimeSeconds();
            } else {
                spawnTime = 0;
                //blocks.end()[-1]->setMass(1);
                //blocks.end()[-2]->setMass(1);
                createNewBlock(blocks, maxStiffness);


            }*/
            //cout << "What is time? I don't know. Time is... " << spawnTime << endl;
            // check if button pressed
            if (tool->getUserSwitch(1) && clicked == 0)
            {
                //cout << "Spawning new block" << endl;
                //clicked = 1;
                //blocks.end()[-1]->setMass(1);
                //blocks.end()[-2]->setMass(1);
                //createNewBlock(blocks, maxStiffness);
                //blockCounter++;
                //cout <<"Poäng:" << blockCounter << endl;


                /*
                // position grasp point in world coordinates
                //cVector3d posA = graspObject->getLocalTransform() * graspPos;
                cVector3d posA = graspObject->getLocalTransform() * graspPos;

                // position of tool
                cVector3d posB = tool->getHapticPoint(0)->getGlobalPosGoal();
 double cubeSize = 0.15;
                // update line
                graspLine->m_pointA = posA;
                graspLine->m_pointB = posB;
                graspLine->setShowEnabled(true);

                // compute force
                cVector3d force = 1.0 * (posB - posA);

                // apply force
                graspObject->addExternalForceAtPoint(force, posA);

                // apply reaction force to haptic device
                tool->addDeviceGlobalForce(-force);*/
            }
            else if(tool->getUserSwitch(1)==false)
            {
                graspLine->setShowEnabled(false);
               // graspActive = false;
                graspObject = NULL;
                clicked = 0;
            }
            //skapa count=0
            // if count 0 och click --> spawn block --> set count ++1
            // else (count > 0 ) --> count = 0
        //}
        //else
        //{
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(0);


            // check primary contact point if available
            if (interactionPoint->getNumCollisionEvents() > 0)

            {
                int numEvents = interactionPoint->getNumCollisionEvents();
                //cout << numEvents << endl;
                for (int i = 0 ; i < numEvents; i++){

                    cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

                    // given the mesh object we may be touching, we search for its owner which
                    // could be the mesh itself or a multi-mesh object. Once the owner found, we
                    // look for the parent that will point to the ODE object itself.
                    cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                    //cout << "This is what we collide into: " << collisionEvent->m_object << endl;

                    // cast to ODE object
                    cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                    // if ODE object, we apply interaction forces
                    if (ODEobject != NULL)
                    {

                        if (ODEobject == blocks.end()[-1]|| ODEobject == blocks.end()[-2]){
                            ODEobject->addExternalForceAtPoint(-10 * interactionPoint->getLastComputedForce(),
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
            }
        //}

        // update simulation
        ODEWorld->updateDynamics(timeInterval);

        // send forces to haptic device
        tool->applyToDevice();

        if (gameEnded == false) {
            for (int i = 0; i < blocks.size(); i++) {
                if (blocks[i]->getLocalPos().z() < -cubeSize*12) {

                    gameEnded = true;
                    endGame();

                    endText->setText(" GAME OVER SCORE IS. . " + cStr(blockCounter) + "  ");
                    // update position of label
                    endText->setLocalPos((int)(0.5*(width - endText->getWidth())), 0.5*(height - endText->getHeight()));


                }
            }
        }
    }




    // exit haptics thread
    simulationFinished = true;
}

// ------------------------Our functions-----------------------

//Create New Block
void createNewBlock(vector<cODEGenericBody*> &blocks, double &maxStiffness) {
    //bestäm om I-block eller nåt annat
    //annat: T, L-sort, Squiggly-sort, O
    //pos: Reverse eller inte, form
    cVector3d newPosition;
    vector<char> shapes = {'T', 'L', 'J', 'Z', 'S', 'O', 'I'};

    //Modifiers
    double sizeModifier1, sizeModifier2, yPosModifier, zPosModifier;

    //Color
    //TBA
    int shapeIndex = rand() % 7;
    newBlockShape(shapes[shapeIndex], sizeModifier1, sizeModifier2, yPosModifier, zPosModifier);

    cODEGenericBody* newBlockPart1;
    cODEGenericBody* newBlockPart2;

    dJointGroupID testJointGroupIDZ;
    dJointID testPointZ;

    newBlockPart1 = new cODEGenericBody(ODEWorld);
    newBlockPart2 = new cODEGenericBody(ODEWorld);

    cMesh* testObjectZ1 = new cMesh();
    cMesh* testObjectZ2 = new cMesh();


    ////////////////
    cCreateBox(testObjectZ1, cubeSize, cubeSize*sizeModifier1, cubeSize);
    cCreateBox(testObjectZ2, cubeSize, cubeSize*sizeModifier2, cubeSize);
    ////////////////

    cMaterial mat;
    mat.setStiffness(0.5 * maxStiffness);
    mat.setDynamicFriction(1.0);
    mat.setStaticFriction(1.0);
    if (shapes[shapeIndex] == 'T') {
        mat.setPurple();
    }
    if (shapes[shapeIndex] == 'L') {
        mat.setOrange();
    }
    if (shapes[shapeIndex] == 'J') {
        mat.setBlue();
    }
    if (shapes[shapeIndex] == 'Z') {
        mat.setRed();
    }
    if (shapes[shapeIndex] == 'S') {
        mat.setGreen();
    }
    if (shapes[shapeIndex] == 'O') {
        mat.setYellow();
    }
    if (shapes[shapeIndex] == 'I') {
        mat.setBlueTurquoise();
    }
    //GLfloat grayLevel[] = {0.2f, 0.2f, 0.2f, 1.0f};
    //mat.setGrayLevel(grayLevel);
    //mat.updateColors();
    testObjectZ1->setMaterial(mat, true);
    testObjectZ2->setMaterial(mat, true);


    newBlockPart1->setImageModel(testObjectZ1);
    newBlockPart2->setImageModel(testObjectZ2);


    testObjectZ1->createAABBCollisionDetector(toolRadius);
    testObjectZ2->createAABBCollisionDetector(toolRadius);

    //////////////////
    newBlockPart1->createDynamicBox(cubeSize, cubeSize*sizeModifier1, cubeSize);
    newBlockPart2->createDynamicBox(cubeSize, cubeSize*sizeModifier2, cubeSize);
    //////////////////
    if (shapes[shapeIndex] == 'I') {
        sizeModifier1 = 2.0;
        sizeModifier2 = 2.0;
    }


    newBlockPart1->setMass(1*sizeModifier1*0.25);
    newBlockPart2->setMass(1*sizeModifier2*0.25);

    //////////////////
    newBlockPart1->setLocalPos(0.0,cubeSize*yPosModifier,(cubeSize*zPosModifier)+(cubeSize*3));
    newBlockPart2->setLocalPos(0.0, 0.0,0.0+(cubeSize*3));
    //////////////////
    testJointGroupIDZ = dJointGroupCreate(0);
    testPointZ = dJointCreateFixed(ODEWorld-> m_ode_world, testJointGroupIDZ);
    dJointAttach(testPointZ, newBlockPart1->m_ode_body, newBlockPart2->m_ode_body);
    dJointSetFixed(testPointZ);

    blocks.push_back(newBlockPart1);
    blocks.push_back(newBlockPart2);

    cout << "Function is active" << endl;
}

//New Block Shape
void newBlockShape(char shape, double& sizeModifier1, double& sizeModifier2, double& yPosModifier, double& zPosModifier) {
    if (shape == 'I') {
        cout << "is I" << endl;
        zPosModifier = 0.0;
        yPosModifier = 0.0;
        sizeModifier1 = 4.0;
        sizeModifier2 = 4.0;
    }
    else{
        zPosModifier = 1.0;
        if(shape == 'T'||shape == 'L'||shape == 'J'){
            sizeModifier1 = 1.0;
            sizeModifier2 = 3.0;

            if(shape == 'T'){
                yPosModifier = 0.0;
            }
            else if (shape == 'L'){
                yPosModifier = 1.0;
            }
            else if (shape == 'J'){
                yPosModifier = -1.0;
            }
        }
        else{
            sizeModifier1 = 2.0;
            sizeModifier2 = 2.0;
            if (shape == 'O'){
                yPosModifier = 0.0;
            }
            else if (shape == 'Z'){
                yPosModifier = -1.0;
            }
            else if (shape == 'S'){
                yPosModifier = 1.0;
            }
        }
    }
    cout << "Created " << shape << " block." << endl;


}


void endGame() {
    //ODEBodyEndGame = new cODEGenericBody(ODEWorld);
    //ODEBodyEndGame->createStaticPlane(cVector3d(cubeSize*10, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));

    cMesh* endPlane = new cMesh();
    world->addChild(endPlane);

    // create a plane
    double endSize = 100.0;
    cMatrix3d rot (cos(M_PI/4),0.0,-sin(M_PI/4), 0.0,1.0,0.0, sin(M_PI/4),0.0,cos(M_PI/4));
    cCreatePlane(endPlane, endSize, endSize, cVector3d(2,0,0), rot);

    cMaterial endMaterial;

    endMaterial.setWhite();
    endMaterial.m_emission.setGrayLevel(1);
    endPlane->setMaterial(endMaterial);


    // position ground in world where the invisible ODE plane is located (ODEGPlane1)





    /*
    cMesh* objectEndGame = new cMesh();
    cCreateBox(objectEndGame, 100, 100, 100);
    cMaterial matEndGame;
    matEndGame.setStiffness(maxStiffness);
    matEndGame.setDynamicFriction(0.2);
    matEndGame.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3);
    ODEBodyGround->setMaterial(matGround);
    ODEBodyGround->setImageModel(objectGround);
    ODEBodyGround->createDynamicBox(cubeSize, cubeSize*6, cubeSize*2,true);
    cout<<"hit kom vi"<<endl;
    ODEBodyGround->setLocalPos(cubeSize/2,0.0,-cubeSize*10);*/
}

//------------------------------------------------------------------------------
