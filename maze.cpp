//------------------------------------------------------------------------------
#include "chai3d.h"
#include "math.h"
#include <chrono>
//    cMultiMesh::getMesh();
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"

#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
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


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// stiffness of virtual spring
double linGain = 0.2;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 800;
double angStiffness = 30;

//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE objects
cODEGenericBody* ODEMaze;
cODEGenericBody* ODETool;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// transparency level
double transparencyLevel = 0.3;

cMultiMesh* imgMaze;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())
#define PI 3.14159265358979
#define sensitivity 500
#define speed 0.0002
#define ZERO 0.0


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

cVector3d getRotationAngles(cVector3d direction);

// main haptics simulation loop
void updateHaptics(void);


//==============================================================================
/*
  maze.cpp
 
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
    cout << "DH2626 - Lab 2" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Texture   (ON/OFF)" << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[3] - Collision tree (ON/OFF)" << endl;
    cout << "[+] - Increase collision tree display depth" << endl;
    cout << "[-] - Decrease collision tree display depth" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;
    
    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);
    
    
    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------
    
    // initialize GLUT
    glutInit(&argc, argv);
    
    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.9 * screenW;
    windowH = 0.9 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;
    
    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    }
    
    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);
    glewInit();
    
    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");
    
    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }
    

    
    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------
    
    // create a new world.
    world = new cWorld();
    
    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.0, 0.0, 0.0);
    
    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);
    
    // position and oriente the camera
    camera->set( cVector3d (2.0, 0.0, 0.2),    // camera position (eye)
                cVector3d (0.0, 0.0,-0.1),    // lookat position (target)
                cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
    
    
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
    
    // enable shadow casting
    camera->setUseShadowCasting(true);
    // create a light source
    light = new cSpotLight(world);
    
    // attach light to camera
    world->addChild(light);
    
    // enable light source
    light->setEnabled(true);
    
    // position the light source
    light->setLocalPos( 2.0, 2.0, 1.0);
    
    // define the direction of the light beam
    light->setDir(-2.0, -2.0,-1.0);
    
    // set uniform concentration level of light
    light->setSpotExponent(5.0);
    
    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);
    
    // set the resolution of the shadow map
    light->m_shadowMap->setResolutionLow();
    //light->m_shadowMap->setResolutionMedium();
    
    // set light cone half angle
    light->setCutOffAngleDeg(45);
    
        
    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------
    
    // create a haptic device handler
    handler = new cHapticDeviceHandler();
    
    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);
    
    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
    
    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    world->addChild(tool);
    
    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);
    
    tool->setWaitForSmallForce(false);
    
    //tool->m_hapticPoint->m_sphereProxy->m_material->setTransparencyLevel(0);
    
    // initialize tool by connecting to haptic device
    tool->start();
    
    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);
    
    // define a radius for the tool
    tool->setRadius(0.0);
    
    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(false, false);
    
    // start the haptic tool
    tool->start();
    
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------
    
    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);
    
    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);
    
    // set background properties
    background->setCornerColors(cColorf(1.0, 1.0, 1.0),
                                cColorf(1.0, 1.0, 1.0),
                                cColorf(0.8, 0.8, 0.8),
                                cColorf(0.8, 0.8, 0.8));
    
    
       
    //-----------------------------------------------------------------------
    // CREATE ODE WORLD AND OBJECTS
    //-----------------------------------------------------------------------
    
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
    ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
    
    // create a new ODE object that is automatically added to the ODE world
    ODEMaze = new cODEGenericBody(ODEWorld);
    
    // create a virtual mesh that will be used for the geometry
    // representation of the dynamic body
    imgMaze = new cMultiMesh();
    
    // load model
    bool fileload;
    fileload = imgMaze->loadFromFile(RESOURCE_PATH("resources/models/maze/pipe.obj"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = imgTeeth->loadFromFile("../../../bin/resources/models/maze/pipe.obj");
#endif
    }
    imgMaze->scale(0.04);
    imgMaze->createAABBCollisionDetector(0.0);
    
    cMaterial matMaze;
    matMaze.setStiffness(0.3 * maxStiffness);
    matMaze.setHapticTriangleSides(true, false);
    //imgMaze->getMesh(0);
    imgMaze->setMaterial(matMaze);
    
    // add mesh to ODE object
    ODEMaze->setImageModel(imgMaze);
    
    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEMaze->createDynamicMesh(true);
    
    // position and orient model
    ODEMaze->setLocalPos( -0.2, 0.0,0.0);
    ODEMaze->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 0);
    ODEMaze->rotateAboutGlobalAxisDeg(cVector3d(0,1,0), 90);
    ODEMaze->rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 90);
    
    // create a virtual tool
    ODETool = new cODEGenericBody(ODEWorld);
    cMesh* objectTool = new cMesh();
//    cCreateBox(objectTool, size, size, size);
    cCreateSphere(objectTool, 0.015);
    
    // define some material properties for each cube
    cMaterial matTool;
    matTool.m_ambient.set(0.4, 0.4, 0.4);
    matTool.m_diffuse.set(0.8, 0.8, 0.8);
    matTool.m_specular.set(1.0, 1.0, 1.0);
    matTool.setDynamicFriction(0.8);
    matTool.setStaticFriction(0.8);
    objectTool->setMaterial(matTool);
    objectTool->setTransparencyLevel(0.4);
    
    // add mesh to ODE object
    ODETool->setImageModel(objectTool);
    //ODETool->createDynamicBox(size, size, size);
    ODETool->createDynamicSphere(0.015);
    
    // define some mass properties for each cube
    ODETool->setMass(0.01);
    dBodySetAngularDamping(ODETool->m_ode_body, 0.04);
    dBodySetLinearDamping(ODETool->m_ode_body, 0.04);
    
    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    
    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    
    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();
    
    // close everything
    close();
    
    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------


void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();
        
        // exit application
        exit(0);
    }
    
    // help menu:
    if (key == 'h')
    {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;
        cout << "[3] - decrease linear haptic gain" << endl;
        cout << "[4] - increase linear haptic gain" << endl;
        cout << "[5] - decrease angular haptic gain" << endl;
        cout << "[6] - increase angular haptic gain" << endl  << endl;
        cout << "[7] - decrease linear stiffness" << endl;
        cout << "[8] - increase linear stiffness" << endl;
        cout << "[9] - decrease angular stiffness" << endl;
        cout << "[0] - increase angular stiffness" << endl << endl;
        cout << "[x] - Exit application\n" << endl;
        cout << endl << endl;
    }
    
    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }
    
    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
    
    // option 1:
    if (key == '1')
    {
        
        bool useWireMode = imgMaze->getMesh(1)->getWireMode();
        imgMaze->getMesh(1)->setWireMode(!useWireMode);
    }
    
    // option 2:
    if (key == '2')
    {
        bool useWireMode = imgMaze->getMesh(2)->getWireMode();
        imgMaze->getMesh(2)->setWireMode(!useWireMode);
    
    }
    
    // option 3: decrease linear haptic gain
    if (key == '3')
    {
        
        bool useWireMode = imgMaze->getMesh(3)->getWireMode();
        imgMaze->getMesh(3)->setWireMode(!useWireMode);
    
    }
    
    // option 4: increase linear haptic gain
    if (key == '4')
    {
        bool useWireMode = imgMaze->getMesh(4)->getWireMode();
        imgMaze->getMesh(4)->setWireMode(!useWireMode);
    }
    
    // option 5: decrease angular haptic gain
    if (key == '5')
    {
        bool useWireMode = imgMaze->getMesh(5)->getWireMode();
        imgMaze->getMesh(5)->setWireMode(!useWireMode);
    }
    
    // option 6: increase angular haptic gain
    if (key == '6')
    {
        angGain = angGain + 0.005;
        printf("angular haptic gain:  %f\n", angGain);
    }
    
    // option 7: decrease linear stiffness
    if (key == '7')
    {
        linStiffness = linStiffness - 50;
        if (linStiffness < 0)
            linStiffness = 0;
        printf("linear stiffness:  %f\n", linStiffness);
    }
    
    // option 8: increase linear stiffness
    if (key == '8')
    {
        linStiffness = linStiffness + 50;
        printf("linear stiffness:  %f\n", linStiffness);
    }
    
    // option 9: decrease angular stiffness
    if (key == '9')
    {
        angStiffness = angStiffness - 1;
        if (angStiffness < 0)
            angStiffness = 0;
        printf("angular stiffness:  %f\n", angStiffness);
    }
    
    // option 0: increase angular stiffness
    if (key == '0')
    {
        angStiffness = angStiffness + 1;
        printf("angular stiffness:  %f\n", angStiffness);
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
    hapticDevice->close();
}
//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
    
    glutTimerFunc(50, graphicsTimer, 0);
}


//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////
    
    // update haptic rate label
    labelHapticRate->setString ("haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]");
    
    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);
    
    
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////
    
    // render world
    camera->renderView(windowW, windowH);
    
    // swap buffers
    glutSwapBuffers();
    
    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
  
    camera->set( tool->getDeviceGlobalPos(),    // camera position (eye)
                cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

}

///---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    
    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);
    
    cMatrix3d prevRotTool;
    prevRotTool.identity();
    
    // main haptic simulation loop
    while(simulationRunning)
    {
        // update frequency counter
        frequencyCounter.signal(1);
        
        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = 0.0005;//cClamp(time, 0.00001, 0.0002);
        
        // reset clock
        simClock.reset();
        simClock.start();
        
        // compute global reference frames for each object
        world->computeGlobalPositions(true);
        
        // update position and orientation of tool
        tool->updatePose();
        
        // compute interaction forces
        tool->computeInteractionForces();
        
        // update position and orientation of tool
        cVector3d posDevice;
        cMatrix3d rotDevice;
        //hapticDevice->getPosition(posDevice);
        //hapticDevice->getRotation(rotDevice);
        posDevice = tool->m_hapticPoint->getGlobalPosProxy();
        rotDevice = tool->getDeviceGlobalRot();
        
        // read position of tool
        cVector3d posTool = ODETool->getLocalPos();
        cMatrix3d rotTool = ODETool->getLocalRot();
        
        // compute position and angular error between tool and haptic device
        cVector3d deltaPos = (posDevice - posTool);
        cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotDevice);
        double angle;
        cVector3d axis;
        deltaRot.toAxisAngle(axis, angle);
        
        // compute force and torque to apply to tool
        cVector3d force, torque;
        force = linStiffness * deltaPos;
        ODETool->addExternalForce(force);
        
        torque = cMul((angStiffness * angle), axis);
        rotTool.mul(torque);
        ODETool->addExternalTorque(torque);
        
        // compute force and torque to apply to haptic device
        force = -linG * force;
        torque = -angG * torque;
        
        // add force contribution from ODE model
        tool->m_lastComputedGlobalForce.add(force);
        
        // send forces to haptic device.
        tool->applyForces();
        //hapticDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
        
        if (linG < linGain)
        {
            linG = linG + 0.1 * time * linGain;
        }
        else
        {
            linG = linGain;
        }
        
        if (angG < angGain)
        {
            angG = angG + 0.1 * time * angGain;
        }
        else
        {
            angG = angGain;
        }
        
        // update simulation
        ODEWorld->updateDynamics(nextSimInterval);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

















