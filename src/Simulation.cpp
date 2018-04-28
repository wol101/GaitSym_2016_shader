/*
 *  Simulation.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Simulation.cpp - this simulation object is used to encapsulate
// a ODE simulation

#include "Simulation.h"

#include "Util.h"
#include "DebugControl.h"
#include "CyclicDriver.h"
#include "StepDriver.h"
#include "DataTarget.h"
#include "DataTargetScalar.h"
#include "DataTargetQuaternion.h"
#include "DataTargetVector.h"
#include "DataFile.h"
#include "PGDMath.h"
#include "Body.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "FloatingHingeJoint.h"
#include "CappedCylinderGeom.h"
#include "SphereGeom.h"
#include "Muscle.h"
#include "MAMuscle.h"
#include "MAMuscleExtended.h"
#include "MAMuscleComplete.h"
#include "UGMMuscle.h"
#include "DampedSpringMuscle.h"
#include "TwoPointStrap.h"
#include "ThreePointStrap.h"
#include "CylinderWrapStrap.h"
#include "TwoCylinderWrapStrap.h"
#include "Environment.h"
#include "PlaneGeom.h"
#include "Contact.h"
#include "ErrorHandler.h"
#include "NPointStrap.h"
#include "FixedJoint.h"
#include "TrimeshGeom.h"
#include "Marker.h"
#include "Reporter.h"
#include "TorqueReporter.h"
#include "PositionReporter.h"
#include "UniversalJoint.h"
#include "PIDMuscleLength.h"
#include "Controller.h"
#include "SwingClearanceAbortReporter.h"
#include "AMotorJoint.h"
#include "SliderJoint.h"
#include "BoxGeom.h"
#include "BoxCarDriver.h"
#include "StackedBoxCarDriver.h"
#include "PIDTargetMatch.h"
#include "Warehouse.h"
#include "FixedDriver.h"

#ifdef USE_QT
#include "GLUtils.h"
#include "FacetedObject.h"
static bool SortOnAlpha(FacetedObject* d1, FacetedObject* d2);
#endif

#if defined(USE_QT) // && !defined(USE_WI_BB) // this is a bit odd - I'm not sure why it is here in the USE_WI_BB version
#include "mainwindow.h"
#endif
#if defined(USE_QT)
#include <QColor>
#include "preferences.h"
#endif

#include <ode/ode.h>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <typeinfo>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <list>
#include <ctype.h>
#include <cmath>
#include <cmath>


extern char *gGraphicsRoot;
extern int gDisplaySkip;
extern bool gDelayedDraw;
extern std::list<FacetedObject *> gDelayedDrawList;

#define _I(i,j) I[(i)*4+(j)]

Simulation::Simulation()
{
    // allocate some general purpose memory
    // this is assumed to be big enough!
    m_BufferSize = 1000000;
    m_Buffer = new char[m_BufferSize];
    m_LargeBuffer = new char[m_BufferSize * 10];
    m_BufferPtrs = new char *[m_BufferSize];
    m_DoubleList = new double[m_BufferSize];

    // initialise the ODE world
    dInitODE();
    m_WorldID = dWorldCreate();
    m_SpaceID = dSimpleSpaceCreate(0);
    m_ContactGroup = dJointGroupCreate(0);

    m_Environment = new Environment();
    m_MaxContacts = 16;

    // set some variables
    m_SimulationTime = 0;
    m_StepCount = 0;
    m_StepSize = 0;
    m_CycleTime = -1;
    m_MechanicalEnergy = 0;
    m_MetabolicEnergy = 0;
    m_FitnessType = DistanceTravelled;
    m_DistanceTravelledBodyID = 0;
    m_BMR = 0;
    m_OutputModelStateAtTime = -1;
    m_OutputModelStateAtCycle = -1;
    m_TimeLimit = 0;
    m_MechanicalEnergyLimit = 0;
    m_MetabolicEnergyLimit = 0;
    m_InputKinematicsFlag = false;
    m_OutputKinematicsFlag = false;
    m_OutputWarehouseFlag = false;
    m_OutputModelStateFilename = "ModelState.xml";
    m_OutputKinematicsFilename = "Kinematics.txt";
    m_OutputWarehouseFilename = "Warehouse.txt";
    m_OutputModelStateOccured = false;
    m_AbortAfterModelStateOutput = false;
    m_KinematicMatchFitness = 0;
    m_MungeModelStateFlag = false;
    m_MungeRotationFlag = false;
    m_ModelStateRelative = true;
    m_AllowInternalCollisions = true;
    m_AllowConnectedCollisions = false;
    m_StepType = WorldStep;
    m_ContactAbort = false;
    m_SimulationError = 0;
    m_DataTargetAbort = false;
    m_AbortOnODEMessage = false;
    m_KinematicMatchMiniMaxFitness = 0;
    m_YUp = false;
    m_StraightenBody = false;
    m_ClosestWarehouseFitness = -DBL_MAX;
    m_WarehouseDistance = 0;
    m_WarehouseUnitIncreaseDistanceThreshold = 0;
    m_WarehouseDecreaseThresholdFactor = 0;
    m_OutputKinematicsFirstTimeFlag = true;
    m_OutputWarehouseLastTime = -DBL_MAX;
    m_OutputModelStateAtWarehouseDistance = 0;
    m_OutputWarehouseAsText = true;
    m_WarehouseFailDistanceAbort = 0;
    m_WarehouseUsePCA = true;

    // values for energy partition
    m_PositiveMechanicalWork = 0;
    m_NegativeMechanicalWork = 0;
    m_PositiveContractileWork = 0;
    m_NegativeContractileWork = 0;
    m_PositiveSerialElasticWork = 0;
    m_NegativeSerialElasticWork = 0;
    m_PositiveParallelElasticWork = 0;
    m_NegativeParallelElasticWork = 0;

    // format controls
    m_SanityCheckAxis = YAxis;

    dSetMessageHandler(ODEMessageTrap);

#ifdef USE_QT
    m_Interface.EnvironmentAxisSize[0] = m_Interface.EnvironmentAxisSize[1] = m_Interface.EnvironmentAxisSize[2] = 1.0;
    m_Interface.EnvironmentColour.SetColour(1, 0, 1, 1);
    m_Interface.BodyAxisSize[0] = m_Interface.BodyAxisSize[1] = m_Interface.BodyAxisSize[2] = 0.05;
    m_Interface.BodyColour.SetColour(0.275, 0.725, 0.451, 1.0);
    m_Interface.JointAxisSize[0] = m_Interface.JointAxisSize[1] = m_Interface.JointAxisSize[2] = 0.05;
    m_Interface.JointColour.SetColour(0, 1, 0, 1);
    m_Interface.GeomColour.SetColour(0, 0, 1, 0.5);
    m_Interface.StrapColour.SetColour(1, 0, 0, 1);
    m_Interface.StrapRadius = 0.005;
    m_Interface.StrapForceColour.SetColour(1, 0, 0, 0.5);
    m_Interface.StrapForceRadius = 0.02;
    m_Interface.StrapForceScale = 0.0001;
    m_Interface.StrapCylinderColour.SetColour(0, 1, 1, 0.5);
    m_Interface.StrapCylinderLength = 0.1;
    m_Interface.GeomColour.SetColour(0, 0, 1, 0.5);
    m_Interface.GeomAxisSize[0] = m_Interface.GeomAxisSize[1] = m_Interface.GeomAxisSize[2] = 0.05;
    m_Interface.GeomForceColour.SetColour(0, 0, 1, 0.5);
    m_Interface.GeomForceRadius = 0.02;
    m_Interface.GeomForceScale = 0.0001;
    m_Interface.ReporterColour.SetColour(1, 1, 0, 1);
    m_Interface.ReporterRadius = 0.05;
    m_Interface.DataTargetColour.SetColour(1, 0, 0.5, 1);
    m_Interface.DataTargetRadius = 0.05;
    m_Interface.MarkerColour.SetColour(1, 1, 0, 0.5);
    m_Interface.MarkerRadius = 0.03;
    m_Interface.TrackBodyID = "Torso";

    m_nextTextureID = 1;
    m_UseProgramInterface = true;
#endif
#ifdef USE_QT
    m_MainWindow = 0;
#endif

}

//----------------------------------------------------------------------------
Simulation::~Simulation()
{
    dSetMessageHandler(0);

    if (gDebug == EnergyPartitionDebug)
    {
        *gDebugStream << "m_PositiveMechanicalWork " << m_PositiveMechanicalWork <<
                " m_NegativeMechanicalWork " << m_NegativeMechanicalWork <<
                " m_PositiveContractileWork " << m_PositiveContractileWork <<
                " m_NegativeContractileWork " << m_NegativeContractileWork <<
                " m_PositiveSerialElasticWork " << m_PositiveSerialElasticWork <<
                " m_NegativeSerialElasticWork " << m_NegativeSerialElasticWork <<
                " m_PositiveParallelElasticWork " << m_PositiveParallelElasticWork <<
                " m_NegativeParallelElasticWork " << m_NegativeParallelElasticWork <<
                "\n";
    }

    // get rid of all those memory alloactions

    for (std::map<std::string, Body *>::const_iterator iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++) delete iter1->second;
    for (std::map<std::string, Joint *>::const_iterator iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++) delete iter2->second;
    for (std::map<std::string, Muscle *>::const_iterator iter3 = m_MuscleList.begin(); iter3 != m_MuscleList.end(); iter3++) delete iter3->second;
    for (std::map<std::string, Driver *>::const_iterator iter4 = m_DriverList.begin(); iter4 != m_DriverList.end(); iter4++) delete iter4->second;
    for (std::map<std::string, DataTarget *>::const_iterator iter5 = m_DataTargetList.begin(); iter5 != m_DataTargetList.end(); iter5++) delete iter5->second;
    for (std::map<std::string, Geom *>::const_iterator iter6 = m_GeomList.begin(); iter6 != m_GeomList.end(); iter6++) delete iter6->second;
    for (std::map<std::string, Marker *>::const_iterator iter7 = m_MarkerList.begin(); iter7 != m_MarkerList.end(); iter7++) delete iter7->second;
    for (std::map<std::string, Reporter *>::const_iterator iter8 = m_ReporterList.begin(); iter8 != m_ReporterList.end(); iter8++) delete iter8->second;
    for (std::map<std::string, Controller *>::const_iterator iter = m_ControllerList.begin(); iter != m_ControllerList.end(); iter++) delete iter->second;
    for (std::map<std::string, FixedJoint *>::const_iterator iter = m_JointStressList.begin(); iter != m_JointStressList.end(); iter++) delete iter->second;
    for (std::map<std::string, Warehouse *>::const_iterator iter = m_WarehouseList.begin(); iter != m_WarehouseList.end(); iter++) delete iter->second;

    delete m_Environment;

    // destroy the ODE world
#ifdef OPENGL
    for (unsigned int i = 0; i < m_PickGeomList.size(); i++) delete m_PickGeomList[i];
#endif
    dJointGroupDestroy(m_ContactGroup);
    dSpaceDestroy(m_SpaceID);
    dWorldDestroy(m_WorldID);
    dCloseODE();

    // clear the stored xml data
    std::vector<xmlNodePtr>::const_iterator iter0;
    for (iter0 = m_TagContentsList.begin(); iter0 != m_TagContentsList.end(); iter0++)
        xmlFreeNode(*iter0);

    // delete the rest of the allocated memory
    for (unsigned int c = 0; c < m_ContactList.size(); c++) delete m_ContactList[c];

    // close any open files
    if (m_OutputWarehouseFlag) m_OutputWarehouseFile.close();
    if (m_OutputKinematicsFlag) m_OutputKinematicsFile.close();

    delete [] m_Buffer;
    delete [] m_LargeBuffer;
    delete [] m_BufferPtrs;
    delete [] m_DoubleList;

}

//----------------------------------------------------------------------------
int Simulation::LoadModel(char *xmlDataBuffer)
{
    xmlDocPtr doc;
    xmlNodePtr cur, nodeCopy;
    int size = strlen(xmlDataBuffer);

    if (gDebug == SimulationDebug)
    {
        *gDebugStream << "Simulation::LoadModel " << size << "\n" <<
                xmlDataBuffer << "\n";
    }

    // do the basic XML parsing

    doc = xmlReadMemory(xmlDataBuffer, size, 0, 0, XML_PARSE_HUGE);

    if (doc == NULL )
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        m_MainWindow->log("Document not parsed successfully");
#endif
        fprintf(stderr,"Document not parsed successfully.\n");
        return 1;
    }

#if defined(USE_QT) && !defined(USE_WI_BB)
    m_MainWindow->log("Valid XML Document found");
#endif

    cur = xmlDocGetRootElement(doc);

    if (cur == NULL)
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        m_MainWindow->log("Document empty error");
#endif
        fprintf(stderr,"Empty document\n");
        xmlFreeDoc(doc);
        return 1;
    }

    if (xmlStrcmp(cur->name, (const xmlChar *) "GAITSYMODE"))
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        m_MainWindow->log("Document of the wrong type, root node != GAITSYMODE");
#endif
        fprintf(stderr,"Document of the wrong type, root node != GAITSYMODE\n");
        xmlFreeDoc(doc);
        return 1;
    }

    // now parse the elements in the file

    try
    {
        cur = cur->xmlChildrenNode;

        while (cur != NULL)
        {
            nodeCopy = xmlCopyNode(cur, 1);
            m_TagContentsList.push_back(nodeCopy);

            if (gDebug == XMLDebug)
            {
                *gDebugStream << "cur->name " << cur->name << "\n";
            }

            try
            {

                if ((!xmlStrcmp(cur->name, (const xmlChar *)"GLOBAL"))) ParseGlobal(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"ENVIRONMENT"))) ParseEnvironment(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"BODY"))) ParseBody(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"JOINT"))) ParseJoint(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"GEOM"))) ParseGeom(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"MUSCLE"))) ParseMuscle(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"DRIVER"))) ParseDriver(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"DATATARGET"))) ParseDataTarget(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"IOCONTROL"))) ParseIOControl(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"MARKER"))) ParseMarker(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"REPORTER"))) ParseReporter(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"CONTROLLER"))) ParseController(cur);
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"WAREHOUSE"))) ParseWarehouse(cur);
#ifdef USE_QT
                if ((!xmlStrcmp(cur->name, (const xmlChar *)"INTERFACE"))) ParseInterface(cur);
#endif
            }

            catch (int e)
            {
                std::cerr << "Error reading XML tag " << cur->name << " on line: " << cur->line << "\n";
#if defined(USE_QT) && !defined(USE_WI_BB)
                std::stringstream ss;
                ss << "Error reading XML tag " << cur->name << " on line: " << cur->line << "\n";
                m_MainWindow->log(ss.str().c_str());
                throw __LINE__;
#endif
            }

            cur = cur->next;
        }


        // and do the late initialisation
        std::map<std::string, Muscle *>::const_iterator iter2;
        for (iter2 = m_MuscleList.begin(); iter2 != m_MuscleList.end(); iter2++)
        {
            if (gDebug == XMLDebug)
            {
                *gDebugStream << iter2->first << " late initialisation\n";
            }
            iter2->second->LateInitialisation();
        }

        m_DistanceTravelledBodyID = m_BodyList[m_DistanceTravelledBodyIDName];
        if (m_DistanceTravelledBodyID == 0)
        {
#if defined(USE_QT) && !defined(USE_WI_BB)
            std::stringstream ss;
            ss << "Error parsing XML file: DistanceTravelledBodyIDName not found - \"" << m_DistanceTravelledBodyIDName << "\"";
            m_MainWindow->log(ss.str().c_str());
#endif
            throw __LINE__;
        }

        // for the time being just set the current warehouse to the first one in the list
        if (m_CurrentWarehouse.length() == 0 && m_WarehouseList.size() > 0) m_CurrentWarehouse = m_WarehouseList.begin()->first;

#if !defined(USE_MPI) && !defined(USE_GAUL)
        // left and right sanity checking
        if (m_SanityCheckLeft.size() != 0 && m_SanityCheckRight.size() != 0)
        {
            std::string rightSearch;
            int err;

            std::map<std::string, Body *>::const_iterator bodyLeft;
            for (bodyLeft = m_BodyList.begin(); bodyLeft != m_BodyList.end(); bodyLeft++)
            {
                if (bodyLeft->first.find(m_SanityCheckLeft) != std::string::npos)
                {
                    rightSearch = bodyLeft->first;
                    Util::FindAndReplace(&rightSearch, m_SanityCheckLeft, m_SanityCheckRight);
                    std::map<std::string, Body *>::const_iterator bodyRight = m_BodyList.find(rightSearch);
                    if (bodyRight != m_BodyList.end())
                    {
                        err = bodyLeft->second->SanityCheck(bodyRight->second, m_SanityCheckAxis, m_SanityCheckLeft, m_SanityCheckRight);
                        if (err)
                        {
#if defined(USE_QT) && !defined(USE_WI_BB)
                            std::stringstream ss;
                            ss << "Warning: possible LR sanity error in BODYs \"" << bodyLeft->first << "\"" << " and \"" << bodyRight->first << "\" Line " << err;
                            m_MainWindow->log(ss.str().c_str());
#endif
                            std::cerr << "Warning: possible LR sanity error in BODYs \"" << bodyLeft->first << "\"" << " and \"" << bodyRight->first << "\" Line " << err << "\n";
                        }
                    }
                }
            }

            std::map<std::string, Joint *>::const_iterator iter2;
            for (iter2=m_JointList.begin(); iter2 != m_JointList.end(); iter2++)
            {

            }

            std::map<std::string, Muscle *>::const_iterator muscleLeft;
            for (muscleLeft = m_MuscleList.begin(); muscleLeft != m_MuscleList.end(); muscleLeft++)
            {
                if (muscleLeft->first.find(m_SanityCheckLeft) != std::string::npos)
                {
                    rightSearch = muscleLeft->first;
                    Util::FindAndReplace(&rightSearch, m_SanityCheckLeft, m_SanityCheckRight);
                    std::map<std::string, Muscle *>::const_iterator muscleRight = m_MuscleList.find(rightSearch);
                    if (muscleRight != m_MuscleList.end())
                    {
                        err = muscleLeft->second->SanityCheck(muscleRight->second, m_SanityCheckAxis, m_SanityCheckLeft, m_SanityCheckRight);
                        if (err)
                        {
#if defined(USE_QT) && !defined(USE_WI_BB)
                            std::stringstream ss;
                            ss << "Warning: possible LR sanity error in MUSCLEs \"" << muscleLeft->first << "\"" << " and \"" << muscleRight->first << "\" Line " << err;
                            m_MainWindow->log(ss.str().c_str());
#endif
                            std::cerr << "Warning: possible LR sanity error in BODYs \"" << muscleLeft->first << "\"" << " and \"" << muscleRight->first << "\" Line " << err << "\n";
                        }
                    }
                }
            }

            std::map<std::string, Geom *>::const_iterator iter6;
            for (iter6 = m_GeomList.begin(); iter6 != m_GeomList.end(); iter6++)
            {

            }

        }
#endif
    }

    catch(int e)
    {
        if (cur)
        {
            std::cerr << e << " Error parsing XML file: " << cur->name << "\n" << m_Message << "\n";
#if defined(USE_QT) && !defined(USE_WI_BB)
            std::stringstream ss;
            ss << e << " Error parsing XML file: " << cur->name << " " << m_Message;
            m_MainWindow->log(ss.str().c_str());
#endif
    }
        else
        {
            std::cerr << __FILE__ << " " << e << " Error parsing XML file\n" << m_Message << "\n";
    #if defined(USE_QT) && !defined(USE_WI_BB)
            std::stringstream ss;
            ss << e << " Error parsing XML file " << m_Message << "\n";
            m_MainWindow->log(ss.str().c_str());
    #endif
        }
        xmlFreeDoc(doc);
        return 1;
    }

#ifdef OUTPUTS_AFTER_SIMULATION_STEP
    if (m_OutputModelStateAtTime == 0.0 || m_OutputModelStateAtCycle == 0)
    {
        OutputProgramState();
        m_OutputModelStateAtTime = -1.0;
        m_OutputModelStateAtCycle = -1;
    }
#endif

    xmlFreeDoc(doc);
    return 0;
}


//----------------------------------------------------------------------------
void Simulation::UpdateSimulation()
{
    // read in external kinematics if used
    if (m_InputKinematicsFlag)
    {
        InputKinematics();
        std::map<std::string, Muscle *>::const_iterator iter1;
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
            iter1->second->CalculateStrap(m_StepSize);
        m_StepCount++;
        return;
    }

    // calculate the warehouse and position matching fitnesses before we move to a new location
    if (m_FitnessType != DistanceTravelled)
    {
        if (m_FitnessType == ClosestWarehouse)
        {
        }
        else if (m_FitnessType == KinematicMatch || m_FitnessType == KinematicMatchMiniMax)
        {

            double minScore = DBL_MAX;
            double matchScore;
            std::map<std::string, DataTarget *>::const_iterator iter3;
            for (iter3=m_DataTargetList.begin(); iter3 != m_DataTargetList.end(); iter3++)
            {
                int lastIndex = iter3->second->GetLastMatchIndex();
                int index = iter3->second->TargetMatch(m_SimulationTime, m_StepSize * 0.50000000001);
                // on rare occasions because of rounding we may get two matches we can check this using the lastIndex since this is the only palce where a match is requested
                if (index != -1 && index != lastIndex) // since step size is much smaller than the interval between targets (probably), this should get called exactly once per target time defintion
                {
                    matchScore = iter3->second->GetMatchValue(index);
                    m_KinematicMatchFitness += matchScore;
                    if (matchScore < minScore)
                        minScore = matchScore;
                    if (gDebug == FitnessDebug) *gDebugStream <<
                                                                 "Simulation::UpdateSimulation m_SimulationTime " << m_SimulationTime <<
                                                                 " DataTarget->name " << *iter3->second->GetName() <<
                                                                 " matchScore " << matchScore <<
                                                                 " minScore " << minScore <<
                                                                 " m_KinematicMatchFitness " << m_KinematicMatchFitness << "\n";
                }
            }
            if (minScore < DBL_MAX)
                m_KinematicMatchMiniMaxFitness += minScore;
        }
    }

    // now start the actual simulation

    // check collisions first
    dJointGroupEmpty(m_ContactGroup);
    for (unsigned int c = 0; c < m_ContactList.size(); c++) delete m_ContactList[c];
    m_ContactList.clear();
    std::map<std::string, Geom *>::const_iterator GeomIter;
    for (GeomIter = m_GeomList.begin(); GeomIter != m_GeomList.end(); GeomIter++) GeomIter->second->ClearContacts();
    dSpaceCollide(m_SpaceID, this, &NearCallback);

    bool activationsDone = false;

//    if (activationsDone == false)
//    {
//        // update the driver based activations
//        double activation;
//        std::map<std::string, Driver *>::const_iterator iter2;
//        for (iter2 = m_DriverList.begin(); iter2 != m_DriverList.end(); iter2++)
//        {
//            activation = iter2->second->GetValue(m_SimulationTime);
//            iter2->second->GetTarget()->SetActivation(activation, m_StepSize);
//        }
//    }

    // update the muscles
    double tension;
    std::vector<PointForce *> *pointForceList;
    std::map<std::string, Muscle *>::const_iterator iter1;
    PointForce *pointForce;
    for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
    {
        if (activationsDone == false) iter1->second->SumDrivers(m_SimulationTime);
        iter1->second->SetActivation(iter1->second->GetCurrentDriverSum(), m_StepSize);
        iter1->second->CalculateStrap(m_StepSize);

        pointForceList = iter1->second->GetPointForceList();
        tension = iter1->second->GetTension();
#ifdef DEBUG_CHECK_FORCES
        pgd::Vector force(0, 0, 0);
#endif
        for (unsigned int i = 0; i < pointForceList->size(); i++)
        {
            pointForce = (*pointForceList)[i];
            dBodyAddForceAtPos(pointForce->body->GetBodyID(),
                               pointForce->vector[0] * tension, pointForce->vector[1] * tension, pointForce->vector[2] * tension,
                               pointForce->point[0], pointForce->point[1], pointForce->point[2]);
#ifdef DEBUG_CHECK_FORCES
            force += pgd::Vector(pointForce->vector[0] * tension, pointForce->vector[1] * tension, pointForce->vector[2] * tension);
#endif
        }
#ifdef DEBUG_CHECK_FORCES
        std::cerr.setf(std::ios::floatfield, std::ios::fixed);
        std::cerr << iter1->first << " " << force.x << " " << force.y << " " << force.z << "\n";
        std::cerr.unsetf(std::ios::floatfield);
#endif
    }

    // update the joints (needed for motors, end stops and stress calculations)
    std::map<std::string, Joint *>::const_iterator jointIter;
    for (jointIter = m_JointList.begin(); jointIter != m_JointList.end(); jointIter++) jointIter->second->Update();


#ifndef OUTPUTS_AFTER_SIMULATION_STEP
    if (m_OutputKinematicsFlag && m_StepCount % gDisplaySkip == 0) OutputKinematics();
    if (m_OutputWarehouseFlag) OutputWarehouse();
    if (m_OutputModelStateAtTime > 0.0)
    {
        if (m_SimulationTime >= m_OutputModelStateAtTime)
        {
            OutputProgramState();
            m_OutputModelStateAtTime = 0.0;
        }
    }
    else if (m_OutputModelStateAtCycle >= 0 && m_CycleTime >= 0 && m_SimulationTime >= m_CycleTime * m_OutputModelStateAtCycle)
    {
        OutputProgramState();
        m_OutputModelStateAtCycle = -1;
    }
    else if (m_OutputModelStateAtWarehouseDistance > 0 && m_WarehouseDistance >= m_OutputModelStateAtWarehouseDistance)
    {
        OutputProgramState();
        m_OutputModelStateAtWarehouseDistance = 0;
    }
#endif


    // run the simulation
    switch (m_StepType)
    {
    case WorldStep:
        dWorldStep(m_WorldID, m_StepSize);
        break;

    case QuickStep:
        dWorldQuickStep(m_WorldID, m_StepSize);
        break;
    }

    // update the time counter
    m_SimulationTime += m_StepSize;

    // update the step counter
    m_StepCount++;

    // calculate the energies
    for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
    {
        m_MechanicalEnergy += iter1->second->GetPower() * m_StepSize;
        m_MetabolicEnergy += iter1->second->GetMetabolicPower() * m_StepSize;
    }
    m_MetabolicEnergy += m_BMR * m_StepSize;

#ifdef USE_QT
    // update the footprint indicator
    if (m_ContactList.size() > 0)
    {
        dJointFeedback *jointFeedback;
        double *position;
        PlaneGeom *plane;
        for (unsigned int i = 0; i < m_ContactList.size(); i++)
        {
            if (dJointGetBody(m_ContactList[i]->GetJointID(), 0) == 0)
            {
                plane = dynamic_cast<PlaneGeom *>(m_Environment->GetGeom(0));
                if (plane)
                {
                    jointFeedback = m_ContactList[i]->GetJointFeedback();
                    position = (*m_ContactList[i]->GetContactPosition());
                    plane->AddImpulse(position[0], position[1], position[2], jointFeedback->f1[0], jointFeedback->f1[1], jointFeedback->f1[2], GetTimeIncrement());
                }
            }
            else if (dJointGetBody(m_ContactList[i]->GetJointID(), 1) == 0)
            {
                plane = dynamic_cast<PlaneGeom *>(m_Environment->GetGeom(0));
                if (plane)
                {
                    jointFeedback = m_ContactList[i]->GetJointFeedback();
                    position = (*m_ContactList[i]->GetContactPosition());
                    plane->AddImpulse(position[0], position[1], position[2], -jointFeedback->f1[0], -jointFeedback->f1[1], -jointFeedback->f1[2], GetTimeIncrement());
                }
            }
        }
    }

#endif

    // all reporting is done after a simulation step

    Dump();

    if (gDebug == MuscleDebug)
    {
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
        {
            *gDebugStream << *iter1->second->GetName() << " " << m_SimulationTime
                    << " length " << iter1->second->GetLength()
                    << " velocity " << iter1->second->GetVelocity()
                    << " tension " << iter1->second->GetTension()
                    << " power " << iter1->second->GetPower()
                    << " activation " << iter1->second->GetActivation()
                    << " metabolic " << iter1->second->GetMetabolicPower()
                    << "\n";
        }
    }

    double totalESE = 0;
    double totalEPE = 0;
    double totalElasticEnergy = 0;
    if (gDebug == EnergyPartitionDebug)
    {
        for (iter1 = m_MuscleList.begin(); iter1 != m_MuscleList.end(); iter1++)
        {
            UGMMuscle *ugm = dynamic_cast<UGMMuscle *>(iter1->second);
            if (ugm)
            {
                if (ugm->GetPower() > 0)
                    m_PositiveMechanicalWork += ugm->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += ugm->GetPower() * m_StepSize;

                if (ugm->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * ugm->GetVCE() *
                                                 ugm->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * ugm->GetVCE() *
                                                 ugm->GetFCE() * m_StepSize;

                if (ugm->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * ugm->GetVSE() *
                                                   ugm->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * ugm->GetVSE() *
                                                   ugm->GetFSE() * m_StepSize;

                if (ugm->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * ugm->GetVPE() *
                                                     ugm->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * ugm->GetVPE() *
                                                     ugm->GetFPE() * m_StepSize;

                *gDebugStream << *ugm->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << ugm->GetPower() << " ContractilePower "
                        << ugm->GetFCE() << " SerialElasticPower "
                        << ugm->GetFSE() << " ParallelElasticPower "
                        << ugm->GetFPE() << " SerialElasticEnergy "
                        << ugm->GetESE() << " ParallelElasticEnergy "
                        << ugm->GetEPE() << "\n";

                totalESE += ugm->GetESE();
                totalEPE += ugm->GetEPE();
                totalElasticEnergy += ugm->GetESE();
                totalElasticEnergy += ugm->GetEPE();
            }

            MAMuscle *mam = dynamic_cast<MAMuscle *>(iter1->second);
            if (mam)
            {
                if (mam->GetPower() > 0)
                    m_PositiveMechanicalWork += mam->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mam->GetPower() * m_StepSize;

                *gDebugStream << *mam->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mam->GetPower() << "\n";
            }

            MAMuscleExtended *mamext = dynamic_cast<MAMuscleExtended *>(iter1->second);
            if (mamext)
            {
                if (mamext->GetPower() > 0)
                    m_PositiveMechanicalWork += mamext->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mamext->GetPower() * m_StepSize;

                if (mamext->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * mamext->GetVCE() *
                                                 mamext->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * mamext->GetVCE() *
                                                 mamext->GetFCE() * m_StepSize;

                if (mamext->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * mamext->GetVSE() *
                                                   mamext->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * mamext->GetVSE() *
                                                   mamext->GetFSE() * m_StepSize;

                if (mamext->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * mamext->GetVPE() *
                                                     mamext->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * mamext->GetVPE() *
                                                     mamext->GetFPE() * m_StepSize;

                *gDebugStream << *mamext->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mamext->GetPower() << " ContractilePower "
                        << mamext->GetPCE() << " SerialElasticPower "
                        << mamext->GetPSE() << " ParallelElasticPower "
                        << mamext->GetPPE() << " SerialElasticEnergy "
                        << mamext->GetESE() << " ParallelElasticEnergy "
                        << mamext->GetEPE() << "\n";

                totalESE += mamext->GetESE();
                totalEPE += mamext->GetEPE();
                totalElasticEnergy += mamext->GetESE();
                totalElasticEnergy += mamext->GetEPE();
            }

            MAMuscleComplete *mamcomplete = dynamic_cast<MAMuscleComplete *>(iter1->second);
            if (mamcomplete)
            {
                if (mamcomplete->GetPower() > 0)
                    m_PositiveMechanicalWork += mamcomplete->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += mamcomplete->GetPower() * m_StepSize;

                if (mamcomplete->GetVCE() < 0)
                    m_PositiveContractileWork += -1 * mamcomplete->GetVCE() *
                                                 mamcomplete->GetFCE() * m_StepSize;
                else
                    m_NegativeContractileWork += -1 * mamcomplete->GetVCE() *
                                                 mamcomplete->GetFCE() * m_StepSize;

                if (mamcomplete->GetVSE() < 0)
                    m_PositiveSerialElasticWork += -1 * mamcomplete->GetVSE() *
                                                   mamcomplete->GetFSE() * m_StepSize;
                else
                    m_NegativeSerialElasticWork += -1 * mamcomplete->GetVSE() *
                                                   mamcomplete->GetFSE() * m_StepSize;

                if (mamcomplete->GetVPE() < 0)
                    m_PositiveParallelElasticWork += -1 * mamcomplete->GetVPE() *
                                                     mamcomplete->GetFPE() * m_StepSize;
                else
                    m_NegativeParallelElasticWork += -1 * mamcomplete->GetVPE() *
                                                     mamcomplete->GetFPE() * m_StepSize;

                *gDebugStream << *mamcomplete->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << mamcomplete->GetPower() << " ContractilePower "
                        << mamcomplete->GetPCE() << " SerialElasticPower "
                        << mamcomplete->GetPSE() << " ParallelElasticPower "
                        << mamcomplete->GetPPE() << " SerialElasticEnergy "
                        << mamcomplete->GetESE() << " ParallelElasticEnergy "
                        << mamcomplete->GetEPE() << "\n";

                totalESE += mamcomplete->GetESE();
                totalEPE += mamcomplete->GetEPE();
                totalElasticEnergy += mamcomplete->GetESE();
                totalElasticEnergy += mamcomplete->GetEPE();
            }

            DampedSpringMuscle *dsm = dynamic_cast<DampedSpringMuscle *>(iter1->second);
            if (dsm)
            {
                if (dsm->GetPower() > 0)
                    m_PositiveMechanicalWork += dsm->GetPower() * m_StepSize;
                else
                    m_NegativeMechanicalWork += dsm->GetPower() * m_StepSize;

                *gDebugStream << *dsm->GetName() << " "
                        << m_SimulationTime << " MechanicalPower "
                        << dsm->GetPower() << " ElasticEnergy "
                        << dsm->GetElasticEnergy() << "\n";

                totalElasticEnergy += dsm->GetElasticEnergy();
            }
        }

        double potentialEnergy, rotationalKineticEnergy;
        dVector3 linearKineticEnergy;
        double totalPotentialEnergy = 0;
        dVector3 totalLinearKineticEnergy;
        double totalRotationalKineticEnergy = 0;

        totalLinearKineticEnergy[0] = totalLinearKineticEnergy[1] = totalLinearKineticEnergy[2] = 0;

        std::map<std::string, Body *>::const_iterator iter5;
        for (iter5 = m_BodyList.begin(); iter5 != m_BodyList.end(); iter5++)
        {
            potentialEnergy = iter5->second->GetGravitationalPotentialEnergy();
            iter5->second->GetLinearKineticEnergy(linearKineticEnergy);
            rotationalKineticEnergy = iter5->second->GetRotationalKineticEnergy();
            *gDebugStream << *iter5->second->GetName() << " "
                    << m_SimulationTime << " "
                    << potentialEnergy << " "
                    << linearKineticEnergy[0] << " " << linearKineticEnergy[1] << " " << linearKineticEnergy[2] << " "
                    << rotationalKineticEnergy << "\n";
            totalPotentialEnergy += potentialEnergy;
            totalLinearKineticEnergy[0] += linearKineticEnergy[0];
            totalLinearKineticEnergy[1] += linearKineticEnergy[1];
            totalLinearKineticEnergy[2] += linearKineticEnergy[2];
            totalRotationalKineticEnergy += rotationalKineticEnergy;
        }
        *gDebugStream << "total_pe_lke3_rke_ese_epe" << " "
                << m_SimulationTime << " "
                << totalPotentialEnergy << " "
                << totalLinearKineticEnergy[0] << " " << totalLinearKineticEnergy[1] << " " << totalLinearKineticEnergy[2] << " "
                << totalRotationalKineticEnergy << " "
                << totalESE << " "
                << totalEPE << "\n";
    }

    if (gDebug == CentreOfMassDebug)
    {
        dVector3 cm = {0, 0, 0, 0};
        dVector3 cmv = {0, 0, 0, 0};
        const double *p;
        double mass;
        double totalMass = 0;
        std::map<std::string, Body *>::const_iterator iter4;
        for (iter4 = m_BodyList.begin(); iter4 != m_BodyList.end(); iter4++)
        {
            p = iter4->second->GetPosition();
            mass = iter4->second->GetMass();
            cm[0] += mass * p[0];
            cm[1] += mass * p[1];
            cm[2] += mass * p[2];
            p = iter4->second->GetLinearVelocity();
            cmv[0] += mass * p[0];
            cmv[1] += mass * p[1];
            cmv[2] += mass * p[2];
            totalMass += mass;
        }
        cm[0] /= totalMass; cm[1] /= totalMass; cm[2] /= totalMass;
        cmv[0] /= totalMass; cmv[1] /= totalMass; cmv[2] /= totalMass;
        *gDebugStream << "Time " << m_SimulationTime
                << " Mass " << totalMass
                << " CM " << cm[0] << " " << cm[1] << " " << cm[2]
                << " " << cmv[0] << " " << cmv[1] << " " << cmv[2] << "\n";
    }

    if (gDebug == JointDebug)
    {
        dJointFeedback *jointFeedback;
        std::map<std::string, Joint *>::const_iterator iter3;
        for (iter3 = m_JointList.begin(); iter3 != m_JointList.end(); iter3++)
        {
            jointFeedback = iter3->second->GetFeedback();
            *gDebugStream << "Joint " << *iter3->second->GetName() <<
                    " f1 " << jointFeedback->f1[0] << " " << jointFeedback->f1[1] << " " << jointFeedback->f1[2] << " " <<
                    " t1 " << jointFeedback->t1[0] << " " << jointFeedback->t1[1] << " " << jointFeedback->t1[2] << " " <<
                    " f2 " << jointFeedback->f2[0] << " " << jointFeedback->f2[1] << " " << jointFeedback->f2[2] << " " <<
                    " t2 " << jointFeedback->t2[0] << " " << jointFeedback->t2[1] << " " << jointFeedback->t2[2] << "\n";
            HingeJoint *hingeJoint = dynamic_cast<HingeJoint *>(iter3->second);
            if (hingeJoint)
            {
                dVector3 anchor, anchor2, axis;
                hingeJoint->GetHingeAnchor(anchor);
                hingeJoint->GetHingeAnchor2(anchor2);
                hingeJoint->GetHingeAxis(axis);
                *gDebugStream << "Joint " << *hingeJoint->GetName() <<
                        " Angle " << hingeJoint->GetHingeAngle() <<
                        " AngleRate " << hingeJoint->GetHingeAngleRate() <<
                        " Anchor " << anchor[0] << " "  << anchor[1] << " "  << anchor[2] <<
                        " Anchor2 " << anchor2[0] << " "  << anchor2[1] << " "  << anchor2[2] <<
                        " Axis " << axis[0] << " "  << axis[1] << " "  << axis[2] <<
                        "\n";
            }
            BallJoint *ballJoint = dynamic_cast<BallJoint *>(iter3->second);
            if (ballJoint)
            {
                dVector3 anchor, anchor2;
                ballJoint->GetBallAnchor(anchor);
                ballJoint->GetBallAnchor2(anchor2);
                *gDebugStream << "Joint " << *ballJoint->GetName() <<
                        " Anchor " << anchor[0] << " "  << anchor[1] << " "  << anchor[2] <<
                        " Anchor2 " << anchor2[0] << " "  << anchor2[1] << " "  << anchor2[2] <<
                        "\n";
            }        }
    }

    if (gDebug == ContactDebug)
    {
        dJointFeedback *jointFeedback;
        dBodyID bodyID;
        if (m_ContactList.size())
        {
            for (unsigned int i = 0; i < m_ContactList.size(); i++)
            {
                *gDebugStream << "Time " << m_SimulationTime << " ";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 0);
                if (bodyID == 0) *gDebugStream << "Static_Environment ";
                else *gDebugStream << *((Body *)(dBodyGetData(bodyID)))->GetName() << " ";
                bodyID = dJointGetBody(m_ContactList[i]->GetJointID(), 1);
                if (bodyID == 0) *gDebugStream << "Static_Environment";
                else *gDebugStream << *((Body *)(dBodyGetData(bodyID)))->GetName();

                *gDebugStream << " x " << (*m_ContactList[i]->GetContactPosition())[0] <<
                        " y " << (*m_ContactList[i]->GetContactPosition())[1] <<
                        " z " << (*m_ContactList[i]->GetContactPosition())[2];

                jointFeedback = m_ContactList[i]->GetJointFeedback();
                *gDebugStream <<
                        " f1 " << jointFeedback->f1[0] << " " << jointFeedback->f1[1] << " " << jointFeedback->f1[2] << " " <<
                        " t1 " << jointFeedback->t1[0] << " " << jointFeedback->t1[1] << " " << jointFeedback->t1[2] << " " <<
                        " f2 " << jointFeedback->f2[0] << " " << jointFeedback->f2[1] << " " << jointFeedback->f2[2] << " " <<
                        " t2 " << jointFeedback->t2[0] << " " << jointFeedback->t2[1] << " " << jointFeedback->t2[2] << "\n";
            }
        }
        else
        {
            *gDebugStream << "Time " << m_SimulationTime << " ";
            *gDebugStream << "nil nil";
            *gDebugStream << " x " << 0 << " y " << 0 << " z " << 0;
            *gDebugStream <<
                    " f1 " << 0 << " " << 0 << " " << 0 << " " <<
                    " t1 " << 0 << " " << 0 << " " << 0 << " " <<
                    " f2 " << 0 << " " << 0 << " " << 0 << " " <<
                    " t2 " << 0 << " " << 0 << " " << 0 << "\n";
        }
    }

    if (gDebug == ActivationSegmentStateDebug)
    {
        *gDebugStream << m_DriverList.size();
        std::map<std::string, Driver *>::const_iterator iter6;
        for (iter6 = m_DriverList.begin(); iter6 != m_DriverList.end(); iter6++)
        {
            *gDebugStream << "\t" << iter6->second->GetValue(m_SimulationTime);
        }

        *gDebugStream << "\t" << m_BodyList.size();
        std::map<std::string, Body *>::const_iterator iter5;
        for (iter5 = m_BodyList.begin(); iter5 != m_BodyList.end(); iter5++)
        {
            const double *p = iter5->second->GetPosition();
            const double *r = iter5->second->GetRotation();
            const double *v = iter5->second->GetLinearVelocity();
            const double *rv = iter5->second->GetAngularVelocity();
            *gDebugStream << "\t" << p[0] << "\t" << p[1] << "\t" << p[2];
            *gDebugStream << "\t" << r[0] << "\t" << r[1] << "\t" << r[2];
            *gDebugStream << "\t" << r[4] << "\t" << r[5] << "\t" << r[6];
            *gDebugStream << "\t" << r[8] << "\t" << r[9] << "\t" << r[10];
            *gDebugStream << "\t" << v[0] << "\t" << v[1] << "\t" << v[2];
            *gDebugStream << "\t" << rv[0] << "\t" << rv[1] << "\t" << rv[2];
        }
        *gDebugStream << "\n";
    }

#ifdef OUTPUTS_AFTER_SIMULATION_STEP
    if (m_OutputKinematicsFlag && m_StepCount % gDisplaySkip == 0) OutputKinematics();
    if (m_OutputWarehouseFlag) OutputWarehouse();
    if (m_OutputModelStateAtTime > 0.0)
    {
        if (m_SimulationTime >= m_OutputModelStateAtTime)
        {
            OutputProgramState();
            m_OutputModelStateAtTime = 0.0;
        }
    }
    else if (m_OutputModelStateAtCycle >= 0 && m_CycleTime >= 0 && m_SimulationTime >= m_CycleTime * m_OutputModelStateAtCycle)
    {
        OutputProgramState();
        m_OutputModelStateAtCycle = -1;
    }
    else if (m_OutputModelStateAtWarehouseDistance > 0 && m_WarehouseDistance >= m_OutputModelStateAtWarehouseDistance)
    {
        OutputProgramState();
        m_OutputModelStateAtWarehouseDistance = 0;
    }
#endif
}

//----------------------------------------------------------------------------
bool Simulation::TestForCatastrophy()
{
#if defined(USE_QT) && !defined(USE_WI_BB)
    std::stringstream ss;
#endif

    // first of all check to see that ODE is happy
    if (IsMessage())
    {
        int num;
        const char *messageText = GetLastMessage(&num);
#if defined(USE_QT) && !defined(USE_WI_BB)
        ss << "Failed due to ODE warning " << num << " " << messageText;
        m_MainWindow->log(ss.str().c_str());
#endif
        std::cerr << "Failed due to ODE warning " << num << " " << messageText << "\n";
        if (m_AbortOnODEMessage)
            return true;
    }

    // check for simulation error
    if (m_SimulationError)
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        ss << "Failed due to simulation error " << m_SimulationError;
        m_MainWindow->log(ss.str().c_str());
#endif
        std::cerr << "Failed due to simulation error " << m_SimulationError << "\n";
        return true;
    }

    // check for contact abort
    if (m_ContactAbort)
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        ss << "Failed due to contact abort";
        m_MainWindow->log(ss.str().c_str());
#endif
        std::cerr << "Failed due to contact abort\n";
        return true;
    }

    // check for data target abort
    if (m_DataTargetAbort)
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        ss << "Failed due to DataTarget abort";
        m_MainWindow->log(ss.str().c_str());
#endif
        std::cerr << "Failed due to DataTarget abort\n";
        return true;
    }

    // check that all bodies meet velocity and stop conditions

    std::map<std::string, Body *>::const_iterator iter1;
    LimitTestResult p;
    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        p = iter1->second->TestLimits();
        switch (p)
        {
        case WithinLimits:
            break;

        case XPosError:
        case YPosError:
        case ZPosError:
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << "Failed due to position error " << p << " in: " << *iter1->second->GetName();
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to position error " << p << " in: " << *iter1->second->GetName() << "\n";
            return true;

        case XVelError:
        case YVelError:
        case ZVelError:
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << "Failed due to velocity error " << p << " in: " << *iter1->second->GetName();
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to velocity error " << p << " in: " << *iter1->second->GetName() << "\n";
            return true;

        case NumericalError:
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << "Failed due to numerical error " << p << " in: " << *iter1->second->GetName();
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to numerical error " << p << " in: " << *iter1->second->GetName() << "\n";
            return true;
        }
    }

    std::map<std::string, Joint *>::const_iterator iter3;
    HingeJoint *j;
    FixedJoint *f;
    int t;
    for (iter3 = m_JointList.begin(); iter3 != m_JointList.end(); iter3++)
    {
        j = dynamic_cast<HingeJoint *>(iter3->second);
        if (j)
        {
            t = j->TestLimits();
            if (t < 0)
            {
#if defined(USE_QT) && !defined(USE_WI_BB)
                ss << __FILE__ << "Failed due to LoStopTorqueLimit error in: " << *iter3->second->GetName();
                m_MainWindow->log(ss.str().c_str());
#endif
                std::cerr << "Failed due to LoStopTorqueLimit error in: " << *iter3->second->GetName() << "\n";
                return true;
            }
            else if (t > 0)
            {
#if defined(USE_QT) && !defined(USE_WI_BB)
                ss << __FILE__ << "Failed due to HiStopTorqueLimit error in: " << *iter3->second->GetName();
                m_MainWindow->log(ss.str().c_str());
#endif
                std::cerr << "Failed due to HiStopTorqueLimit error in: " << *iter3->second->GetName() << "\n";
                return true;
            }
        }
    }

    // and test the reporters for stop conditions
    std::map<std::string, Reporter *>::const_iterator ReporterIter;
    for (ReporterIter = m_ReporterList.begin(); ReporterIter != m_ReporterList.end(); ReporterIter++)
    {
        if (ReporterIter->second->ShouldAbort())
        {
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << __FILE__ << "Failed due to Reporter Abort in: " << *ReporterIter->second->GetName();
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to Reporter Abort in: " << *ReporterIter->second->GetName() << "\n";
            return true;
        }
    }

    // test for WarehouseFailDistanceAbort if set
    if (m_WarehouseFailDistanceAbort > 0 && m_WarehouseList.size() > 0 && m_FitnessType != ClosestWarehouse)
    {
        if (m_WarehouseDistance > m_WarehouseFailDistanceAbort)
        {
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << __FILE__ << "Failed due to >WarehouseFailDistanceAbort. m_WarehouseFailDistanceAbort=" << m_WarehouseFailDistanceAbort << " WarehouseDistance = " << m_WarehouseDistance;
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to >WarehouseFailDistanceAbort. m_WarehouseFailDistanceAbort=" << m_WarehouseFailDistanceAbort << " WarehouseDistance = " << m_WarehouseDistance << "\n";
            return true;
        }
    }
    else if (m_WarehouseFailDistanceAbort < 0 && m_WarehouseList.size() > 0 && m_FitnessType != ClosestWarehouse && m_SimulationTime > 0)
    {
        if (m_WarehouseDistance < std::fabs(m_WarehouseFailDistanceAbort))
        {
#if defined(USE_QT) && !defined(USE_WI_BB)
            ss << __FILE__ << "Failed due to <WarehouseFailDistanceAbort. m_WarehouseFailDistanceAbort=" << m_WarehouseFailDistanceAbort << " WarehouseDistance = " << m_WarehouseDistance;
            m_MainWindow->log(ss.str().c_str());
#endif
            std::cerr << "Failed due to <WarehouseFailDistanceAbort. m_WarehouseFailDistanceAbort=" << m_WarehouseFailDistanceAbort << " WarehouseDistance = " << m_WarehouseDistance << "\n";
            return true;
        }
    }

    if (m_OutputModelStateOccured && m_AbortAfterModelStateOutput)
    {
#if defined(USE_QT) && !defined(USE_WI_BB)
        ss << __FILE__ << "Abort because ModelState successfully written";
        m_MainWindow->log(ss.str().c_str());
#endif
        std::cerr << "Abort because ModelState successfully written\n";
        return true;
    }

    return false;
}


//----------------------------------------------------------------------------
double Simulation::CalculateInstantaneousFitness()
{
    const double *p;
    switch (m_FitnessType)
    {
    case DistanceTravelled:
        p = m_DistanceTravelledBodyID->GetPosition();
        if (std::isinf(p[0]) || std::isnan(p[0]))
        {
            m_SimulationError = 1;
            return 0;
        }
        else
        {
            return p[0];
        }

    case KinematicMatch:
        return m_KinematicMatchFitness;

    case KinematicMatchMiniMax:
        return m_KinematicMatchMiniMaxFitness;

    case ClosestWarehouse:
        return m_ClosestWarehouseFitness;
    }
    return 0;
}

void Simulation::ParseGlobal(xmlNodePtr cur)
{
    dVector3 gravity;
    double ERP;
    double CFM;
    double contactMaxCorrectingVel;
    double contactSurfaceLayer;
    char *buf;

    // gravity
    buf = DoXmlGetProp(cur, (const xmlChar *)"GravityVector");
    if (buf == 0) throw __LINE__;
    Util::Double(buf, 3, m_DoubleList);
    gravity[0] = m_DoubleList[0];
    gravity[1] = m_DoubleList[1];
    gravity[2] = m_DoubleList[2];

    // set the simulation integration step size
    buf = DoXmlGetProp(cur, (const xmlChar *)"IntegrationStepSize");
    if (buf == 0) throw __LINE__;
    m_StepSize = Util::Double(buf);

    // can specify ERP & CFM; SpringConstant & DampingConstant; SpringConstant & ERP
    // but not CFM & DampingConstant - can't think why you would want to
    buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant");
    if (buf)
    {
        double ks = Util::Double(buf);
        buf = DoXmlGetProp(cur, (const xmlChar *)"DampingConstant");
        if (buf)
        {
            double kd = Util::Double(buf);
            ERP = m_StepSize * ks/(m_StepSize * ks + kd);
            CFM = 1.0/(m_StepSize * ks + kd);
        }
        else
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ERP");
            if (buf == 0) throw __LINE__;
            ERP = Util::Double(buf);
            CFM = ERP / (m_StepSize * ks);
        }
    }
    else
    {
        buf = DoXmlGetProp(cur, (const xmlChar *)"ERP");
        if (buf == 0) throw __LINE__;
        ERP = Util::Double(buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"CFM");
        if (buf == 0) throw __LINE__;
        CFM = Util::Double(buf);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactMaxCorrectingVel");
    if (buf == 0) throw __LINE__;
    contactMaxCorrectingVel = Util::Double(buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSurfaceLayer");
    if (buf == 0) throw __LINE__;
    contactSurfaceLayer = Util::Double(buf);

    // set the global simulation parameters
    dWorldSetGravity(m_WorldID, gravity[0], gravity[1], gravity[2]);
    dWorldSetERP(m_WorldID, ERP);
    dWorldSetCFM(m_WorldID, CFM);
    dWorldSetContactMaxCorrectingVel(m_WorldID, contactMaxCorrectingVel);
    dWorldSetContactSurfaceLayer(m_WorldID, contactSurfaceLayer);

    // get the stepper required
    // WorldStep, accurate but slow
    // QuickStep, faster but less accurate
    buf = DoXmlGetProp(cur, (const xmlChar *)"StepType");
    if (buf)
    {
        if (strcmp((char *)buf, "WorldStep") == 0) m_StepType = WorldStep;
        else if (strcmp((char *)buf, "QuickStep") == 0) m_StepType = QuickStep;
        else throw __LINE__;
    }

    // allow internal collisions
    buf = DoXmlGetProp(cur, (const xmlChar *)"AllowInternalCollisions");
    if (buf == 0) throw __LINE__;
    m_AllowInternalCollisions = Util::Bool(buf);

    // allow collisions for objects connected by a joint
    buf = DoXmlGetProp(cur, (const xmlChar *)"AllowConnectedCollisions");
    if (buf) m_AllowConnectedCollisions = Util::Bool(buf);

    // now some run parameters

    buf = DoXmlGetProp(cur, (const xmlChar *)"BMR");
    if (buf == 0) throw __LINE__;
    m_BMR = Util::Double(buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"TimeLimit");
    if (buf == 0) throw __LINE__;
    m_TimeLimit = Util::Double(buf);
    buf = DoXmlGetProp(cur, (const xmlChar *)"MechanicalEnergyLimit");
    if (buf == 0) throw __LINE__;
    m_MechanicalEnergyLimit = Util::Double(buf);
    buf = DoXmlGetProp(cur, (const xmlChar *)"MetabolicEnergyLimit");
    if (buf == 0) throw __LINE__;
    m_MetabolicEnergyLimit = Util::Double(buf);
    buf = DoXmlGetProp(cur, (const xmlChar *)"DistanceTravelledBodyID"); // DistanceTravelledBodyID is used for Munge so it is necessary anyway
    if (buf == 0) throw __LINE__;
    m_DistanceTravelledBodyIDName = buf;
    buf = DoXmlGetProp(cur, (const xmlChar *)"FitnessType");
    if (buf == 0) throw __LINE__;
    if (strcmp((char *)buf, "DistanceTravelled") == 0) m_FitnessType = DistanceTravelled;
    else if (strcmp((char *)buf, "KinematicMatch") == 0) m_FitnessType = KinematicMatch;
    else if (strcmp((char *)buf, "KinematicMatchMiniMax") == 0) m_FitnessType = KinematicMatchMiniMax;
    else if (strcmp((char *)buf, "ClosestWarehouse") == 0) m_FitnessType = ClosestWarehouse;
    else throw __LINE__;

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateFilename");
    if (buf) SetOutputModelStateFile((char *)buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateAtTime");
    if (buf) SetOutputModelStateAtTime(Util::Double(buf));

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputModelStateAtCycle");
    if (buf) SetOutputModelStateAtCycle(Util::Int(buf));

    buf = DoXmlGetProp(cur, (const xmlChar *)"MungeModelState");
    if (buf) SetMungeModelStateFlag(Util::Bool(buf));

    buf = DoXmlGetProp(cur, (const xmlChar *)"OutputKinematicsFile");
    if (buf) SetOutputKinematicsFile((char *)buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"InputKinematicsFile");
    if (buf) SetInputKinematicsFile((char *)buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"YUp");
    if (buf) SetYUp(Util::Bool(buf));

    buf = DoXmlGetProp(cur, (const xmlChar *)"WarehouseFailDistanceAbort");
    if (buf) m_WarehouseFailDistanceAbort = Util::Double(buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"WarehouseUnitIncreaseDistanceThreshold");
    if (buf) m_WarehouseUnitIncreaseDistanceThreshold = Util::Double(buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"WarehouseDecreaseThresholdFactor");
    if (buf) m_WarehouseDecreaseThresholdFactor = Util::Double(buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"CurrentWarehouse");
    if (buf) m_CurrentWarehouse = buf;

    buf = DoXmlGetProp(cur, (const xmlChar *)"AbortAfterModelStateOutput");
    if (buf) m_AbortAfterModelStateOutput = Util::Bool(buf);
}

void Simulation::ParseEnvironment(xmlNodePtr cur)
{
    char *buf;

    // planes
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Plane"));
    Util::Double(buf, 4, m_DoubleList);
    PlaneGeom *plane = new PlaneGeom(m_SpaceID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
    plane->SetGeomLocation(Geom::environment);
    m_Environment->AddGeom(plane);

#ifdef USE_QT
    buf = DoXmlGetProp(cur, (const xmlChar *)"TrackSensitivity");
    if (buf)
    {
        Util::Double(buf, 2, m_DoubleList);
        // list is lowRange, highRange
        plane->SetTrackSensitivity(m_DoubleList[0], m_DoubleList[1]);

        buf = DoXmlGetProp(cur, (const xmlChar *)"CheckerboardLow"); // these need to be set before SetTrackPatch
        if (buf) plane->SetCheckerboardLow(Util::Double(buf));
        buf = DoXmlGetProp(cur, (const xmlChar *)"CheckerboardHigh"); // these need to be set before SetTrackPatch
        if (buf) plane->SetCheckerboardHigh(Util::Double(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TrackPatch"));
        Util::Double(buf, 6, m_DoubleList);
        // list is trackPatchStartX, trackPatchStartY, trackPatchEndX, trackPatchEndY, trackPatchResolutionX, trackPatchResolutionY
        plane->SetTrackPatch(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3], m_DoubleList[4], m_DoubleList[5]);

        buf = DoXmlGetProp(cur, (const xmlChar *)"TrackDrawThreshold");
        if (buf) plane->SetTrackDrawThreshold(Util::Double(buf));
    }

    m_Environment->SetAxisSize(m_Interface.EnvironmentAxisSize);
    m_Environment->SetColour(m_Interface.EnvironmentColour);
#endif
}

void Simulation::ParseBody(xmlNodePtr cur)
{
    char *buf;
    dMass mass;
    double theMass;
    double I11, I22, I33, I12, I13, I23;

    // create the new body
    Body *theBody = new Body(m_WorldID);
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    theBody->SetName((const char *)buf);

    // set the start parameters
    // note quaternion is (qs,qx,qy,qz)
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion"));
    theBody->SetQuaternion((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
    theBody->SetPosition((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocity"));
    theBody->SetLinearVelocity((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"AngularVelocity"));
    theBody->SetAngularVelocity((const char *)buf);

    // and now the mass properties
    // (remember the origin is always at the centre of mass)

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Mass"));
    theMass = Util::Double(buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MOI"));
    Util::Double(buf, 6, m_DoubleList);

    // note: inertial matrix is as follows
    // [ I11 I12 I13 ]
    // [ I12 I22 I23 ]
    // [ I13 I23 I33 ]
    I11 = m_DoubleList[0];
    I22 = m_DoubleList[1];
    I33 = m_DoubleList[2];
    I12 = m_DoubleList[3];
    I13 = m_DoubleList[4];
    I23 = m_DoubleList[5];
    dMassSetParameters(&mass, theMass, 0, 0, 0, I11, I22, I33, I12, I13, I23);
    theBody->SetMass(&mass);

    // get limits if available
    buf = DoXmlGetProp(cur, (const xmlChar *)"PositionLowBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetPositionLowBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"PositionHighBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetPositionHighBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocityLowBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetLinearVelocityLowBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearVelocityHighBound");
    if (buf)
    {
        Util::Double(buf, 3, m_DoubleList);
        theBody->SetLinearVelocityHighBound(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
    }

    // set damping if necessary
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearDamping");
    if (buf) theBody->SetLinearDamping(Util::Double(buf));
    buf = DoXmlGetProp(cur, (const xmlChar *)"AngularDamping");
    if (buf) theBody->SetAngularDamping(Util::Double(buf));
    buf = DoXmlGetProp(cur, (const xmlChar *)"LinearDampingThreshold");
    if (buf) theBody->SetLinearDampingThreshold(Util::Double(buf));
    buf = DoXmlGetProp(cur, (const xmlChar *)"AngularDampingThreshold");
    if (buf) theBody->SetAngularDampingThreshold(Util::Double(buf));
    buf = DoXmlGetProp(cur, (const xmlChar *)"MaxAngularSpeed");
    if (buf) theBody->SetMaxAngularSpeed(Util::Double(buf));


#ifdef USE_QT
    FacetedObject *facetedObject = new FacetedObject();

    // parameters that affect how the mesh is read in
    buf = DoXmlGetProp(cur, (const xmlChar *)"VerticesAsSpheresRadius");
    if (buf)
    {
        facetedObject->SetVerticesAsSpheresRadius(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"BadMesh");
    if (buf)
    {
        facetedObject->SetBadMesh(Util::Bool(buf));
    }

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"GraphicFile"));
    std::string filename;
    if (gGraphicsRoot)
    {
        if (strlen(gGraphicsRoot) > 0)
            filename = std::string(gGraphicsRoot) + std::string("/");
    }
    filename += std::string((const char *)buf);
    facetedObject->ParseOBJFile(filename.c_str());

    // parameters for altering mesh after it has been read
    buf = DoXmlGetProp(cur, (const xmlChar *)"Scale");
    if (buf)
    {
        int nTokens = DataFile::CountTokens(buf);
        if (nTokens == 1)
        {
            double scale = Util::Double(buf);
            if (scale != 1.0)
                facetedObject->Scale(scale, scale, scale);
        }
        else
        {
            if (nTokens == 3)
            {
                Util::Double(buf, 3, m_DoubleList);
                if (m_DoubleList[0] != 1.0 || m_DoubleList[1] != 1.0 || m_DoubleList[2] != 1.0)
                    facetedObject->Scale(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            }
            else
            {
                throw __LINE__;
            }
        }
    }
    double density = -1;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Density");
    if (buf)
    {
        density = Util::Double(buf);
    }
    if (density <= 0)
    {
        buf = DoXmlGetProp(cur, (const xmlChar *)"Offset");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            theBody->SetOffset(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            facetedObject->Move(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
        }
    }

    bool clockwise = false;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Clockwise");
    if (buf)
    {
        clockwise = Util::Bool(buf);
    }
    // but we always want anticlockwise objects
    if (clockwise) facetedObject->ReverseWinding();

    theBody->SetFacetedObject(facetedObject);
    theBody->SetColour(m_Interface.BodyColour);
    theBody->SetAxisSize(m_Interface.BodyAxisSize);

    if (density > 0)
    {
        // override the position values
        theBody->SetPosition(0, 0, 0);
        theBody->SetQuaternion(1, 0, 0, 0);

        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        const double *p = theBody->GetPosition();
        dVector3 newP;
        newP[0] = mass.c[0] + p[0]; newP[1] = mass.c[1] + p[1]; newP[2] = mass.c[2] + p[2];
        theBody->SetOffset(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->Move(-mass.c[0], -mass.c[1], -mass.c[2]);
        facetedObject->CalculateMassProperties(&mass, density, false); // generally we assume anticlockwise winding
        std::cerr << *theBody->GetName() << " mass " << mass.mass
                << " CM " << mass.c[0] << " " << mass.c[1] << " " << mass.c[2] << " "
                << " I11_I22_I33 " << mass._I(0,0) << " " << mass._I(1,1) << " " << mass._I(2,2) << " "
                << " I12_I13_I23 " << mass._I(0,1) << " " << mass._I(0,2) << " " << mass._I(1,2) << "\n";
        mass.c[0] = mass.c[1] = mass.c[2]  = 0;
        theBody->SetMass(&mass);
        theBody->SetPosition(newP[0], newP[1], newP[2]);
    }

    // now create a dummy geom for picking
    TrimeshGeom *trimeshGeom = new TrimeshGeom(0, facetedObject);
    trimeshGeom->SetBody(theBody->GetBodyID());
    trimeshGeom->SetName(*theBody->GetName());
    m_PickGeomList.push_back(trimeshGeom);

#endif

    m_BodyList[*theBody->GetName()] = theBody;
}


void Simulation::ParseJoint(xmlNodePtr cur)
{
    char *buf;
    Joint *joint = 0;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "Hinge") == 0)
    {

        HingeJoint *hingeJoint = new HingeJoint(m_WorldID);
        joint = hingeJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        hingeJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body1 = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) THROWIFZERO(body2 = m_BodyList[(const char *)buf]);
        hingeJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAnchor"));
        hingeJoint->SetHingeAnchor((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HingeAxis"));
        hingeJoint->SetHingeAxis((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"StartAngleReference");
        if (buf)
        {
            hingeJoint->SetStartAngleReference(Util::GetAngle(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop"));
            double loStop = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
            double hiStop = Util::GetAngle(buf);
            hingeJoint->SetJointStops(loStop, hiStop);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"HiStopTorqueLimit");
        if (buf)
        {
            double hiStopTorqueLimit = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"LoStopTorqueLimit"));
            double loStopTorqueLimit = Util::Double(buf);
            hingeJoint->SetTorqueLimits(loStopTorqueLimit, hiStopTorqueLimit);

            buf = DoXmlGetProp(cur, (const xmlChar *)"StopTorqueWindow");
            if (buf) hingeJoint->SetStopTorqueWindow(Util::Int(buf));
        }

        // can specify StopERP & StopCFM; StopSpringConstant & StopDampingConstant; StopSpringConstant & StopERP
        // but not StopCFM & StopDampingConstant - can't think why you would want to
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopCFM");
        if (buf)
        {
            hingeJoint->SetStopCFM(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP");
        if (buf)
        {
            hingeJoint->SetStopERP(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopSpringConstant");
        if (buf)
        {
            double ks = Util::Double(buf);
            buf = DoXmlGetProp(cur, (const xmlChar *)"StopDampingConstant");
            if (buf)
            {
                double kd = Util::Double(buf);
                hingeJoint->SetStopSpringDamp(ks, kd, m_StepSize);
            }
            else
            {
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP"));
                double erp = Util::Double(buf);
                hingeJoint->SetStopSpringERP(ks, erp, m_StepSize);
            }
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StopBounce");
        if (buf)
        {
            hingeJoint->SetStopBounce(Util::Double(buf));
        }

    }

    else if (strcmp((const char *)buf, "FloatingHinge") == 0)
    {

        FloatingHingeJoint *floatingHingeJoint = new FloatingHingeJoint(m_WorldID);
        joint = floatingHingeJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        floatingHingeJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        floatingHingeJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FloatingHingeAxis"));
        floatingHingeJoint->SetFloatingHingeAxis((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"StartAngleReference");
        if (buf)
        {
            floatingHingeJoint->SetStartAngleReference(Util::GetAngle(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop"));
            double loStop = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
            double hiStop = Util::GetAngle(buf);
            floatingHingeJoint->SetJointStops(loStop, hiStop);
        }
    }

    else if (strcmp((const char *)buf, "Ball") == 0)
    {
        int mode;
        buf = DoXmlGetProp(cur, (const xmlChar *)"AxisMode");
        if (buf ==0)
        {
            mode = -99;
        }
        else
        {
            if (strcmp((const char *)buf, "FixedEuler") == 0) mode = dAMotorEuler;
            else if (strcmp((const char *)buf, "UserEuler") == 0) mode = dAMotorUser;
            else throw __LINE__;
        }

        BallJoint *ballJoint = new BallJoint(m_WorldID, mode);
        joint = ballJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        ballJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        ballJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BallAnchor"));
        ballJoint->SetBallAnchor((const char *)buf);


        if (mode != -99)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Axis0"));
            Util::Double(buf, 3, m_DoubleList);
            pgd::Vector a0(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            if (mode == dAMotorUser)
            {
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Axis1"));
                Util::Double(buf, 3, m_DoubleList);
            }
            pgd::Vector a1(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Axis2"));
            Util::Double(buf, 3, m_DoubleList);
            pgd::Vector a2(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
/*
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Angle0"));
        double angle0 = Util::GetAngle(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Angle1"));
        double angle1 = Util::GetAngle(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Angle2"));
        double angle2 = Util::GetAngle(buf);
*/
            ballJoint->SetAxes(a0.x, a0.y, a0.z, a1.x, a1.y, a1.z, a2.x, a2.y, a2.z, 1);


            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop0"));
            double loStop0 = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop0"));
            double hiStop0 = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop1"));
            double loStop1 = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop1"));
            double hiStop1 = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop2"));
            double loStop2 = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop2"));
            double hiStop2 = Util::GetAngle(buf);

            ballJoint->SetStops(loStop0, hiStop0, loStop1, hiStop1, loStop2, hiStop2);

            // set the EulerReferenceVectors if present (only used in dAMotorEuler mode)
            buf = DoXmlGetProp(cur, (const xmlChar *)"EulerReferenceVectors");
            if (buf)
            {
                Util::Double(buf, 6, m_DoubleList);
                dVector3 reference1, reference2;
                reference1[0] = m_DoubleList[0]; reference1[1] = m_DoubleList[1]; reference1[2] = m_DoubleList[2];
                reference2[0] = m_DoubleList[3]; reference2[1] = m_DoubleList[4]; reference2[2] = m_DoubleList[5];
                ballJoint->SetEulerReferenceVectors(reference1, reference2);
            }
        }
    }

    else if (strcmp((const char *)buf, "Fixed") == 0)
    {

        FixedJoint *fixedJoint = new FixedJoint(m_WorldID);
        joint = fixedJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        fixedJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        fixedJoint->Attach(body1, body2);
        fixedJoint->SetFixed();


    }

    else if (strcmp((const char *)buf, "Universal") == 0)
    {

        UniversalJoint *universalJoint = new UniversalJoint(m_WorldID);
        joint = universalJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        universalJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        universalJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"UniversalAnchor"));
        universalJoint->SetUniversalAnchor((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"UniversalAxis1"));
        universalJoint->SetUniversalAxis1((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"UniversalAxis2"));
        universalJoint->SetUniversalAxis2((const char *)buf);

    }

    else if (strcmp((const char *)buf, "AMotor") == 0)
    {

        AMotorJoint *aMotorJoint = new AMotorJoint(m_WorldID);
        joint = aMotorJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        aMotorJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        aMotorJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Axis"));
        aMotorJoint->SetAxis((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop");
        if (buf)
        {
            double loStop = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
            double hiStop = Util::GetAngle(buf);
            aMotorJoint->SetStops(loStop, hiStop);
        }

        bool maxTorqueSpecified = false;
        buf = DoXmlGetProp(cur, (const xmlChar *)"MaxTorque");
        if (buf)
        {
            aMotorJoint->SetMaxTorque(Util::Double(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetVelocity"));
            aMotorJoint->SetTargetVelocity(Util::Double(buf));
            maxTorqueSpecified = true;
        }

    }

    else if (strcmp((const char *)buf, "Slider") == 0)
    {
        SliderJoint *sliderJoint = new SliderJoint(m_WorldID);
        joint = sliderJoint;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        sliderJoint->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body1ID"));
        Body *body1 = 0;
        if (strcmp((const char *)buf, "World")) body1 = m_BodyList[(const char *)buf];
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Body2ID"));
        Body *body2 = 0;
        if (strcmp((const char *)buf, "World")) body2 = m_BodyList[(const char *)buf];
        sliderJoint->Attach(body1, body2);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SliderAxis"));
        sliderJoint->SetSliderAxis((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"StartDistanceReference");
        if (buf)
        {
            sliderJoint->SetStartDistanceReference(Util::GetAngle(buf));
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamLoStop"));
            double loStop = Util::GetAngle(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParamHiStop"));
            double hiStop = Util::GetAngle(buf);
            sliderJoint->SetJointStops(loStop, hiStop);
        }

        // can specify StopERP & StopCFM; StopSpringConstant & StopDampingConstant; StopSpringConstant & StopERP
        // but not StopCFM & StopDampingConstant - can't think why you would want to
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopCFM");
        if (buf)
        {
            sliderJoint->SetStopCFM(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP");
        if (buf)
        {
            sliderJoint->SetStopERP(Util::Double(buf));
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"StopSpringConstant");
        if (buf)
        {
            double ks = Util::Double(buf);
            buf = DoXmlGetProp(cur, (const xmlChar *)"StopDampingConstant");
            if (buf)
            {
                double kd = Util::Double(buf);
                sliderJoint->SetStopSpringDamp(ks, kd, m_StepSize);
            }
            else
            {
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StopERP"));
                double erp = Util::Double(buf);
                sliderJoint->SetStopSpringERP(ks, erp, m_StepSize);
            }
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StopBounce");
        if (buf)
        {
            sliderJoint->SetStopBounce(Util::Double(buf));
        }

    }

    else
    {
        throw __LINE__;
    }

#ifdef USE_QT
    joint->SetAxisSize(m_Interface.JointAxisSize);
    joint->SetColour(m_Interface.JointColour);
#endif

    m_JointList[*joint->GetName()] = joint;
}

void Simulation::ParseGeom(xmlNodePtr cur)
{
    char *buf;
    Geom *geom = 0;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "CappedCylinder") == 0)
    {

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Radius"));
        double radius = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Length"));
        double length = Util::Double(buf);

        CappedCylinderGeom *cappedCylinderGeom = new CappedCylinderGeom(m_SpaceID, radius, length);
        geom = cappedCylinderGeom;
    }

    else if (strcmp((const char *)buf, "Sphere") == 0)
    {

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Radius"));
        double radius = Util::Double(buf);

        SphereGeom *cappedCylinderGeom = new SphereGeom(m_SpaceID, radius);
        geom = cappedCylinderGeom;
    }

    else if (strcmp((const char *)buf, "Box") == 0)
    {

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Dimensions"));
        Util::Double(buf, 3, m_DoubleList);

        BoxGeom *boxGeom = new BoxGeom(m_SpaceID, m_DoubleList[0], m_DoubleList[1], m_DoubleList[2]);
        geom = boxGeom;
    }

    else
    {
        throw __LINE__;
    }

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    geom->SetName((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
    dBodyID bodyID = m_BodyList[(const char *)buf]->GetBodyID();
    geom->SetBody(bodyID);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
    geom->SetPosition((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion"));
    geom->SetQuaternion((const char *)buf);

    // can specify ContactSoftERP & ContactSoftCFM; SpringConstant & DampingConstant; SpringConstant & ContactSoftERP
    // but not ContactSoftCFM & DampingConstant - can't think why you would want to
    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftCFM");
    if (buf)
    {
        geom->SetContactSoftCFM(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftERP");
    if (buf)
    {
        geom->SetContactSoftERP(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant");
    if (buf)
    {
        double ks = Util::Double(buf);
        buf = DoXmlGetProp(cur, (const xmlChar *)"DampingConstant");
        if (buf)
        {
            double kd = Util::Double(buf);
            geom->SetSpringDamp(ks, kd, m_StepSize);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ContactSoftERP"));
            double erp = Util::Double(buf);
            geom->SetSpringERP(ks, erp, m_StepSize);
        }
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Bounce");
    if (buf)
    {
        geom->SetContactBounce(Util::Double(buf));
    }
    buf = DoXmlGetProp(cur, (const xmlChar *)"Mu");
    if (buf)
    {
        if (strcasecmp((const char *)buf, "infinity") == 0) geom->SetContactMu(dInfinity);
        else geom->SetContactMu(Util::Double(buf));
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Abort");
    if (buf)
    {
        geom->SetAbort(Util::Bool(buf));
    }

#ifdef USE_QT
    geom->SetColour(m_Interface.GeomColour);
#endif

    geom->SetGeomLocation(Geom::body);
    m_GeomList[*geom->GetName()] = geom;
}

void Simulation::ParseMuscle(xmlNodePtr cur)
{
    char *buf;
    Muscle *muscle;
    Strap *strap;
    std::string muscleID;
    std::string strapID;
    Body *theBody;
    dVector3 p;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    muscleID = (const char *)buf;

    strapID = muscleID + "Strap";

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Strap"));
    if (strcmp((char *)buf, "TwoPoint") ==  0)
    {
        // 2 attachment point muscle

        TwoPointStrap *twoPointStrap = new TwoPointStrap();
        strap = twoPointStrap;
        twoPointStrap->SetName(strapID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
        twoPointStrap->SetOrigin(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
        twoPointStrap->SetInsertion(theBody, (const char *)buf);
    }
    else if (strcmp((char *)buf, "ThreePoint") ==  0)
    {
        // 3 attachment point muscle

        ThreePointStrap *threePointStrap = new ThreePointStrap();
        strap = threePointStrap;
        threePointStrap->SetName(strapID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
        threePointStrap->SetOrigin(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MidPointBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MidPoint"));
        threePointStrap->SetMidpoint(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
        threePointStrap->SetInsertion(theBody, (const char *)buf);
    }
    else if (strcmp((char *)buf, "NPoint") ==  0)
    {
        // 3 attachment point muscle

        NPointStrap *nPointStrap = new NPointStrap();
        strap = nPointStrap;
        nPointStrap->SetName(strapID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
        nPointStrap->SetOrigin(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
        nPointStrap->SetInsertion(theBody, (const char *)buf);

        int viaCount = 0;
        std::vector<Body *> bodyList;
        std::vector<std::string *> pointList;
        while (1)
        {
            sprintf(m_Buffer, "ViaPoint%d", viaCount);
            buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer);
            if (buf == 0) break;
            std::string *tempP = new std::string((const char *)buf);
            pointList.push_back(tempP);
            sprintf(m_Buffer, "ViaPointBody%d", viaCount);
            buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer);
            if (buf == 0)
            {
                sprintf(m_Buffer, "ViaPoint%dBodyID", viaCount);
                THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)m_Buffer));
            }
            THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
            bodyList.push_back(theBody);
            viaCount++;
        }
        THROWIFZERO(pointList.size());
        nPointStrap->SetViaPoints(&bodyList, &pointList);
        for (unsigned int i = 0; i < pointList.size(); i++) delete pointList[i];
    }
    else if (strcmp((char *)buf, "CylinderWrap") ==  0)
    {
        // cylinder wrapping muscle

        CylinderWrapStrap *cylinderWrapStrap = new CylinderWrapStrap();
        strap = cylinderWrapStrap;
        cylinderWrapStrap->SetName(strapID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
        cylinderWrapStrap->SetOrigin(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
        cylinderWrapStrap->SetInsertion(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        cylinderWrapStrap->SetCylinderBody(theBody);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderPosition"));
        cylinderWrapStrap->SetCylinderPosition((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderRadius"));
        cylinderWrapStrap->SetCylinderRadius(Util::Double(buf));
        buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderQuaternion");
        if (buf)
        {
            cylinderWrapStrap->SetCylinderQuaternion((const char *)buf);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderAxis"));
            cylinderWrapStrap->SetCylinderAxis((const char *)buf);
        }

#ifdef USE_QT
        cylinderWrapStrap->SetCylinderColour(m_Interface.StrapCylinderColour);
        cylinderWrapStrap->SetCylinderLength(m_Interface.StrapCylinderLength);
#endif
    }
    else if (strcmp((char *)buf, "TwoCylinderWrap") ==  0)
    {
        // cylinder wrapping muscle

        TwoCylinderWrapStrap *twoCylinderWrapStrap = new TwoCylinderWrapStrap();
        strap = twoCylinderWrapStrap;
        twoCylinderWrapStrap->SetName(strapID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"OriginBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Origin"));
        twoCylinderWrapStrap->SetOrigin(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"InsertionBodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Insertion"));
        twoCylinderWrapStrap->SetInsertion(theBody, (const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder1BodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        twoCylinderWrapStrap->SetCylinder1Body(theBody);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder1Position"));
        twoCylinderWrapStrap->SetCylinder1Position((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder1Radius"));
        twoCylinderWrapStrap->SetCylinder1Radius(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder2BodyID"));
        THROWIFZERO(theBody = m_BodyList[(const char *)buf]);
        twoCylinderWrapStrap->SetCylinder2Body(theBody);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder2Position"));
        twoCylinderWrapStrap->SetCylinder2Position((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Cylinder2Radius"));
        twoCylinderWrapStrap->SetCylinder2Radius(Util::Double(buf));
        buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderQuaternion");
        if (buf)
        {
            twoCylinderWrapStrap->SetCylinderQuaternion((const char *)buf);
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CylinderAxis"));
            twoCylinderWrapStrap->SetCylinderAxis((const char *)buf);
        }

#ifdef USE_QT
        twoCylinderWrapStrap->SetCylinderColour(m_Interface.StrapCylinderColour);
        twoCylinderWrapStrap->SetCylinderLength(m_Interface.StrapCylinderLength);
#endif
    }
    else
    {
        std::cerr << "Unrecognised Strap Type:" << buf << "\n";
        throw __LINE__;
    }

#ifdef USE_QT
    strap->SetRadius(m_Interface.StrapRadius);
    strap->SetForceRadius(m_Interface.StrapForceRadius);
    strap->SetForceScale(m_Interface.StrapForceScale);
#endif

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));
    if (strcmp((const char *)buf, "MinettiAlexander") == 0)
    {
        muscle = new MAMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        double forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        double vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        double pca = Util::Double(buf);
        ((MAMuscle *)muscle)->SetF0(pca * forcePerUnitArea);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        double fibreLength = Util::Double(buf);
        ((MAMuscle *)muscle)->SetVMax(fibreLength * vMaxFactor);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        ((MAMuscle *)muscle)->SetK(Util::Double(buf));
    }
    else if (strcmp((const char *)buf, "MinettiAlexanderExtended") == 0)
    {
        muscle = new MAMuscleExtended(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        double forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        double vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        double pca = Util::Double(buf);
        double f0 = pca * forcePerUnitArea;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        double fibreLength = Util::Double(buf);
        double vMax = fibreLength * vMaxFactor;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        double activationK = Util::Double(buf);
        ((MAMuscleExtended *)muscle)->SetMuscleProperties(vMax, f0, activationK);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        double tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        double serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        double parallelStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationKinetics"));
        bool activationKinetics = Util::Bool(buf);
        double serialElasticConstant;
        if (tendonLength > 0) serialElasticConstant = f0 / (serialStrainAtFmax * tendonLength);
        else serialElasticConstant = f0 / (serialStrainAtFmax * 0.001); // apply standard fixup (1mm tendon)
        if (serialElasticConstant < 0) throw __LINE__;
        double parallelElasticConstant;
        if (parallelStrainAtFmax <= 0) parallelElasticConstant = 0;
        else parallelElasticConstant = f0 / (parallelStrainAtFmax * fibreLength);
        ((MAMuscleExtended *)muscle)->SetParallelElasticProperties(parallelElasticConstant, fibreLength);
        ((MAMuscleExtended *)muscle)->SetSerialElasticProperties(serialElasticConstant, tendonLength);
        ((MAMuscleExtended *)muscle)->SetActivationKinetics(activationKinetics);

        buf = DoXmlGetProp(cur, (const xmlChar *)"InitialFibreLength");
        if (buf)
            ((MAMuscleExtended *)muscle)->SetInitialFibreLength(Util::Double(buf));
    }
    else if (strcmp((const char *)buf, "MinettiAlexanderComplete") == 0)
    {
        muscle = new MAMuscleComplete(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        double forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        double vMaxFactor = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        double pca = Util::Double(buf);
        double f0 = pca * forcePerUnitArea;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        double fibreLength = Util::Double(buf);
        double vMax = fibreLength * vMaxFactor;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationK"));
        double activationK = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Width"));
        double width = Util::Double(buf);
        ((MAMuscleComplete *)muscle)->SetMuscleProperties(vMax, f0, activationK, width);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        double tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        double serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainRateAtFmax"));
        double serialStrainRateAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainModel"));
        MAMuscleComplete::StrainModel serialStrainModel;
        if (strcasecmp((const char *)buf, "Linear") == 0)
            serialStrainModel = MAMuscleComplete::linear;
        else if (strcasecmp((const char *)buf, "Square") == 0)
            serialStrainModel = MAMuscleComplete::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        double parallelStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainRateAtFmax"));
        double parallelStrainRateAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainModel"));
        MAMuscleComplete::StrainModel parallelStrainModel;
        if (strcasecmp((const char *)buf, "Linear") == 0)
            parallelStrainModel = MAMuscleComplete::linear;
        else if (strcasecmp((const char *)buf, "Square") == 0)
            parallelStrainModel = MAMuscleComplete::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationKinetics"));
        bool activationKinetics = Util::Bool(buf);
        ((MAMuscleComplete *)muscle)->SetSerialElasticProperties(serialStrainAtFmax, serialStrainRateAtFmax, tendonLength, serialStrainModel);
        ((MAMuscleComplete *)muscle)->SetParallelElasticProperties(parallelStrainAtFmax, parallelStrainRateAtFmax, fibreLength, parallelStrainModel);
        if (activationKinetics)
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FastTwitchProportion"));
            double akFastTwitchProportion = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TActivationA"));
            double akTActivationA = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TActivationB"));
            double akTActivationB = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TDeactivationA"));
            double akTDeactivationA = Util::Double(buf);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TDeactivationB"));
            double akTDeactivationB = Util::Double(buf);
            ((MAMuscleComplete *)muscle)->SetActivationKinetics(activationKinetics, akFastTwitchProportion, akTActivationA, akTActivationB, akTDeactivationA, akTDeactivationB);
        }
        buf = DoXmlGetProp(cur, (const xmlChar *)"InitialFibreLength");
        if (buf)
            ((MAMuscleComplete *)muscle)->SetInitialFibreLength(Util::Double(buf));
        buf = DoXmlGetProp(cur, (const xmlChar *)"ActivationRate");
        if (buf)
            ((MAMuscleComplete *)muscle)->SetActivationRate(Util::Double(buf));
        buf = DoXmlGetProp(cur, (const xmlChar *)"StartActivation");
        if (buf)
            ((MAMuscleComplete *)muscle)->SetStartActivation(Util::Double(buf));
    }
    else if (strcmp((const char *)buf, "DampedSpring") == 0)
    {
        muscle = new DampedSpringMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"UnloadedLength"));
        ((DampedSpringMuscle *)muscle)->SetUnloadedLength(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SpringConstant"));
        ((DampedSpringMuscle *)muscle)->SetSpringConstant(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Area"));
        ((DampedSpringMuscle *)muscle)->SetArea(Util::Double(buf));
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Damping"));
        ((DampedSpringMuscle *)muscle)->SetDamping(Util::Double(buf));

    }
    else if (strcmp((const char *)buf, "UmbergerGerritsenMartin") == 0)
    {
        muscle = new UGMMuscle(strap);
        muscle->SetName(muscleID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PCA"));
        double PCSA = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FibreLength"));
        double optimumLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TendonLength"));
        double tendonLength = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainModel"));
        UGMMuscle::StrainModel serialStrainModel;
        if (strcmp((const char *)buf, "Linear") == 0)
            serialStrainModel = UGMMuscle::linear;
        else if (strcmp((const char *)buf, "Square") == 0)
            serialStrainModel = UGMMuscle::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SerialStrainAtFmax"));
        double serialStrainAtFmax = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainModel"));
        UGMMuscle::StrainModel parallelStrainModel;
        if (strcmp((const char *)buf, "Linear") == 0)
            parallelStrainModel = UGMMuscle::linear;
        else if (strcmp((const char *)buf, "Square") == 0)
            parallelStrainModel = UGMMuscle::square;
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ParallelStrainAtFmax"));
        double parallelStrainAtFmax = Util::Double(buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ForcePerUnitArea"));
        double forcePerUnitArea = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VMaxFactor"));
        double vMaxFactor = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MuscleDensity"));
        double muscleDensity = Util::Double(buf);
        ((UGMMuscle *)muscle)->SetModellingConstants(forcePerUnitArea, vMaxFactor, muscleDensity);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"FastTwitchProportion"));
        ((UGMMuscle *)muscle)->SetFibreComposition(Util::Double(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Width"));
        double muscleWidth = Util::Double(buf);
        ((UGMMuscle *)muscle)->SetMuscleGeometry(PCSA, optimumLength, muscleWidth, tendonLength,
                                                 serialStrainModel, serialStrainAtFmax,
                                                 parallelStrainModel, parallelStrainAtFmax);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Aerobic"));
        ((UGMMuscle *)muscle)->SetAerobic(Util::Bool(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"AllowReverseWork"));
        ((UGMMuscle *)muscle)->AllowReverseWork(Util::Bool(buf));

    }
    else
    {
        std::cerr << "Unrecognised Muscle Type:" << buf << "\n";
        throw __LINE__;
    }

#ifdef USE_QT
    muscle->SetColour(m_Interface.StrapColour);
    muscle->SetForceColour(m_Interface.StrapForceColour);
#endif

    m_MuscleList[*muscle->GetName()] = muscle;

}

void Simulation::ParseDriver(xmlNodePtr cur)
{
    char *buf;
    int count;

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "Cyclic") == 0)
    {
        CyclicDriver *cyclicDriver = new CyclicDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = cyclicDriver;
        cyclicDriver->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs"));
        count = DataFile::CountTokens((char *)buf);
        Util::Double(buf, count, m_DoubleList);
        cyclicDriver->SetValueDurationPairs(count, m_DoubleList);
        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        if (m_MuscleList.find((const char *)buf) != m_MuscleList.end()) cyclicDriver->SetTarget(m_MuscleList[(const char *)buf]);
        else if (m_ControllerList.find((const char *)buf) != m_ControllerList.end()) cyclicDriver->SetTarget(m_ControllerList[(const char *)buf]);
        else throw __LINE__;

        buf = DoXmlGetProp(cur, (const xmlChar *)"PhaseDelay");
        if (buf)
        {
            cyclicDriver->SetPhaseDelay(Util::Double(buf));
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"DriverRange");
        if (buf)
        {
            count = DataFile::CountTokens((char *)buf);
            if (count >= 2)
            {
                Util::Double(buf, count, m_DoubleList);
                cyclicDriver->SetMinMax(m_DoubleList[0],m_DoubleList[1]);
            }
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"LinearInterpolation");
        if (buf)
        {
            cyclicDriver->SetInterp(Util::Bool(buf));
        }

        // assumes all cycles are the same duration
        m_CycleTime = cyclicDriver->GetCycleTime();
    }
    else if (strcmp((const char *)buf, "Step") == 0)
    {
        StepDriver *stepDriver = new StepDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = stepDriver;
        stepDriver->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs"));
        count = DataFile::CountTokens((char *)buf);
        Util::Double(buf, count, m_DoubleList);
        stepDriver->SetValueDurationPairs(count, m_DoubleList);
        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        if (m_MuscleList.find((const char *)buf) != m_MuscleList.end()) stepDriver->SetTarget(m_MuscleList[(const char *)buf]);
        else if (m_ControllerList.find((const char *)buf) != m_ControllerList.end()) stepDriver->SetTarget(m_ControllerList[(const char *)buf]);
        else throw __LINE__;

        buf = DoXmlGetProp(cur, (const xmlChar *)"DriverRange");
        if (buf)
        {
            count = DataFile::CountTokens((char *)buf);
            if (count >= 2)
            {
                Util::Double(buf, count, m_DoubleList);
                stepDriver->SetMinMax(m_DoubleList[0],m_DoubleList[1]);
            }
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"LinearInterpolation");
        if (buf)
        {
            stepDriver->SetInterp(Util::Bool(buf));
        }

    }
    else if (strcmp((const char *)buf, "BoxCar") == 0)
    {
        BoxCarDriver *BoxCarDriver1 = new BoxCarDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = BoxCarDriver1;
        BoxCarDriver1->SetName((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        if (m_MuscleList.find((const char *)buf) != m_MuscleList.end()) BoxCarDriver1->SetTarget(m_MuscleList[(const char *)buf]);
        else if (m_ControllerList.find((const char *)buf) != m_ControllerList.end()) BoxCarDriver1->SetTarget(m_ControllerList[(const char *)buf]);
        else throw __LINE__;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CycleTime"));
        double CycleTime = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Delay"));
        double Delay = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Width"));
        double Width = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Height"));
        double Height = Util::Double(buf);
        BoxCarDriver1->SetBoxCarParameters(CycleTime, Delay, Width, Height);

        buf = DoXmlGetProp(cur, (const xmlChar *)"DriverRange");
        if (buf)
        {
            count = DataFile::CountTokens((char *)buf);
            if (count >= 2)
            {
                Util::Double(buf, count, m_DoubleList);
                BoxCarDriver1->SetMinMax(m_DoubleList[0],m_DoubleList[1]);
            }
        }

        // assumes all cycles are the same duration
        m_CycleTime = BoxCarDriver1->GetCycleTime();
    }
    else if (strcmp((const char *)buf, "StackedBoxCar") == 0)
    {
        StackedBoxCarDriver *StackedBoxCarDriver1 = new StackedBoxCarDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = StackedBoxCarDriver1;
        StackedBoxCarDriver1->SetName((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        if (m_MuscleList.find((const char *)buf) != m_MuscleList.end()) StackedBoxCarDriver1->SetTarget(m_MuscleList[(const char *)buf]);
        else if (m_ControllerList.find((const char *)buf) != m_ControllerList.end()) StackedBoxCarDriver1->SetTarget(m_ControllerList[(const char *)buf]);
        else throw __LINE__;

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"StackSize"));
        int StackSize = Util::Int(buf);
        StackedBoxCarDriver1->SetStackSize(StackSize);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"CycleTimes"));
        if (DataFile::CountTokens((char *)buf) != StackSize) throw __LINE__;
        Util::Double(buf, StackSize, m_DoubleList);
        StackedBoxCarDriver1->SetCycleTimes(m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Delays"));
        if (DataFile::CountTokens((char *)buf) != StackSize) throw __LINE__;
        Util::Double(buf, StackSize, m_DoubleList);
        StackedBoxCarDriver1->SetDelays(m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Widths"));
        if (DataFile::CountTokens((char *)buf) != StackSize) throw __LINE__;
        Util::Double(buf, StackSize, m_DoubleList);
        StackedBoxCarDriver1->SetWidths(m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Heights"));
        if (DataFile::CountTokens((char *)buf) != StackSize) throw __LINE__;
        Util::Double(buf, StackSize, m_DoubleList);
        StackedBoxCarDriver1->SetHeights(m_DoubleList);

        buf = DoXmlGetProp(cur, (const xmlChar *)"DriverRange");
        if (buf)
        {
            count = DataFile::CountTokens((char *)buf);
            if (count >= 2)
            {
                Util::Double(buf, count, m_DoubleList);
                StackedBoxCarDriver1->SetMinMax(m_DoubleList[0],m_DoubleList[1]);
            }
        }

        // assumes all cycles are the same duration
        std::vector<double> *CycleTimes = StackedBoxCarDriver1->GetCycleTimes();
        m_CycleTime = CycleTimes->at(0);
    }
    else if (strcmp((const char *)buf, "Fixed") == 0)
    {
        FixedDriver *fixedDriver = new FixedDriver();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        m_DriverList[(const char *)buf] = fixedDriver;
        fixedDriver->SetName((const char *)buf);
        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        if (m_MuscleList.find((const char *)buf) != m_MuscleList.end()) fixedDriver->SetTarget(m_MuscleList[(const char *)buf]);
        else if (m_ControllerList.find((const char *)buf) != m_ControllerList.end()) fixedDriver->SetTarget(m_ControllerList[(const char *)buf]);
        else throw __LINE__;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Value"));
        fixedDriver->SetValue(Util::Double(buf));
    }
    else
    {
        throw __LINE__;
    }


}


void Simulation::ParseDataTarget(xmlNodePtr cur)
{
    char *buf;
    int count, i;
    DataTarget *dataTarget;
    buf = DoXmlGetProp(cur, (const xmlChar *)"Type");
    char defType[] = "Scalar";
    if (buf == 0) buf = defType;

    if (strcmp((const char *)buf, "Scalar") == 0)
    {
        DataTargetScalar *dataTargetScalar = new DataTargetScalar();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetScalar->SetName((const char *)buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DataType"));
        if (strcmp((const char *)buf, "XP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XP);
        else if (strcmp((const char *)buf, "YP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YP);
        else if (strcmp((const char *)buf, "ZP") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZP);
        else if (strcmp((const char *)buf, "Q0") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q0);
        else if (strcmp((const char *)buf, "Q1") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q1);
        else if (strcmp((const char *)buf, "Q2") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q2);
        else if (strcmp((const char *)buf, "Q3") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Q3);
        else if (strcmp((const char *)buf, "XV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XV);
        else if (strcmp((const char *)buf, "YV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YV);
        else if (strcmp((const char *)buf, "ZV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZV);
        else if (strcmp((const char *)buf, "XRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::XRV);
        else if (strcmp((const char *)buf, "YRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::YRV);
        else if (strcmp((const char *)buf, "ZRV") == 0) dataTargetScalar->SetDataType(DataTargetScalar::ZRV);
        else if (strcmp((const char *)buf, "Angle") == 0) dataTargetScalar->SetDataType(DataTargetScalar::Angle);
        else if (strcmp((const char *)buf, "MetabolicEnergy") == 0) dataTargetScalar->SetDataType(DataTargetScalar::MetabolicEnergy);
        else if (strcmp((const char *)buf, "MechanicalEnergy") == 0) dataTargetScalar->SetDataType(DataTargetScalar::MechanicalEnergy);
        else throw(__LINE__);

        if (dataTargetScalar->GetDataType() != DataTargetScalar::MetabolicEnergy && dataTargetScalar->GetDataType() != DataTargetScalar::MechanicalEnergy)
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
            if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
            std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
            if (iterBody != m_BodyList.end()) dataTargetScalar->SetTarget(iterBody->second);
            std::map<std::string, Joint *>::const_iterator iterJoint = m_JointList.find(buf);
            if (iterJoint != m_JointList.end()) dataTargetScalar->SetTarget(iterJoint->second);
            std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
            if (iterGeom != m_GeomList.end()) dataTargetScalar->SetTarget(iterGeom->second);
            std::map<std::string, Reporter *>::const_iterator iterReporter = m_ReporterList.find(buf);
            if (iterReporter != m_ReporterList.end()) dataTargetScalar->SetTarget(iterReporter->second);
            THROWIFZERO(dataTargetScalar->GetTarget());
        }

        // check presence of Intercept
        buf = DoXmlGetProp(cur, (const xmlChar *)"Intercept");
        if (buf)
        {
            dataTargetScalar->SetIntercept(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetScalar->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetScalar->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetScalar->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetScalar->SetAbortThreshold(Util::Double(buf));
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"DurationValuePairs");
        if (buf)
        {
            count = DataFile::ReturnTokens(buf, m_BufferPtrs, m_BufferSize);

            Util::Double(buf, count, m_DoubleList);
            count = count / 2;
            double *times = new double[count];
            double *values = new double[count];
            for (i = 0; i < count; i++)
            {
                times[i] = Util::Double(m_BufferPtrs[i * 2]);
                if (dataTargetScalar->GetDataType() == DataTargetScalar::Angle)
                    values[i] = Util::GetAngle(m_BufferPtrs[i * 2 + 1]);
                else
                    values[i] = Util::Double(m_BufferPtrs[i * 2 + 1]);
            }
            dataTargetScalar->SetTargetTimes(count, times);
            dataTargetScalar->SetTargetValues(count, values);
            delete [] times;
            delete [] values;
        }
        else
        {
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
            count = DataFile::CountTokens((char *)buf);
            Util::Double(buf, count, m_DoubleList);
            dataTargetScalar->SetTargetTimes(count, m_DoubleList);
            THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
            count = DataFile::CountTokens((char *)buf);
            Util::Double(buf, count, m_DoubleList);
            dataTargetScalar->SetTargetValues(count, m_DoubleList);
        }
        dataTarget = dataTargetScalar;
    }
    else if (strcmp((const char *)buf, "Quaternion") == 0)
    {
        DataTargetQuaternion *dataTargetQuaternion = new DataTargetQuaternion();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetQuaternion->SetName((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
        if (iterBody != m_BodyList.end()) dataTargetQuaternion->SetTarget(iterBody->second);
        std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
        if (iterGeom != m_GeomList.end()) dataTargetQuaternion->SetTarget(iterGeom->second);
        std::map<std::string, Reporter *>::const_iterator iterReporter = m_ReporterList.find(buf);
        if (iterReporter != m_ReporterList.end()) dataTargetQuaternion->SetTarget(iterReporter->second);
        THROWIFZERO(dataTargetQuaternion->GetTarget());

        // check presence of Intercept
        buf = DoXmlGetProp(cur, (const xmlChar *)"Intercept");
        if (buf)
        {
            dataTargetQuaternion->SetIntercept(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetQuaternion->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetQuaternion->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetQuaternion->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetQuaternion->SetAbortThreshold(Util::Double(buf));
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
        int timeCount = DataFile::CountTokens((char *)buf);
        Util::Double(buf, timeCount, m_DoubleList);
        dataTargetQuaternion->SetTargetTimes(timeCount, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
        dataTargetQuaternion->SetTargetValues((char *)buf);

        dataTarget = dataTargetQuaternion;
    }
    else if (strcmp((const char *)buf, "Vector") == 0)
    {
        DataTargetVector *dataTargetVector = new DataTargetVector();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        dataTargetVector->SetName((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Target");
        if (buf == 0) THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetID"));
        std::map<std::string, Body *>::const_iterator iterBody = m_BodyList.find(buf);
        if (iterBody != m_BodyList.end()) dataTargetVector->SetTarget(iterBody->second);
        std::map<std::string, Joint *>::const_iterator iterJoint = m_JointList.find(buf);
        if (iterJoint != m_JointList.end()) dataTargetVector->SetTarget(iterJoint->second);
        std::map<std::string, Geom *>::const_iterator iterGeom = m_GeomList.find(buf);
        if (iterGeom != m_GeomList.end()) dataTargetVector->SetTarget(iterGeom->second);
        std::map<std::string, Reporter *>::const_iterator iterReporter = m_ReporterList.find(buf);
        if (iterReporter != m_ReporterList.end()) dataTargetVector->SetTarget(iterReporter->second);
        THROWIFZERO(dataTargetVector->GetTarget());

        // check presence of Intercept
        buf = DoXmlGetProp(cur, (const xmlChar *)"Intercept");
        if (buf)
        {
            dataTargetVector->SetIntercept(Util::Double(buf));
        }
        // check presence of Slope
        buf = DoXmlGetProp(cur, (const xmlChar *)"Slope");
        if (buf)
        {
            dataTargetVector->SetSlope(Util::Double(buf));
        }
        // check presence of MatchType
        buf = DoXmlGetProp(cur, (const xmlChar *)"MatchType");
        if (buf)
        {
            if (strcmp((const char *)buf, "Linear") == 0) dataTargetVector->SetMatchType(DataTarget::linear);
            else if (strcmp((const char *)buf, "Square") == 0) dataTargetVector->SetMatchType(DataTarget::square);
            else throw(__LINE__);
        }
        // check presence of AbortThreshold
        buf = DoXmlGetProp(cur, (const xmlChar *)"AbortThreshold");
        if (buf)
        {
            dataTargetVector->SetAbortThreshold(Util::Double(buf));
        }

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetTimes"));
        int timeCount = DataFile::CountTokens((char *)buf);
        Util::Double(buf, timeCount, m_DoubleList);
        dataTargetVector->SetTargetTimes(timeCount, m_DoubleList);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"TargetValues"));
        dataTargetVector->SetTargetValues((char *)buf);

#ifdef USE_QT
        buf = DoXmlGetProp(cur, (const xmlChar *)"Visible");
        if (buf)
        {
            dataTargetVector->SetVisible(Util::Double(buf));
        }
        Colour dataTargetColour = m_Interface.DataTargetColour;
        dataTargetVector->SetColour(dataTargetColour);
        dataTargetVector->SetRadius(m_Interface.DataTargetRadius);
#endif

        dataTarget = dataTargetVector;
    }

    else
    {
        throw __LINE__;
    }

    m_DataTargetList[*dataTarget->GetName()] = dataTarget;

}

void Simulation::ParseIOControl(xmlNodePtr cur)
{
    char *buf;
    buf = DoXmlGetProp(cur, (const xmlChar *)"SanityCheckLeft");
    if (buf)
    {
        m_SanityCheckLeft = buf;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SanityCheckRight"));
        m_SanityCheckRight = buf;
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"SanityCheckAxis"));
        if (strcmp((const char *)buf, "x") == 0 || strcmp((const char *)buf, "X") == 0) m_SanityCheckAxis = XAxis;
        if (strcmp((const char *)buf, "y") == 0 || strcmp((const char *)buf, "Y") == 0) m_SanityCheckAxis = YAxis;
        if (strcmp((const char *)buf, "z") == 0 || strcmp((const char *)buf, "Z") == 0) m_SanityCheckAxis = ZAxis;
    }

}

void Simulation::ParseMarker(xmlNodePtr cur)
{
    char *buf;
    Marker *marker = new Marker();

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
    marker->SetName((const char *)buf);

    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
    if (strcmp(buf, "World"))
    {
        dBodyID bodyID = m_BodyList[(const char *)buf]->GetBodyID();
        marker->SetBody(bodyID);
    }

    buf = DoXmlGetProp(cur, (const xmlChar *)"Position");
    if (buf)
        marker->SetPosition((const char *)buf);

    buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion");
    if (buf)
        marker->SetQuaternion((const char *)buf);

#ifdef USE_QT
    marker->SetColour(m_Interface.MarkerColour);
    marker->SetRadius(m_Interface.MarkerRadius);
#endif

    m_MarkerList[*marker->GetName()] = marker;
}

void Simulation::ParseReporter(xmlNodePtr cur)
{
    char *buf;
    Reporter *reporter;
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "Torque") == 0)
    {
        TorqueReporter *torqueReporter = new TorqueReporter();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        torqueReporter->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
        torqueReporter->SetBody(m_BodyList[(const char *)buf]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MuscleID"));
        torqueReporter->SetMuscle(m_MuscleList[(const char *)buf]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"PivotPoint"));
        torqueReporter->SetPivotPoint((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Axis");
        if (buf)
                torqueReporter->SetAxis((const char *)buf);

        reporter = torqueReporter;
    }

    else if (strcmp((const char *)buf, "Position") == 0)
    {
        PositionReporter *positionReporter = new PositionReporter();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        positionReporter->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
        Body *bodyID = m_BodyList[(const char *)buf];
        positionReporter->SetBody(bodyID);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Position");
        if (buf)
            positionReporter->SetPosition((const char *)buf);

        buf = DoXmlGetProp(cur, (const xmlChar *)"Quaternion");
        if (buf)
            positionReporter->SetQuaternion((const char *)buf);

    #ifdef USE_QT
        positionReporter->SetColour(m_Interface.ReporterColour);
        positionReporter->SetRadius(m_Interface.ReporterRadius);
    #endif

        reporter = positionReporter;
    }

    else if (strcmp((const char *)buf, "SwingClearanceAbort") == 0)
    {
        SwingClearanceAbortReporter *swingClearanceAbortReporter = new SwingClearanceAbortReporter();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        swingClearanceAbortReporter->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"BodyID"));
        Body *bodyID = m_BodyList[(const char *)buf];
        swingClearanceAbortReporter->SetBody(bodyID);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Position"));
        swingClearanceAbortReporter->SetPosition((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"HeightThreshold"));
        swingClearanceAbortReporter->SetHeightThreshold(Util::Double(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"VelocityThreshold"));
        swingClearanceAbortReporter->SetVelocityThreshold(Util::Double(buf));

        // defaults to Z up
        buf = DoXmlGetProp(cur, (const xmlChar *)"UpAxis");
        if (buf) swingClearanceAbortReporter->SetUpAxis(buf);

        // optionally specify a direction axis for the velocity test
        // gets normalised internally
        buf = DoXmlGetProp(cur, (const xmlChar *)"DirectionAxis");
        if (buf) swingClearanceAbortReporter->SetDirectionAxis(buf);

    #ifdef USE_QT
        swingClearanceAbortReporter->SetColour(m_Interface.ReporterColour);
        swingClearanceAbortReporter->SetRadius(m_Interface.ReporterRadius);
    #endif

        reporter = swingClearanceAbortReporter;
    }

    else
    {
        throw __LINE__;
    }

    m_ReporterList[*reporter->GetName()] = reporter;
}

void Simulation::ParseController(xmlNodePtr cur)
{
    char *buf;
    Controller *controller;
    THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Type"));

    if (strcmp((const char *)buf, "PIDMuscleLength") == 0)
    {
        PIDMuscleLength *pidMuscleLength = new PIDMuscleLength();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        pidMuscleLength->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MuscleID"));
        pidMuscleLength->SetMuscle(m_MuscleList[(const char *)buf]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"NominalLength"));
        pidMuscleLength->SetNominalLength(Util::Double(buf));

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Kp"));
        double Kp = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Ki"));
        double Ki = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Kd"));
        double Kd = Util::Double(buf);
        pidMuscleLength->SetPID(Kp, Ki, Kd);

        controller = pidMuscleLength;
    }

    else if (strcmp((const char *)buf, "PIDTargetMatch") == 0)
    {
        PIDTargetMatch *pidTargetMatch = new PIDTargetMatch();
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"ID"));
        pidTargetMatch->SetName((const char *)buf);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"MuscleID"));
        pidTargetMatch->SetMuscle(m_MuscleList[(const char *)buf]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"DataTargetID"));
        pidTargetMatch->SetDataTarget(m_DataTargetList[(const char *)buf]);

        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Kp"));
        double Kp = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Ki"));
        double Ki = Util::Double(buf);
        THROWIFZERO(buf = DoXmlGetProp(cur, (const xmlChar *)"Kd"));
        double Kd = Util::Double(buf);
        pidTargetMatch->SetPID(Kp, Ki, Kd);

        controller = pidTargetMatch;
    }

    else
    {
        throw __LINE__;
    }

    m_ControllerList[*controller->GetName()] = controller;
}

void Simulation::ParseWarehouse(xmlNodePtr cur)
{
}

#ifdef USE_QT
void Simulation::ParseInterface(xmlNodePtr cur)
{
    char  *buf;

    buf = DoXmlGetProp(cur, (const xmlChar *)"TrackBodyID");
    if (buf)
    {
        m_Interface.TrackBodyID = (char *)buf;
    }

#if defined(USE_QT)
    if (m_UseProgramInterface)
    {
        const Preferences *prefs = m_MainWindow->GetPreferences();
        m_Interface.EnvironmentAxisSize[0] = prefs->EnvironmentX;
        m_Interface.EnvironmentAxisSize[1] = prefs->EnvironmentY;
        m_Interface.EnvironmentAxisSize[2] = prefs->EnvironmentZ;
        m_Interface.BodyAxisSize[0] = prefs->BodyX;
        m_Interface.BodyAxisSize[1] = prefs->BodyY;
        m_Interface.BodyAxisSize[2] = prefs->BodyZ;
        m_Interface.JointAxisSize[0] = prefs->JointX;
        m_Interface.JointAxisSize[1] = prefs->JointY;
        m_Interface.JointAxisSize[2] = prefs->JointZ;
        m_Interface.GeomAxisSize[0] = prefs->GeomX;
        m_Interface.GeomAxisSize[1] = prefs->GeomY;
        m_Interface.GeomAxisSize[2] = prefs->GeomZ;
        m_Interface.GeomForceScale = prefs->GeomForceScale;
        m_Interface.GeomForceRadius = prefs->GeomForceRadius;
        m_Interface.StrapRadius = prefs->StrapRadius;
        m_Interface.StrapForceRadius = prefs->StrapForceRadius;
        m_Interface.StrapForceScale = prefs->StrapForceScale;
        m_Interface.StrapCylinderLength = prefs->StrapCylinderLength;
        m_Interface.ReporterRadius = prefs->ReporterX;
        m_Interface.DataTargetRadius = prefs->DataTargetX;

        // marker to do

        QColor EnvironmentColour = prefs->EnvironmentColour;
        QColor BodyColour = prefs->BodyColour;
        QColor JointColour = prefs->JointColour;
        QColor GeomColour = prefs->GeomColour;
        QColor GeomForceColour = prefs->GeomForceColour;
        QColor StrapColour = prefs->StrapColour;
        QColor StrapForceColour = prefs->StrapForceColour;
        QColor StrapCylinderColour = prefs->StrapCylinderColour;
        QColor ReporterColour = prefs->ReporterColour;
        QColor DataTargetColour = prefs->DataTargetColour;

        m_Interface.EnvironmentColour.SetColour(EnvironmentColour.redF(), EnvironmentColour.greenF(), EnvironmentColour.blueF(), EnvironmentColour.alphaF());
        m_Interface.BodyColour.SetColour(BodyColour.redF(), BodyColour.greenF(), BodyColour.blueF(), BodyColour.alphaF());
        m_Interface.JointColour.SetColour(JointColour.redF(), JointColour.greenF(), JointColour.blueF(), JointColour.alphaF());
        m_Interface.GeomColour.SetColour(GeomColour.redF(), GeomColour.greenF(), GeomColour.blueF(), GeomColour.alphaF());
        m_Interface.GeomForceColour.SetColour(GeomForceColour.redF(), GeomForceColour.greenF(), GeomForceColour.blueF(), GeomForceColour.alphaF());
        m_Interface.StrapColour.SetColour(StrapColour.redF(), StrapColour.greenF(), StrapColour.blueF(), StrapColour.alphaF());
        m_Interface.StrapForceColour.SetColour(StrapForceColour.redF(), StrapForceColour.greenF(), StrapForceColour.blueF(), StrapForceColour.alphaF());
        m_Interface.StrapCylinderColour.SetColour(StrapCylinderColour.redF(), StrapCylinderColour.greenF(), StrapCylinderColour.blueF(), StrapCylinderColour.alphaF());
        m_Interface.ReporterColour.SetColour(ReporterColour.redF(), ReporterColour.greenF(), ReporterColour.blueF(), ReporterColour.alphaF());
        m_Interface.DataTargetColour.SetColour(DataTargetColour.redF(), DataTargetColour.greenF(), DataTargetColour.blueF(), DataTargetColour.alphaF());

    }
    else
#endif
    {

        buf = DoXmlGetProp(cur, (const xmlChar *)"EnvironmentAxisSize");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            m_Interface.EnvironmentAxisSize[0] = m_DoubleList[0];
            m_Interface.EnvironmentAxisSize[1] = m_DoubleList[1];
            m_Interface.EnvironmentAxisSize[2] = m_DoubleList[2];
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"EnvironmentColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.EnvironmentColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"BodyAxisSize");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            m_Interface.BodyAxisSize[0] = m_DoubleList[0];
            m_Interface.BodyAxisSize[1] = m_DoubleList[1];
            m_Interface.BodyAxisSize[2] = m_DoubleList[2];
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"BodyColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.BodyColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"JointAxisSize");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            m_Interface.JointAxisSize[0] = m_DoubleList[0];
            m_Interface.JointAxisSize[1] = m_DoubleList[1];
            m_Interface.JointAxisSize[2] = m_DoubleList[2];
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"JointColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.JointColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"GeomColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.GeomColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.StrapColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapRadius");
        if (buf)
        {
            m_Interface.StrapRadius = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.StrapForceColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceRadius");
        if (buf)
        {
            m_Interface.StrapForceRadius = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapForceScale");
        if (buf)
        {
            m_Interface.StrapForceScale = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapCylinderColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.StrapCylinderColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"StrapCylinderLength");
        if (buf)
        {
            m_Interface.StrapCylinderLength = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"GeomAxisSize");
        if (buf)
        {
            Util::Double(buf, 3, m_DoubleList);
            m_Interface.GeomAxisSize[0] = m_DoubleList[0];
            m_Interface.GeomAxisSize[1] = m_DoubleList[1];
            m_Interface.GeomAxisSize[2] = m_DoubleList[2];
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ContactColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.GeomColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.GeomForceColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceRadius");
        if (buf)
        {
            m_Interface.GeomForceRadius = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ContactForceScale");
        if (buf)
        {
            m_Interface.GeomForceScale = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"MarkerColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.MarkerColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"MarkerRadius");
        if (buf)
        {
            m_Interface.MarkerRadius = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ReporterColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.ReporterColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"ReporterRadius");
        if (buf)
        {
            m_Interface.ReporterRadius = Util::Double(buf);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"DataTargetColour");
        if (buf)
        {
            Util::Double(buf, 4, m_DoubleList);
            m_Interface.DataTargetColour.SetColour(m_DoubleList[0], m_DoubleList[1], m_DoubleList[2], m_DoubleList[3]);
        }

        buf = DoXmlGetProp(cur, (const xmlChar *)"DataTargetRadius");
        if (buf)
        {
            m_Interface.DataTargetRadius = Util::Double(buf);
        }
    }

}
#endif

// function to produce a file of link kinematics in tab delimited format

void
        Simulation::OutputKinematics()
{
    const double *p;
    std::map<std::string, Body *>::const_iterator iter1;

    // first time through output the column headings
    if (m_OutputKinematicsFirstTimeFlag)
    {
        m_OutputKinematicsFile.open(m_OutputKinematicsFilename.c_str());

        m_OutputKinematicsFile << "time\t";
        for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
        {
            m_OutputKinematicsFile << *iter1->second->GetName() << "_X\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Y\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Z\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q0\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q1\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q2\t";
            m_OutputKinematicsFile << *iter1->second->GetName() << "_Q3\t";
        }
        m_OutputKinematicsFirstTimeFlag = false;
        m_OutputKinematicsFile << "\n";
    }

    // start by outputting the simulation time
    m_OutputKinematicsFile << m_SimulationTime << "\t";

    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        p = iter1->second->GetPosition();
        m_OutputKinematicsFile << p[0] << "\t";
        m_OutputKinematicsFile << p[1] << "\t";
        m_OutputKinematicsFile << p[2] << "\t";

        p = iter1->second->GetQuaternion();
        m_OutputKinematicsFile << p[0] << "\t";
        m_OutputKinematicsFile << p[1] << "\t";
        m_OutputKinematicsFile << p[2] << "\t";
        m_OutputKinematicsFile << p[3] << "\t";
    }
    m_OutputKinematicsFile << "\n";
}

// function to input a file of link kinematics in tab delimited format
// requires the data in the order produced by OutputKinematics

void
        Simulation::InputKinematics()
{
    std::map<std::string, Body *>::const_iterator iter1;
    double v;
    dQuaternion q;
    double x, y, z;

    m_InputKinematicsFile.SetExitOnError(true);
    if (m_InputKinematicsFile.ReadNext(&v))
    {
        std::cerr << "End of kinematics file\n";
        return;
    }
    m_InputKinematicsFile.SetExitOnError(true);
    m_SimulationTime = v;

    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++)
    {
        m_InputKinematicsFile.ReadNext(&v); x = v;
        m_InputKinematicsFile.ReadNext(&v); y = v;
        m_InputKinematicsFile.ReadNext(&v); z = v;
        iter1->second->SetPosition(x, y, z);

        m_InputKinematicsFile.ReadNext(&v); q[0] = v;
        m_InputKinematicsFile.ReadNext(&v); q[1] = v;
        m_InputKinematicsFile.ReadNext(&v); q[2] = v;
        m_InputKinematicsFile.ReadNext(&v); q[3] = v;
        iter1->second->SetQuaternion(q[0], q[1], q[2], q[3]);
    }
}

// function to produce a file of link kinematics in tab delimited format
// plus additional muscle activation information for use in gait warehousing

void Simulation::OutputWarehouse()
{
    if (m_OutputWarehouseAsText)
    {
        /* file format is \t separated and \n end of line
         *
         * numDrivers name0 name1 name2 ... numBodies name0 name1 name2...
         * time act0 act1 act2 ... x0 y0 z0 angle0 xaxis0 yaxis0 zaxis0 xv0 yv0 zv0 xav0 yav0 zav0 ...
         *
         */

        /* the first defined body is defined as its world coordinates and the rest are relative to the master body */

        // first time through output the column headings
        if (m_OutputWarehouseLastTime < 0)
        {
            m_OutputWarehouseFile.open(m_OutputWarehouseFilename.c_str());

            m_OutputWarehouseFile << m_DriverList.size();
            for (std::map<std::string, Driver *>::const_iterator iter = m_DriverList.begin(); iter != m_DriverList.end(); iter++) m_OutputWarehouseFile << "\t\"" << *iter->second->GetName() << "\"";
            m_OutputWarehouseFile << "\t" << m_BodyList.size();
            m_OutputWarehouseFile << "\t" << *m_BodyList[m_DistanceTravelledBodyIDName]->GetName();
            for (std::map<std::string, Body *>::const_iterator iter = m_BodyList.begin(); iter != m_BodyList.end(); iter++)
                if (*iter->second->GetName() != m_DistanceTravelledBodyIDName) m_OutputWarehouseFile << "\t\"" << *iter->second->GetName() << "\"";
            m_OutputWarehouseFile << "\n";
        }

        m_OutputWarehouseLastTime = m_SimulationTime;
        // simulation time
        m_OutputWarehouseFile << m_SimulationTime;
        // driver activations
        for (std::map<std::string, Driver *>::const_iterator iter = m_DriverList.begin(); iter != m_DriverList.end(); iter++) m_OutputWarehouseFile << "\t" << iter->second->GetValue(m_SimulationTime);
        // output the root body (m_DistanceTravelledBodyIDName)
        Body *rootBody = m_BodyList[m_DistanceTravelledBodyIDName];
        pgd::Vector pos, vel, avel;
        pgd::Quaternion quat;
        rootBody->GetRelativePosition(0, &pos);
        rootBody->GetRelativeQuaternion(0, &quat);
        rootBody->GetRelativeLinearVelocity(0, &vel);
        rootBody->GetRelativeAngularVelocity(0, &avel);
        double angle = QGetAngle(quat);
        pgd::Vector axis = QGetAxis(quat);
        m_OutputWarehouseFile << "\t" << pos.x << "\t" << pos.y << "\t" << pos.z;
        m_OutputWarehouseFile << "\t" << angle << "\t" << axis.x << "\t" << axis.y << "\t" << axis.z ;
        m_OutputWarehouseFile << "\t" << vel.x << "\t" << vel.y << "\t" << vel.z;
        m_OutputWarehouseFile << "\t" << avel.x << "\t" << avel.y << "\t" << avel.z;
        // and now the rest of the bodies
        for (std::map<std::string, Body *>::const_iterator iter = m_BodyList.begin(); iter != m_BodyList.end(); iter++)
        {
            if (*iter->second->GetName() != m_DistanceTravelledBodyIDName)
            {
                iter->second->GetRelativePosition(rootBody, &pos);
                iter->second->GetRelativeQuaternion(rootBody, &quat);
                iter->second->GetRelativeLinearVelocity(rootBody, &vel);
                iter->second->GetRelativeAngularVelocity(rootBody, &avel);
                angle = QGetAngle(quat);
                axis = QGetAxis(quat);
                m_OutputWarehouseFile << "\t" << pos.x << "\t" << pos.y << "\t" << pos.z;
                m_OutputWarehouseFile << "\t" << angle << "\t" << axis.x << "\t" << axis.y << "\t" << axis.z ;
                m_OutputWarehouseFile << "\t" << vel.x << "\t" << vel.y << "\t" << vel.z;
                m_OutputWarehouseFile << "\t" << avel.x << "\t" << avel.y << "\t" << avel.z;
            }
        }
        m_OutputWarehouseFile << "\n";
    }
    else
    {
        /* file format is binary
         *
         * int 0
         * int numDrivers int lenName0 name0 int lenName1 name1 ...
         * int numBodies int lenName0 name0 int lenName1 name1 ...
         * double time double act0 double act1 double act2 ...
         * double x0 double y0 double z0 double angle0 double xaxis0 double yaxis0 double zaxis0 double xv0 double yv0 double zv0 double xav0 double yav0 double zav0 ...
         *
         */

        /* the first defined body is defined as its world coordinates and the rest are relative to the master body */

        // first time through output the column headings
        if (m_OutputWarehouseLastTime < 0)
        {
            m_OutputWarehouseFile.open(m_OutputWarehouseFilename.c_str(), std::ios::binary);
            Util::BinaryOutput(m_OutputWarehouseFile, (uint32_t)0);
            Util::BinaryOutput(m_OutputWarehouseFile, (uint32_t)m_DriverList.size());
            for (std::map<std::string, Driver *>::const_iterator iter = m_DriverList.begin(); iter != m_DriverList.end(); iter++) Util::BinaryOutput(m_OutputWarehouseFile, *iter->second->GetName());
            Util::BinaryOutput(m_OutputWarehouseFile, (uint32_t)m_BodyList.size());
            Util::BinaryOutput(m_OutputWarehouseFile, *m_BodyList[m_DistanceTravelledBodyIDName]->GetName());
            for (std::map<std::string, Body *>::const_iterator iter = m_BodyList.begin(); iter != m_BodyList.end(); iter++)
                if (*iter->second->GetName() != m_DistanceTravelledBodyIDName) Util::BinaryOutput(m_OutputWarehouseFile, *iter->second->GetName());
        }

        m_OutputWarehouseLastTime = m_SimulationTime;
        // simulation time
        Util::BinaryOutput(m_OutputWarehouseFile, m_SimulationTime);
        // driver activations
        for (std::map<std::string, Driver *>::const_iterator iter = m_DriverList.begin(); iter != m_DriverList.end(); iter++) Util::BinaryOutput(m_OutputWarehouseFile, iter->second->GetValue(m_SimulationTime));
        // output the root body (m_DistanceTravelledBodyIDName)
        Body *rootBody = m_BodyList[m_DistanceTravelledBodyIDName];
        pgd::Vector pos, vel, avel;
        pgd::Quaternion quat;
        rootBody->GetRelativePosition(0, &pos);
        rootBody->GetRelativeQuaternion(0, &quat);
        rootBody->GetRelativeLinearVelocity(0, &vel);
        rootBody->GetRelativeAngularVelocity(0, &avel);
        double angle = QGetAngle(quat);
        pgd::Vector axis = QGetAxis(quat);
        Util::BinaryOutput(m_OutputWarehouseFile, pos.x); Util::BinaryOutput(m_OutputWarehouseFile, pos.y); Util::BinaryOutput(m_OutputWarehouseFile, pos.z);
        Util::BinaryOutput(m_OutputWarehouseFile, angle); Util::BinaryOutput(m_OutputWarehouseFile, axis.x); Util::BinaryOutput(m_OutputWarehouseFile, axis.y); Util::BinaryOutput(m_OutputWarehouseFile, axis.z);
        Util::BinaryOutput(m_OutputWarehouseFile, vel.x); Util::BinaryOutput(m_OutputWarehouseFile, vel.y); Util::BinaryOutput(m_OutputWarehouseFile, vel.z);
        Util::BinaryOutput(m_OutputWarehouseFile, avel.x); Util::BinaryOutput(m_OutputWarehouseFile, avel.y); Util::BinaryOutput(m_OutputWarehouseFile, avel.z);
        // and now the rest of the bodies
        for (std::map<std::string, Body *>::const_iterator iter = m_BodyList.begin(); iter != m_BodyList.end(); iter++)
        {
            if (*iter->second->GetName() != m_DistanceTravelledBodyIDName)
            {
                iter->second->GetRelativePosition(rootBody, &pos);
                iter->second->GetRelativeQuaternion(rootBody, &quat);
                iter->second->GetRelativeLinearVelocity(rootBody, &vel);
                iter->second->GetRelativeAngularVelocity(rootBody, &avel);
                angle = QGetAngle(quat);
                axis = QGetAxis(quat);
                Util::BinaryOutput(m_OutputWarehouseFile, pos.x); Util::BinaryOutput(m_OutputWarehouseFile, pos.y); Util::BinaryOutput(m_OutputWarehouseFile, pos.z);
                Util::BinaryOutput(m_OutputWarehouseFile, angle); Util::BinaryOutput(m_OutputWarehouseFile, axis.x); Util::BinaryOutput(m_OutputWarehouseFile, axis.y); Util::BinaryOutput(m_OutputWarehouseFile, axis.z);
                Util::BinaryOutput(m_OutputWarehouseFile, vel.x); Util::BinaryOutput(m_OutputWarehouseFile, vel.y); Util::BinaryOutput(m_OutputWarehouseFile, vel.z);
                Util::BinaryOutput(m_OutputWarehouseFile, avel.x); Util::BinaryOutput(m_OutputWarehouseFile, avel.y); Util::BinaryOutput(m_OutputWarehouseFile, avel.z);
            }
        }
    }
}

// output the simulation state in an XML format that can be re-read

void Simulation::OutputProgramState()
{
    xmlDocPtr doc;
    xmlNodePtr rootNode;
    xmlNodePtr newNode;
    xmlAttrPtr newAttr;
    dVector3 v;
    dVector3 result, r1, r2;
    std::set<std::string> definedList;

    doc = xmlNewDoc((xmlChar *)"1.0");
    if (doc == 0) return;

    rootNode = xmlNewDocNode(doc, 0, (xmlChar *)"GAITSYMODE", 0);
    xmlDocSetRootElement(doc, rootNode);
    xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

    sprintf(m_Buffer, "%.17e", m_SimulationTime);
    newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"STATE", 0);
    newAttr = xmlNewProp(newNode, (xmlChar *)"SimulationTime", (xmlChar *)m_Buffer);
    if (m_OutputModelStateAtCycle >= 0)
    {
        sprintf(m_Buffer, "%.17e", m_OutputModelStateAtCycle);
        newAttr = xmlNewProp(newNode, (xmlChar *)"CycleFraction", (xmlChar *)m_Buffer);
    }
    xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

    const double *mungePos = m_DistanceTravelledBodyID->GetPosition();
    dVector3 munge;
    munge[0] = mungePos[0]; munge[1] = mungePos[1]; munge[2] = mungePos[2];
    if (m_YUp) munge[1] = 0;
    else munge[2] = 0;
    pgd::Quaternion mungeRotation(1, 0, 0, 0); // identity quaternion
    if (m_MungeRotationFlag)
    {
        if (m_StraightenBody)
        {
            // this version uses the orientation of the reference body to try and straighten things
            // out. I think probably deviation from the midline is better
            const double *mungeR = m_DistanceTravelledBodyID->GetRotation(); // calculate Z or Y rotation to straighten up X
            double thetaX, thetaY, thetaZ;
            Util::EulerDecompositionZYX(mungeR, thetaX, thetaY, thetaZ); // this only works if the rotations are small
            if (m_YUp) mungeRotation = pgd::MakeQFromAxis(0, 1, 0, -thetaY);
            else mungeRotation = pgd::MakeQFromAxis(0, 0, 1, -thetaZ);
        }
        else
        {
            // this version tries to straighten out the path
            double forwardDistance = mungePos[0];
            double sidewaysDistance;
            if (m_YUp) sidewaysDistance = mungePos[2];
            else sidewaysDistance = mungePos[1];
            double angle = atan2(sidewaysDistance, forwardDistance);
            if (m_YUp) mungeRotation = pgd::MakeQFromAxis(0, 1, 0, -angle);
            else mungeRotation = pgd::MakeQFromAxis(0, 0, 1, -angle);
        }
    }

    char *buf;
    xmlNodePtr cur;

    newNode = xmlNewTextChild(rootNode, 0, (xmlChar *)"IOCONTROL", 0);
    newAttr = xmlNewProp(newNode, (xmlChar *)"OldStyleInputs", (xmlChar *)"false");
    newAttr = xmlNewProp(newNode, (xmlChar *)"SanityCheckLeft", (xmlChar *)m_SanityCheckLeft.c_str());
    newAttr = xmlNewProp(newNode, (xmlChar *)"SanityCheckRight", (xmlChar *)m_SanityCheckRight.c_str());
    switch (m_SanityCheckAxis)
    {
    case XAxis:
        newAttr = xmlNewProp(newNode, (xmlChar *)"SanityCheckAxis", (xmlChar *)"X");
        break;
    case YAxis:
        newAttr = xmlNewProp(newNode, (xmlChar *)"SanityCheckAxis", (xmlChar *)"Y");
        break;
    case ZAxis:
        newAttr = xmlNewProp(newNode, (xmlChar *)"SanityCheckAxis", (xmlChar *)"Z");
        break;
    }
    xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

    std::vector<xmlNodePtr>::const_iterator iter0;
    for (iter0 = m_TagContentsList.begin(); iter0 != m_TagContentsList.end(); iter0++)
    {
        cur = *iter0;
        if (xmlStrcmp(cur->name, (const xmlChar *)"GLOBAL") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"ENVIRONMENT") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"INTERFACE") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"DRIVER") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"DATATARGET") == 0 ||
                xmlStrcmp(cur->name, (const xmlChar *)"MARKER") == 0)
        {
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting
        }

        if ((!xmlStrcmp(cur->name, (const xmlChar *)"BODY")))
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
            Body *body = m_BodyList[(const char *)buf];
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

            const double *p;

#ifdef USE_QT
            p = body->GetOffset();
            sprintf(m_Buffer, "%.17e %.17e %.17e", p[0], p[1], p[2]);
            newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Offset", (xmlChar *)m_Buffer);
#endif

            dMass mass;
            dBodyGetMass(body->GetBodyID(), &mass);
            sprintf(m_Buffer, "%.17e", mass.mass);
            newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Mass", (xmlChar *)m_Buffer);
            sprintf(m_Buffer, "%.17e %.17e %.17e %.17e %.17e %.17e", mass._I(0,0), mass._I(1,1), mass._I(2,2),
                    mass._I(0,1), mass._I(0,2), mass._I(1,2));
            newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"MOI", (xmlChar *)m_Buffer);

            sprintf(m_Buffer, "-1");
            newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Density", (xmlChar *)m_Buffer);

            if (m_MungeModelStateFlag)
            {
                p = body->GetPosition();
                pgd::Vector v1(p[0] - munge[0], p[1] - munge[1], p[2] - munge[2]);
                pgd::Vector v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "World %.17e %.17e %.17e", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                p = body->GetQuaternion();
                pgd::Quaternion qBody1(p[0], p[1], p[2], p[3]);
                pgd::Quaternion qBody2 = mungeRotation * qBody1;
                sprintf(m_Buffer, "World %.17e %.17e %.17e %.17e", qBody2.n, qBody2.v.x, qBody2.v.y, qBody2.v.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                p = body->GetLinearVelocity();
                v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "%.17e %.17e %.17e", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                p = body->GetAngularVelocity();
                v1.x = p[0]; v1.y = p[1]; v1.z = p[2];
                v2 = pgd::QVRotate(mungeRotation, v1);
                sprintf(m_Buffer, "%.17e %.17e %.17e", v2.x, v2.y, v2.z);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
            }
            else
            {
                if (m_ModelStateRelative == false)
                {
                    p = body->GetPosition();
                    sprintf(m_Buffer, "World %.17e %.17e %.17e", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    p = body->GetQuaternion();
                    sprintf(m_Buffer, "World %.17e %.17e %.17e %.17e", p[0], p[1], p[2], p[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                    p = body->GetLinearVelocity();
                    sprintf(m_Buffer, "%.17e %.17e %.17e", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                    p = body->GetAngularVelocity();
                    sprintf(m_Buffer, "%.17e %.17e %.17e", p[0], p[1], p[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
                }
                else
                {
                    // find parent from joint list
                    Body *parent = 0;
                    Joint *linkingJoint = 0;
                    std::map<std::string, Joint *>::const_iterator iter2;
                    for (iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++)
                    {
                        if (iter2->second->GetBody2() == body)
                        {
                            parent = iter2->second->GetBody1();
                            linkingJoint = iter2->second;
                            break;
                        }
                    }
                    std::string parentName("World");
                    if (parent)
                    {
                        if (definedList.find(*parent->GetName()) == definedList.end())
                        {
                            sprintf(m_Buffer, "Warning: %s, parent of %s, is not yet defined so using World\n", parent->GetName()->c_str(), body->GetName()->c_str());
                            parent = 0;
#if defined(USE_QT) && !defined(USE_WI_BB)
                            m_MainWindow->log(m_Buffer);
#endif
                            fprintf(stderr, "%s", m_Buffer);
                        }
                        else parentName = *parent->GetName();
                    }

                    HingeJoint *hj = dynamic_cast<HingeJoint *>(linkingJoint);
                    BallJoint *bj = dynamic_cast<BallJoint *>(linkingJoint);
                    UniversalJoint *uj = dynamic_cast<UniversalJoint *>(linkingJoint);
                    if (hj || bj || uj)
                    {
                        if (hj) hj->GetHingeAnchor(v);
                        else if (bj) bj->GetBallAnchor(v);
                        else if (uj) uj->GetUniversalAnchor(v);
                        dBodyGetPosRelPoint(linkingJoint->GetBody1()->GetBodyID(), v[0], v[1], v[2], r1);
                        dBodyGetPosRelPoint(linkingJoint->GetBody2()->GetBodyID(), v[0], v[1], v[2], r2);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e %.17e %.17e", parentName.c_str(), r1[0], r1[1], r1[2], r2[0], r2[1], r2[2]);
                    }
                    else
                    {
                        pgd::Vector rpos;
                        body->GetRelativePosition(parent, &rpos);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", parentName.c_str(), rpos.x, rpos.y, rpos.z);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    pgd::Quaternion rquat;
                    body->GetRelativeQuaternion(parent, &rquat);
                    pgd::Vector axis = pgd::QGetAxis(rquat);
                    double angle = pgd::QGetAngle(rquat) * 180 / M_PI;
                    sprintf(m_Buffer, "%s %.17ed %.17e %.17e %.17e", parentName.c_str(), angle, axis.x, axis.y, axis.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                    pgd::Vector rvel;
                    body->GetRelativeLinearVelocity(parent, &rvel);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", parentName.c_str(), rvel.x, rvel.y, rvel.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"LinearVelocity", (xmlChar *)m_Buffer);
                    pgd::Vector ravel;
                    body->GetRelativeAngularVelocity(parent, &ravel);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", parentName.c_str(), ravel.x, ravel.y, ravel.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"AngularVelocity", (xmlChar *)m_Buffer);
                    definedList.insert(*body->GetName());
                }
            }
        }

        if ((!xmlStrcmp(cur->name, (const xmlChar *)"JOINT")))
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
            Joint *joint = m_JointList[(const char *)buf];
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

            HingeJoint *jp = dynamic_cast<HingeJoint *>(joint);
            if (jp)
            {
                Body *body = jp->GetBody1();
                if (body)
                {
                    jp->GetHingeAnchor(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyGetPosRelPoint(jp->GetBody1()->GetBodyID(), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e","World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyVectorFromWorld(jp->GetBody1()->GetBodyID(), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAxis", (xmlChar *)m_Buffer);
                }
                else
                {
                    jp->GetHingeAnchor(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e","World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"HingeAxis", (xmlChar *)m_Buffer);
                }

                // always output the extra joint data - it's very useful
                body = jp->GetBody2();
                if (body)
                {
                    jp->GetHingeAnchor(v);
                    dBodyGetPosRelPoint(body->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    dBodyVectorFromWorld(jp->GetBody2()->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAxis", (xmlChar *)m_Buffer);
                }
                else
                {
                    jp->GetHingeAnchor(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e","World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAnchor", (xmlChar *)m_Buffer);
                    jp->GetHingeAxis(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2HingeAxis", (xmlChar *)m_Buffer);
                }

                sprintf(m_Buffer, "%.17e", jp->GetHingeAngle());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"StartAngleReference", (xmlChar *)m_Buffer);
            }

            BallJoint *jpb = dynamic_cast<BallJoint *>(joint);
            if (jpb)
            {
                Body *body = jpb->GetBody1();
                jpb->GetBallAnchor(v);
                if (m_ModelStateRelative)
                {
                    dBodyGetPosRelPoint(jpb->GetBody1()->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                }
                else
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"BallAnchor", (xmlChar *)m_Buffer);

                // always output the extra joint data - it's very useful
                body = jpb->GetBody2();
                jpb->GetBallAnchor(v);
                dBodyGetPosRelPoint(body->GetBodyID(), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Body2BallAnchor", (xmlChar *)m_Buffer);

                // and output the reference vectors
                dVector3 reference1, reference2;
                jpb->GetEulerReferenceVectors(reference1, reference2);
                sprintf(m_Buffer, "%.17e %.17e %.17e %.17e %.17e %.17e", reference1[0], reference1[1], reference1[2], reference2[0], reference2[1], reference2[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"EulerReferenceVectors", (xmlChar *)m_Buffer);
            }

            UniversalJoint *jpu = dynamic_cast<UniversalJoint *>(joint);
            if (jpu)
            {
                Body *body = jpu->GetBody1();
                if (m_ModelStateRelative)
                {
                    jpu->GetUniversalAnchor(v);
                    dBodyGetPosRelPoint(body->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAnchor", (xmlChar *)m_Buffer);
                    jpu->GetUniversalAxis1(v);
                    dBodyVectorFromWorld(body->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis1", (xmlChar *)m_Buffer);
                    jpu->GetUniversalAxis2(v);
                    dBodyVectorFromWorld(body->GetBodyID(), v[0], v[1], v[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis2", (xmlChar *)m_Buffer);
                }
                else
                {
                    jpu->GetUniversalAnchor(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e","World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAnchor", (xmlChar *)m_Buffer);
                    jpu->GetUniversalAxis1(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis1", (xmlChar *)m_Buffer);
                    jpu->GetUniversalAxis2(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis2", (xmlChar *)m_Buffer);
                }

                // always output the extra joint data - it's very useful
                body = jpu->GetBody2();
                jpu->GetUniversalAnchor(v);
                dBodyGetPosRelPoint(body->GetBodyID(), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAnchor", (xmlChar *)m_Buffer);
                jpu->GetUniversalAxis1(v);
                dBodyVectorFromWorld(body->GetBodyID(), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis1", (xmlChar *)m_Buffer);
                jpu->GetUniversalAxis2(v);
                dBodyVectorFromWorld(body->GetBodyID(), v[0], v[1], v[2], result);
                sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"UniversalAxis2", (xmlChar *)m_Buffer);
            }

            FixedJoint *jpf = dynamic_cast<FixedJoint *>(joint);
            if (jpf)
            {
            }

            SliderJoint *jps = dynamic_cast<SliderJoint *>(joint);
            if (jps)
            {
                Body *body = jps->GetBody1();
                if (body)
                {
                    jps->GetSliderAxis(v);
                    if (m_ModelStateRelative)
                    {
                        dBodyVectorFromWorld(body->GetBodyID(), v[0], v[1], v[2], result);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), result[0], result[1], result[2]);
                    }
                    else
                    {
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"SliderAxis", (xmlChar *)m_Buffer);
                }
                else
                {
                    jps->GetSliderAxis(v);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World" , v[0], v[1], v[2]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"SliderAxis", (xmlChar *)m_Buffer);
                }

                sprintf(m_Buffer, "%.17e", jps->GetSliderDistance());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"StartDistanceReference", (xmlChar *)m_Buffer);
            }

        }

        if ((!xmlStrcmp(cur->name, (const xmlChar *)"GEOM")))
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
            Geom *geom = m_GeomList[(const char *)buf];
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

            if (geom->GetGeomLocation() == Geom::body)
            {
                Body *body = (Body *)(dBodyGetData(geom->GetBody()));
                const double *p = geom->GetPosition();
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (geom->GetBody(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                dQuaternion q;
                geom->GetQuaternion(q);
                sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", body->GetName()->c_str(), q[0], q[1], q[2], q[3]);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
            }
        }

        if ((!xmlStrcmp(cur->name, (const xmlChar *)"REPORTER")))
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
            Reporter *reporter = m_ReporterList[(const char *)buf];
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

            PositionReporter *positionReporter = dynamic_cast<PositionReporter *>(reporter);
            if (positionReporter)
            {
                if (m_ModelStateRelative)
                {
                    Body *body = positionReporter->GetBody();
                    pgd::Vector p = positionReporter->GetPosition();
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(), p.x, p.y, p.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    pgd::Quaternion q = positionReporter->GetQuaternion();
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", body->GetName()->c_str(), q.n, q.v.x, q.v.y, q.v.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                }
                else
                {
                    pgd::Vector p = positionReporter->GetWorldPosition();
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", p.x, p.y, p.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Position", (xmlChar *)m_Buffer);
                    pgd::Quaternion q = positionReporter->GetWorldQuaternion();
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", "World", q.n, q.v.x, q.v.y, q.v.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Quaternion", (xmlChar *)m_Buffer);
                }
            }
        }

        if ((!xmlStrcmp(cur->name, (const xmlChar *)"MUSCLE")))
        {
            buf = DoXmlGetProp(cur, (const xmlChar *)"ID");
            Muscle *muscle = m_MuscleList[(const char *)buf];
            newNode = xmlCopyNode(cur, 1);
            xmlAddChild(rootNode, newNode);
            xmlAddChild(rootNode, xmlNewText((const xmlChar *)"\n" )); // add a line feed for formatting

            MAMuscleExtended *mam = dynamic_cast<MAMuscleExtended *>(muscle);
            if (mam)
            {
                sprintf(m_Buffer, "%.17e", mam->GetSSE());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"TendonLength", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", mam->GetLPE());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"InitialFibreLength", (xmlChar *)m_Buffer);
            }

            MAMuscleComplete *mamc = dynamic_cast<MAMuscleComplete *>(muscle);
            if (mamc)
            {
                sprintf(m_Buffer, "%.17e", mamc->GetSSE());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"TendonLength", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", mamc->GetLPE());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"InitialFibreLength", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", mamc->GetActivation());
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"StartActivation", (xmlChar *)m_Buffer);
            }

            TwoPointStrap *twoPointStrap = dynamic_cast<TwoPointStrap *>(muscle->GetStrap());
            if (twoPointStrap)
            {
                Body *body;
                double *p;
                twoPointStrap->GetOrigin(&body, &p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Origin", (xmlChar *)m_Buffer);
                twoPointStrap->GetInsertion(&body,&p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Insertion", (xmlChar *)m_Buffer);
            }
            ThreePointStrap *threePointStrap = dynamic_cast<ThreePointStrap *>(muscle->GetStrap());
            if (threePointStrap)
            {
                Body *body;
                double *p;
                threePointStrap->GetMidpoint(&body, &p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"MidPoint", (xmlChar *)m_Buffer);
            }
            CylinderWrapStrap *cylinderWrapStrap = dynamic_cast<CylinderWrapStrap *>(muscle->GetStrap());
            if (cylinderWrapStrap)
            {
                Body *body;
                dVector3 p;
                dQuaternion q;
                double radius;
                cylinderWrapStrap->GetOrigin(&body, p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Origin", (xmlChar *)m_Buffer);
                cylinderWrapStrap->GetInsertion(&body,p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Insertion", (xmlChar *)m_Buffer);
                cylinderWrapStrap->GetCylinder(&body, p, &radius, q);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderPosition", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", radius);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderRadius", (xmlChar *)m_Buffer);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", body->GetName()->c_str(),  q[0], q[1], q[2], q[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderQuaternion", (xmlChar *)m_Buffer);
                }
                else
                {
                    const double *bq = body->GetQuaternion();
                    pgd::Quaternion qLocal(q[0], q[1], q[2], q[3]);
                    pgd::Quaternion qBody(bq[0], bq[1], bq[2], bq[3]);
                    pgd::Quaternion qWorld = qBody * qLocal;
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", "World",  qWorld.n, qWorld.v.x, qWorld.v.y, qWorld.v.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderQuaternion", (xmlChar *)m_Buffer);
                }

                DoXmlRemoveProp(newNode, (xmlChar *)"CylinderAxis");

            }
            TwoCylinderWrapStrap *twoCylinderWrapStrap = dynamic_cast<TwoCylinderWrapStrap *>(muscle->GetStrap());
            if (twoCylinderWrapStrap)
            {
                Body *body;
                dVector3 p;
                dQuaternion q;
                double radius;
                twoCylinderWrapStrap->GetOrigin(&body, p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Origin", (xmlChar *)m_Buffer);
                twoCylinderWrapStrap->GetInsertion(&body,p);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Insertion", (xmlChar *)m_Buffer);
                twoCylinderWrapStrap->GetCylinder1(&body, p, &radius, q);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Cylinder1Position", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", radius);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Cylinder1Radius", (xmlChar *)m_Buffer);
                twoCylinderWrapStrap->GetCylinder2(&body, p, &radius, q);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", body->GetName()->c_str(),  p[0], p[1], p[2]);
                }
                else
                {
                    dBodyGetRelPointPos (body->GetBodyID(), p[0], p[1], p[2], result);
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                }
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Cylinder2Position", (xmlChar *)m_Buffer);
                sprintf(m_Buffer, "%.17e", radius);
                newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"Cylinder2Radius", (xmlChar *)m_Buffer);
                if (m_ModelStateRelative)
                {
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", body->GetName()->c_str(),  q[0], q[1], q[2], q[3]);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderQuaternion", (xmlChar *)m_Buffer);
                }
                else
                {
                    const double *bq = body->GetQuaternion();
                    pgd::Quaternion qLocal(q[0], q[1], q[2], q[3]);
                    pgd::Quaternion qBody(bq[0], bq[1], bq[2], bq[3]);
                    pgd::Quaternion qWorld = qBody * qLocal;
                    sprintf(m_Buffer, "%s %.17e %.17e %.17e %.17e", "World",  qWorld.n, qWorld.v.x, qWorld.v.y, qWorld.v.z);
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)"CylinderQuaternion", (xmlChar *)m_Buffer);
                }

                DoXmlRemoveProp(newNode, (xmlChar *)"CylinderAxis");

            }
            NPointStrap *nPointStrap = dynamic_cast<NPointStrap *>(muscle->GetStrap());
            if (nPointStrap)
            {
                std::vector<double *> *viaPoints = nPointStrap->GetViaPoints();
                std::vector<Body *> *viaPointBodies = nPointStrap->GetViaPointBodies();
                char viaPointName[256];
                double *p;
                for (unsigned int i = 0; i < viaPoints->size(); i++)
                {
                    p = (*viaPoints)[i];
                    sprintf(viaPointName, "ViaPoint%d", i);
                    if (m_ModelStateRelative)
                    {
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", (*viaPointBodies)[i]->GetName()->c_str(),  p[0], p[1], p[2]);
                    }
                    else
                    {
                        dBodyGetRelPointPos ((*viaPointBodies)[i]->GetBodyID(), p[0], p[1], p[2], result);
                        sprintf(m_Buffer, "%s %.17e %.17e %.17e", "World", result[0], result[1], result[2]);
                    }
                    newAttr = DoXmlReplaceProp(newNode, (xmlChar *)viaPointName, (xmlChar *)m_Buffer);
                }
            }
        }
    }


    xmlThrDefIndentTreeOutput(1);
    xmlSaveFormatFile(m_OutputModelStateFilename.c_str(), doc, 1);

    xmlFreeDoc(doc);
    m_OutputModelStateOccured = true;
}

void Simulation::SetOutputKinematicsFile(const char *filename)
{
    if (filename)
    {
        m_OutputKinematicsFilename = filename;
        m_OutputKinematicsFlag = true;
    }
    else
    {
        m_OutputKinematicsFlag = false;
    }
}

void Simulation::SetInputKinematicsFile(const char *filename)
{
    if (filename)
    {
        m_InputKinematicsFile.SetExitOnError(true);
        m_InputKinematicsFile.ReadFile(filename);
        // and skip first line
        m_InputKinematicsFile.ReadNextLine(m_Buffer, m_BufferSize, false);
        m_InputKinematicsFlag = true;
    }
    else
    {
        m_InputKinematicsFlag = false;
    }
}

void Simulation::SetOutputModelStateFile(const char *filename)
{
    if (filename)
    {
        m_OutputModelStateFilename = filename;
    }
}

void Simulation::SetOutputWarehouseFile(const char *filename)
{
    if (filename)
    {
        m_OutputWarehouseFilename = filename;
        m_OutputWarehouseFlag = true;
    }
    else
    {
        if (m_OutputWarehouseFlag) m_OutputWarehouseFile.close();
        m_OutputWarehouseFlag = false;
        m_OutputWarehouseFilename = "";
    }
}

// add a warehouse from a file
void Simulation::AddWarehouse(const char *filename)
{
}

bool Simulation::ShouldQuit()
{
    if (m_TimeLimit > 0)
        if (m_SimulationTime > m_TimeLimit) return true;
    if (m_MechanicalEnergyLimit > 0)
        if (m_MechanicalEnergy > m_MechanicalEnergyLimit) return true;
    if (m_MetabolicEnergyLimit > 0)
        if (m_MetabolicEnergy > m_MetabolicEnergyLimit) return true;
    return false;
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

void Simulation::NearCallback(void *data, dGeomID o1, dGeomID o2)
{
    int i;
    int numc;
    Simulation *s = (Simulation *)data;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    Contact *myContact;

    if (s->m_AllowConnectedCollisions == false)
    {
        if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    }

    if (s->m_AllowInternalCollisions == false)
    {
        if (((Geom *)dGeomGetData(o1))->GetGeomLocation() == ((Geom *)dGeomGetData(o2))->GetGeomLocation()) return;
    }

    dContact *contact = new dContact[s->m_MaxContacts];   // up to m_MaxContacts contacts per box-box
    double cfm = MAX(((Geom *)dGeomGetData(o1))->GetContactSoftCFM(),
                    ((Geom *)dGeomGetData(o2))->GetContactSoftCFM());
    double erp = MIN(((Geom *)dGeomGetData(o1))->GetContactSoftERP(),
                    ((Geom *)dGeomGetData(o2))->GetContactSoftERP());
    double mu = MIN(((Geom *)dGeomGetData(o1))->GetContactMu(),
                   ((Geom *)dGeomGetData(o2))->GetContactMu());
    double bounce = MAX(((Geom *)dGeomGetData(o1))->GetContactBounce(),
                       ((Geom *)dGeomGetData(o2))->GetContactBounce());
    for (i = 0; i < s->m_MaxContacts; i++)
    {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = mu;
        if (bounce >= 0)
        {
            contact[i].surface.bounce = bounce;
            contact[i].surface.mode += dContactBounce;
        }
        if (cfm >= 0)
        {
            contact[i].surface.soft_cfm = cfm;
            contact[i].surface.mode += dContactSoftCFM;
        }
        if (erp <= 1)
        {
            contact[i].surface.soft_erp = erp;
            contact[i].surface.mode += dContactSoftERP;
        }
    }
    numc = dCollide(o1, o2, s->m_MaxContacts, &contact[0].geom, sizeof(dContact));
    if (numc)
    {
        for (i = 0; i < numc; i++)
        {
            dJointID c = dJointCreateContact(s->m_WorldID, s->m_ContactGroup, contact + i);
            dJointAttach(c, b1, b2);

            if (((Geom *)dGeomGetData(o1))->GetAbort()) s->SetContactAbort(true);
            if (((Geom *)dGeomGetData(o2))->GetAbort()) s->SetContactAbort(true);

#if !defined(USE_QT) && !defined(USE_QT)
            if (gDebug == ContactDebug)
#endif
            {
                myContact = new Contact();
                dJointSetFeedback(c, myContact->GetJointFeedback());
                myContact->SetJointID(c);
                memcpy(myContact->GetContactPosition(), contact[i].geom.pos, sizeof(dVector3));
                s->m_ContactList.push_back(myContact);
                // only add the contact information once
                // and add it to the non-environment geom
                if (((Geom *)dGeomGetData(o1))->GetGeomLocation() == Geom::environment)
                    ((Geom *)dGeomGetData(o2))->AddContact(myContact);
                else
                    ((Geom *)dGeomGetData(o1))->AddContact(myContact);
#ifdef USE_QT
                myContact->SetAxisSize(s->m_Interface.GeomAxisSize);
                myContact->SetColour(s->m_Interface.GeomForceColour);
                myContact->SetForceRadius(s->m_Interface.GeomForceRadius);
                myContact->SetForceScale(s->m_Interface.GeomForceScale);
#endif
            }
        }
    }
    delete [] contact;
}

Body *Simulation::GetBody(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Body *>::const_iterator iter = m_BodyList.find(name);
    if (iter != m_BodyList.end()) return iter->second;
    return 0;
}

Joint *Simulation::GetJoint(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Joint *>::const_iterator iter = m_JointList.find(name);
    if (iter != m_JointList.end()) return iter->second;
    return 0;
}

Marker *Simulation::GetMarker(const char *name)
{
    // use find to allow null return if name not found
    std::map<std::string, Marker *>::const_iterator iter = m_MarkerList.find(name);
    if (iter != m_MarkerList.end()) return iter->second;
    return 0;
}

char *Simulation::DoXmlGetProp(xmlNode *cur, const xmlChar *name)
{
    const xmlChar *buf = 0;
#ifdef USE_CASE_SENSITIVE_XML_ATTRIBUTES
    buf = xmlGetProp(cur, name);
#else
    for (xmlAttrPtr attr = cur->properties; attr != 0; attr = attr->next)
    {
        if (strcasecmp((const char *)attr->name, (const char *)name) == 0)
        {
            buf = attr->children->content;
            // std::cerr << attr->name << " " << buf << "\n";
            break;
        }
    }
#endif

    if (gDebug == XMLDebug)
    {
        if (buf)
            *gDebugStream << name << "=\"" << buf << "\"\n";
        else
            *gDebugStream << name << " UNDEFINED\n";
    }
#if defined(USE_QT) && !defined(USE_WI_BB)
        std::stringstream ss;
        if (buf)
            ss << name << "=\"" << buf << "\"";
        else
            ss << name << " UNDEFINED";
        m_MainWindow->log(ss.str().c_str());
#endif

    if (buf)
    {
        strcpy(m_LargeBuffer, (char *)buf);
#ifdef USE_CASE_SENSITIVE_XML_ATTRIBUTES
        xmlFree(buf);
#endif
        return m_LargeBuffer;
    }
    else return 0;
}

xmlAttr *Simulation::DoXmlReplaceProp(xmlNode *cur, const xmlChar *name, const xmlChar *newValue)
{
    xmlAttr *ptr;
#ifdef USE_CASE_SENSITIVE_XML_ATTRIBUTES
    ptr = xmlHasProp(cur, name);
#else
    for (ptr = cur->properties; ptr != 0; ptr = ptr->next)
    {
        if (strcasecmp((const char *)ptr->name, (const char *)name) == 0)
        {
            break;
        }
    }
#endif
    if (ptr) xmlRemoveProp(ptr);

    ptr = xmlNewProp(cur, name, newValue);
    return ptr;
}

void Simulation::DoXmlRemoveProp(xmlNode *cur, const xmlChar *name)
{
    xmlAttr *ptr;
#ifdef USE_CASE_SENSITIVE_XML_ATTRIBUTES
    ptr = xmlHasProp(cur, name);
#else
    for (ptr = cur->properties; ptr != 0; ptr = ptr->next)
    {
        if (strcasecmp((const char *)ptr->name, (const char *)name) == 0)
        {
            break;
        }
    }
#endif
    if (ptr) xmlRemoveProp(ptr);
}

xmlAttr *Simulation::DoXmlHasProp(xmlNode *cur, const xmlChar *name)
{
    xmlAttr *ptr;
#ifdef USE_CASE_SENSITIVE_XML_ATTRIBUTES
    ptr = xmlHasProp(cur, name);
#else
    for (ptr = cur->properties; ptr != 0; ptr = ptr->next)
    {
        if (strcasecmp((const char *)ptr->name, (const char *)name) == 0)
        {
            break;
        }
    }
#endif
    return ptr;
}

// this version of the dump routine simply calls the dump functions of the embedded objects
void Simulation::Dump()
{
    std::map<std::string, Body *>::const_iterator BodyIter;
    for (BodyIter = m_BodyList.begin(); BodyIter != m_BodyList.end(); BodyIter++) BodyIter->second->Dump();

    std::map<std::string, Joint *>::const_iterator JointIter;
    for (JointIter = m_JointList.begin(); JointIter != m_JointList.end(); JointIter++) JointIter->second->Dump();

    std::map<std::string, Geom *>::const_iterator GeomIter;
    for (GeomIter = m_GeomList.begin(); GeomIter != m_GeomList.end(); GeomIter++) GeomIter->second->Dump();

    std::map<std::string, Muscle *>::const_iterator MuscleIter;
    for (MuscleIter = m_MuscleList.begin(); MuscleIter != m_MuscleList.end(); MuscleIter++) { MuscleIter->second->Dump(); MuscleIter->second->GetStrap()->Dump(); }

    std::map<std::string, Driver *>::const_iterator DriverIter;
    for (DriverIter = m_DriverList.begin(); DriverIter != m_DriverList.end(); DriverIter++) DriverIter->second->Dump();

    std::map<std::string, DataTarget *>::const_iterator DataTargetIter;
    for (DataTargetIter = m_DataTargetList.begin(); DataTargetIter != m_DataTargetList.end(); DataTargetIter++) DataTargetIter->second->Dump();

    std::map<std::string, Reporter *>::const_iterator ReporterIter;
    for (ReporterIter = m_ReporterList.begin(); ReporterIter != m_ReporterList.end(); ReporterIter++) ReporterIter->second->Dump();

    std::map<std::string, Warehouse *>::const_iterator WarehouseIter;
    for (WarehouseIter = m_WarehouseList.begin(); WarehouseIter != m_WarehouseList.end(); WarehouseIter++) WarehouseIter->second->Dump();

}

//----------------------------------------------------------------------------
#ifdef USE_QT
void
        Simulation::Draw(GLWidget *glWidget)
{
    // draw the most opaque objects first and the least opaque obejects last
    // this isn't the best way - I should distacne sort and/or depth peel but these are quite slow

    std::map<std::string, Body *>::const_iterator iter1;
    std::map<std::string, Joint *>::const_iterator iter2;
    std::map<std::string, Muscle *>::const_iterator iter4;
    std::map<std::string, Geom *>::const_iterator iter3;
    std::map<std::string, Reporter *>::const_iterator ReporterIter;
    std::map<std::string, Marker *>::const_iterator MarkerIter;
    std::map<std::string, DataTarget *>::const_iterator DataTargetIter;

    // draw everything into gDelayedDrawList (if possible)
    gDelayedDraw = true;
//    m_Environment->Draw();
    for (iter1 = m_BodyList.begin(); iter1 != m_BodyList.end(); iter1++) iter1->second->Draw();
    for (iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++) if (iter2->second->GetPhysRep()) iter2->second->Draw(); // draw the facetted object joints
    for (iter4 = m_MuscleList.begin(); iter4 != m_MuscleList.end(); iter4++) iter4->second->Draw();
    for (iter3 = m_GeomList.begin(); iter3 != m_GeomList.end(); iter3++) iter3->second->Draw();
    for (unsigned int c = 0; c < m_ContactList.size(); c++) m_ContactList[c]->Draw();
    for (ReporterIter = m_ReporterList.begin(); ReporterIter != m_ReporterList.end(); ReporterIter++) ReporterIter->second->Draw();
    for (MarkerIter = m_MarkerList.begin(); MarkerIter != m_MarkerList.end(); MarkerIter++) MarkerIter->second->Draw();
    for (DataTargetIter = m_DataTargetList.begin(); DataTargetIter != m_DataTargetList.end(); DataTargetIter++) DataTargetIter->second->Draw();

    // sort on alpha
    gDelayedDrawList.sort(SortOnAlpha);

    // and draw in alpha order (highest alpha first)
    gDelayedDraw = false;
    std::list<FacetedObject *>::const_iterator FacetedObjectIter;
    for (FacetedObjectIter = gDelayedDrawList.begin(); FacetedObjectIter != gDelayedDrawList.end(); FacetedObjectIter++)
    {
        // std::cerr << (*FacetedObjectIter)->GetColour()->alpha << "\n";
        (*FacetedObjectIter)->setGlWidget(glWidget);
        (*FacetedObjectIter)->Draw();
    }
    gDelayedDrawList.clear();

    for (iter2 = m_JointList.begin(); iter2 != m_JointList.end(); iter2++) if (iter2->second->GetPhysRep() == 0) iter2->second->Draw();  // draw the non-facetted object joints
    m_Environment->Draw(); // and the environment currently doesn't use faceted objects either
}

// this sorts so that the largest value of alpha is first
bool SortOnAlpha(FacetedObject* d1, FacetedObject* d2)
{
  return d1->GetColour()->alpha > d2->GetColour()->alpha;
}

#endif


