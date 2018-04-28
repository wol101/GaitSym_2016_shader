/*
 *  Simulation.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Simulation.h - this simulation object is used to encapsulate
// a dynamechs simulation

#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include "Environment.h"
#include "DataFile.h"

#include <ode/ode.h>

#include <map>
#include <string>
#include <fstream>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

class Body;
class Joint;
class Geom;
class Muscle;
class Driver;
class DataTarget;
class Contact;
class Marker;
class Reporter;
class Controller;
class FixedJoint;
class Warehouse;

#ifdef USE_QT
class GLWidget;
class MainWindow;
#endif


#ifdef USE_QT

struct Interface
{
    float EnvironmentAxisSize[3];
    Colour EnvironmentColour;
    float BodyAxisSize[3];
    Colour BodyColour;
    float JointAxisSize[3];
    Colour JointColour;
    Colour StrapColour;
    float StrapRadius;
    Colour StrapForceColour;
    float StrapForceRadius;
    float StrapForceScale;
    Colour StrapCylinderColour;
    float StrapCylinderLength;
    float GeomAxisSize[3];
    Colour GeomColour;
    Colour GeomForceColour;
    float GeomForceRadius;
    float GeomForceScale;
    Colour ReporterColour;
    float ReporterRadius;
    Colour DataTargetColour;
    float DataTargetRadius;
    Colour MarkerColour;
    float MarkerRadius;

    std::string TrackBodyID;
};
#endif

enum WorldStepType
{
    WorldStep,
    QuickStep
};

enum AxisType
{
    XAxis,
    YAxis,
    ZAxis
};

class Simulation: public NamedObject
{
public:

    Simulation(void);
    ~Simulation(void);

    enum FitnessType
    {
        DistanceTravelled = 0,
        KinematicMatch = 1,
        KinematicMatchMiniMax = 2,
        ClosestWarehouse = 3
    };

    // friend void NearCallback(void *data, dGeomID o1, dGeomID o2);
    static void NearCallback(void *data, dGeomID o1, dGeomID o2);

    int LoadModel(char *buffer);  // load parameters from the XML configuration file
    void UpdateSimulation(void);     // called at each iteration through simulation

    // get hold of various variables

    double GetTime(void) { return m_SimulationTime; }
    double GetTimeIncrement(void) { return m_StepSize; }
    long long GetStepCount(void) { return m_StepCount; }
    double GetMechanicalEnergy(void) { return m_MechanicalEnergy; }
    double GetMetabolicEnergy(void) { return m_MetabolicEnergy; }
    double GetTimeLimit(void) { return m_TimeLimit; }
    double GetMetabolicEnergyLimit(void) { return m_MetabolicEnergyLimit; }
    double GetMechanicalEnergyLimit(void) { return m_MechanicalEnergyLimit; }
    Body *GetBody(const char *name);
    Joint *GetJoint(const char *name);
    Marker *GetMarker(const char *name);
    bool GetYUp() { return m_YUp; }
    Environment *GetEnvironment() { return m_Environment; }

    void SetTimeLimit(double timeLimit) { m_TimeLimit = timeLimit; }
    void SetMetabolicEnergyLimit(double energyLimit) { m_MetabolicEnergyLimit = energyLimit; }
    void SetMechanicalEnergyLimit(double energyLimit) { m_MechanicalEnergyLimit = energyLimit; }
    void SetOutputModelStateAtTime(double outputModelStateAtTime) { m_OutputModelStateAtTime = outputModelStateAtTime; }
    void SetOutputModelStateAtCycle(double outputModelStateAtCycle) { m_OutputModelStateAtCycle = outputModelStateAtCycle; }
    void SetOutputModelStateAtWarehouseDistance(double outputModelStateAtWarehouseDistance) { m_OutputModelStateAtWarehouseDistance = outputModelStateAtWarehouseDistance; }
    void SetOutputKinematicsFile(const char *filename);
    void SetInputKinematicsFile(const char *filename);
    void SetOutputModelStateFile(const char *filename);
    void SetOutputWarehouseFile(const char *filename);
    void SetMungeModelStateFlag(bool f) { m_MungeModelStateFlag = f; }
    void SetMungeRotationFlag(bool f) { m_MungeRotationFlag = f; }
    void SetModelStateRelative(bool f) { m_ModelStateRelative = f; }
    void SetYUp(bool yUp) { m_YUp = yUp; }
    void SetWarehouseFailDistanceAbort(double warehouseFailDistanceAbort) { m_WarehouseFailDistanceAbort = warehouseFailDistanceAbort; if (m_FitnessType == ClosestWarehouse) m_FitnessType = DistanceTravelled; }

    void AddWarehouse(const char *filename);

    // get hold of the internal lists (HANDLE WITH CARE)
    std::map<std::string, Body *> *GetBodyList() { return &m_BodyList; }
    std::map<std::string, Joint *> *GetJointList() { return &m_JointList; }
    std::map<std::string, Geom *> *GetGeomList() { return &m_GeomList; }
    std::map<std::string, Muscle *> *GetMuscleList() { return &m_MuscleList; }
    std::map<std::string, Driver *> *GetDriverList() { return &m_DriverList; }
    std::map<std::string, DataTarget *> *GetDataTargetList() { return &m_DataTargetList; }
    std::map<std::string, Marker *> *GetMarkerList() { return &m_MarkerList; }
    std::map<std::string, Reporter *> *GetReporterList() { return &m_ReporterList; }
    std::map<std::string, Controller *> *GetControllerList() { return &m_ControllerList; }
    std::map<std::string, FixedJoint *> *GetJointStressList() { return &m_JointStressList; }
    std::map<std::string, Warehouse *> *GetWarehouseList() { return &m_WarehouseList; }
    std::vector<Contact *> * GetContactList() { return &m_ContactList; }

    // fitness related values
    bool TestForCatastrophy();
    double CalculateInstantaneousFitness();
    bool ShouldQuit();
    void SetContactAbort(bool contactAbort) { m_ContactAbort = contactAbort; }
    void SetDataTargetAbort(bool dataTargetAbort) { m_DataTargetAbort = dataTargetAbort; }

    // these should probably only be internal
    void InputKinematics();
    void OutputKinematics();
    void OutputProgramState();
    void OutputWarehouse();

    virtual void Dump();

    // draw the simulation
#ifdef USE_QT
    void Draw(GLWidget *glWidget);
    Interface *GetInterface() { return &m_Interface; }
    std::vector<Geom *> *GetPickGeomList() { return &m_PickGeomList; }
    void SetMainWindow(MainWindow *mainWindow) { m_MainWindow = mainWindow; }
#endif

protected:

    void ParseGlobal(xmlNodePtr cur);
    void ParseEnvironment(xmlNodePtr cur);
    void ParseBody(xmlNodePtr cur);
    void ParseGeom(xmlNodePtr cur);
    void ParseJoint(xmlNodePtr cur);
    void ParseMuscle(xmlNodePtr cur);
    void ParseDriver(xmlNodePtr cur);
    void ParseDataTarget(xmlNodePtr cur);
    void ParseIOControl(xmlNodePtr cur);
    void ParseMarker(xmlNodePtr cur);
    void ParseReporter(xmlNodePtr cur);
    void ParseController(xmlNodePtr cur);
    void ParseWarehouse(xmlNodePtr cur);

    std::vector<xmlNodePtr> m_TagContentsList;

    char *DoXmlGetProp(xmlNode *cur, const xmlChar *name);
    xmlAttr *DoXmlReplaceProp(xmlNode *cur, const xmlChar *name, const xmlChar *newValue);
    void DoXmlRemoveProp(xmlNode *cur, const xmlChar *name);
    xmlAttr *DoXmlHasProp(xmlNode *cur, const xmlChar *name);

    std::map<std::string, Body *>m_BodyList;
    std::map<std::string, Joint *>m_JointList;
    std::map<std::string, Geom *>m_GeomList;
    std::map<std::string, Muscle *>m_MuscleList;
    std::map<std::string, Driver *>m_DriverList;
    std::map<std::string, DataTarget *>m_DataTargetList;
    std::map<std::string, Marker *>m_MarkerList;
    std::map<std::string, Reporter *>m_ReporterList;
    std::map<std::string, Controller *>m_ControllerList;
    std::map<std::string, FixedJoint *>m_JointStressList;
    std::map<std::string, Warehouse *>m_WarehouseList;
    bool m_DataTargetAbort;

    // Simulation variables
    dWorldID m_WorldID;
    dSpaceID m_SpaceID;
    dJointGroupID m_ContactGroup;
    Environment *m_Environment;
    int m_MaxContacts;
    bool m_AllowInternalCollisions;
    bool m_AllowConnectedCollisions;
    WorldStepType m_StepType;

    // keep track of simulation time

    double m_SimulationTime; // current time
    double m_StepSize; // step size
    long long m_StepCount; // number of steps taken
    double m_CycleTime;

    // and calculated energy

    double m_MechanicalEnergy;
    double m_MetabolicEnergy;
    double m_BMR;

    // FitnessType
    FitnessType m_FitnessType;
    double m_TimeLimit;
    double m_MechanicalEnergyLimit;
    double m_MetabolicEnergyLimit;
    double m_KinematicMatchMiniMaxFitness;
    double m_ClosestWarehouseFitness;

    // some control values
    bool m_InputKinematicsFlag;
    DataFile m_InputKinematicsFile;
    bool m_OutputKinematicsFlag;
    bool m_OutputWarehouseFlag;
    std::string m_OutputKinematicsFilename;
    std::ofstream m_OutputKinematicsFile;
    std::string m_OutputModelStateFilename;
    std::string m_OutputWarehouseFilename;
    std::ofstream m_OutputWarehouseFile;
    bool m_OutputModelStateOccured;
    bool m_AbortAfterModelStateOutput;
    bool m_OutputWarehouseAsText;
    double m_OutputModelStateAtTime;
    double m_OutputModelStateAtCycle;
    bool m_MungeModelStateFlag;
    bool m_MungeRotationFlag;
    int m_SimulationError;
    bool m_ModelStateRelative;
    bool m_YUp;
    bool m_StraightenBody;
    bool m_AbortOnODEMessage;
    double m_WarehouseDistance;
    double m_WarehouseUnitIncreaseDistanceThreshold;
    double m_WarehouseDecreaseThresholdFactor;
    bool m_OutputKinematicsFirstTimeFlag;
    double m_OutputWarehouseLastTime;
    double m_WarehouseFailDistanceAbort;
    double m_OutputModelStateAtWarehouseDistance;
    std::string m_CurrentWarehouse;
    bool m_WarehouseUsePCA;

    // internal memory allocations
    // it will crash if these are exceeded but they should be plenty big enough
    int m_BufferSize;
    char *m_Buffer;
    char *m_LargeBuffer;
    char **m_BufferPtrs;
    double *m_DoubleList;

    // for fitness calculations
    double m_KinematicMatchFitness;
    std::string m_DistanceTravelledBodyIDName;
    Body *m_DistanceTravelledBodyID;

   // contact joint list
    std::vector<Contact *> m_ContactList;
    bool m_ContactAbort;

    // values for energy partition
    double m_PositiveMechanicalWork;
    double m_NegativeMechanicalWork;
    double m_PositiveContractileWork;
    double m_NegativeContractileWork;
    double m_PositiveSerialElasticWork;
    double m_NegativeSerialElasticWork;
    double m_PositiveParallelElasticWork;
    double m_NegativeParallelElasticWork;

    std::string m_SanityCheckLeft;
    std::string m_SanityCheckRight;
    AxisType m_SanityCheckAxis;

#ifdef USE_QT
    void ParseInterface(xmlNodePtr cur);
    Interface m_Interface;
    std::vector<Geom *> m_PickGeomList;
    int m_nextTextureID;
    bool m_UseProgramInterface;
#endif
#ifdef USE_QT
    MainWindow *m_MainWindow;
#endif

};



#endif //__SIMULATION_H__
