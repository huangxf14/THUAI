#pragma once

#include <limits.h>

#define _NULL -1

//global const for player
const int UPDATE_COST        = 2000;
const int UPDATE_COST_PLUS   = 1500;
const int MAX_BD_NUM         = 40;
const int MAX_BD_NUM_PLUS    = 20;
const int MAP_SIZE           = 200;
const int BASE_SIZE          = 7;
const int BD_RANGE           = 8;
const int BD_RANGE_FROM_BASE = 4;
const float AGE_INCREASE     = 0.5;

//definition of enumeration
enum UnitType {
    Base,

    PRODUCTION_BUILDING,
    DEFENSIVE_BUILDING,
    RESOURCE_BUILDING,

    DATA,
    SUBSTANCE,
    ALL
};

enum Age {
    BIT,
    CIRCUIT,
    PROCESSOR,
    ALGORITHM,
    NETWORK,
    AI
};

enum BuildingType{
    //base
    __Base,

    //buidings for producing
    Shannon,
    Thevenin,
    Norton,
    Von_Neumann,
    Berners_Lee,
    Kuen_Kao,
    Turing,
    Tony_Stark,

    //buildings for defending
    Bool,
    Ohm,
    Mole,
    Monte_Carlo,
    Larry_Roberts,
    Robert_Kahn,
    Musk,
    Hawkin,

    //building for resource
    Programmer,
    Building_Type
};

enum BuildingAttribute {
    BUILDING_TYPE,
    ORIGINAL_HEAL,
    ORIGINAL_ATTACK,  //For defensive buildings, it stands for the level of attack.
                      //For resources buildings, it stands for Collective_Value
                      //For production buildings, it means nothing set as _NULL

    ORIGINAL_RANGE,    //FOr production buildings, it stands for the range of production,
                      //For defensive buildings, it stands for the range of attacking
                      //For resource buildings, it means nothing

    TARGET,           //For production buildings, it stands for the type of units it produce,
                      //For defensive buildings, it stands for the type of units it attack.
    AGE,
    ORIGINAL_RESOURCE,
    ORIGINAL_BUILDING_POINT,
    AOE,              //Only works for defensive buildings
    CD,
    Attr_Num          //For the definition of Array
};

enum SoldierName {
    BIT_STREAM,
    VOLTAGE_SOURCE,
    CURRENT_SOURCE,
    ENIAC,
    PACKET,
    OPTICAL_FIBER,
    TURING_MACHINE,
    ULTRON,
    Soldier_Type
};

enum SoldierAttr {
    SOLDIER_TYPE,
    ACTION_MODE,
    SOLDIER_ORIGINAL_HEAL,
    SOLDIER_ORIGINAL_ATTACK,
    ATTACK_RANGE,
    SPEED,
    Soldier_Attr_Num  //For the definition of Array
};

enum ActionMode {
    BUILDING_ATTACK, //attack building
    BASE_ATTACK,     //go forward
    MOVING_ATTACK    //defence
};

//   type                 hp   attack   range  target          age       resurce bd_point  aoe   cd
const int OriginalBuildingAttribute[Building_Type][Attr_Num] = {
    {Base,              10000,  _NULL,  0,    _NULL,           BIT,       0,      _NULL, _NULL,  _NULL},
    {PRODUCTION_BUILDING, 120,  _NULL,  8,   BIT_STREAM,      BIT,       150,    15,    _NULL,  2},
    {PRODUCTION_BUILDING, 160,  _NULL,  5,    VOLTAGE_SOURCE,  CIRCUIT,   160,    16,    _NULL,  6},
    {PRODUCTION_BUILDING, 150,  _NULL,  5,    CURRENT_SOURCE,  CIRCUIT,   160,    16,    _NULL,  4},
    {PRODUCTION_BUILDING, 200,  _NULL,  12,   ENIAC,           PROCESSOR, 200,    20,    _NULL,  15},
    {PRODUCTION_BUILDING, 150,  _NULL,  10,   PACKET,          NETWORK,   250,    25,    _NULL,  3},
    {PRODUCTION_BUILDING, 160,  _NULL,  12,   OPTICAL_FIBER,   NETWORK,   400,    40,    _NULL,  6},
    {PRODUCTION_BUILDING, 250,  _NULL,  10,   TURING_MACHINE,  AI,        600,    60,    _NULL,  20},
    {PRODUCTION_BUILDING, 220, _NULL,   8,   ULTRON,          AI,        600,    60,    _NULL,  20},

    {DEFENSIVE_BUILDING,  220,  16,     32,   DATA,            BIT,       150,    15,    0,      1},
    {DEFENSIVE_BUILDING,  320,  20,     30,   SUBSTANCE,       CIRCUIT,   200,    20,    6,      3},
    {DEFENSIVE_BUILDING,  250,  4,      36,   DATA,            PROCESSOR, 225,    22,    0,      1},
    {DEFENSIVE_BUILDING,  350,  25,     50,   SUBSTANCE,       ALGORITHM, 200,    20,    0,      2},
    {DEFENSIVE_BUILDING,  220,  6,      40,   ALL,             NETWORK,   250,    25,    8,      1},
    {DEFENSIVE_BUILDING,  520,  5,      35,   DATA,            NETWORK,   450,    45,    0,      1},
    {DEFENSIVE_BUILDING,  1000, 0,      24,   ALL,             AI,        500,    50,    0,      1},
    {DEFENSIVE_BUILDING,  480, INT_MAX, 20,   ALL,             AI,        500,    50,    2,      2},

    {RESOURCE_BUILDING,   100,  10,     _NULL,_NULL,          _NULL,      100,    10, _NULL,  _NULL}
};

const int OriginalSoldierAttribute[Soldier_Type][Soldier_Attr_Num] = {
    {DATA,      BUILDING_ATTACK,        16,     10,     16,     12},
    {SUBSTANCE, BUILDING_ATTACK,        60,     18,     24,     8},
    {SUBSTANCE, BASE_ATTACK,            40,     160,    3,      15},
    {SUBSTANCE, MOVING_ATTACK,          550,    12,     10,     4},
    {DATA,      BASE_ATTACK,            50,     300,    3,      16},
    {SUBSTANCE, BUILDING_ATTACK,        40,     25,     40,     12},
    {DATA,      MOVING_ATTACK,          850,    8,      12,     3},
    {SUBSTANCE, BUILDING_ATTACK,        200,    500,    20,     8}
};


//struct

//The global variables of both sides
struct Resource {
    int building_point_1;
    int building_point_2;

    int resource_1;
    int resource_2;
    Resource(int building_point_1=0, int building_point_2=0, int resource_1=0, int resource_2=0):
        building_point_1(building_point_1), building_point_2(building_point_2), resource_1(resource_1), resource_2(resource_2) {}
};

struct Position {
    int x;
    int y;
    Position(int x = -1, int y = -1): x(x), y(y) {}
};

struct Soldier {
    SoldierName soldier_name;
    int heal;
    Position pos;
    int flag;
    int unit_id;
    int level;

	Soldier() {};
	Soldier(SoldierName soldier_name, int heal, Position pos, int flag, int unit_id,int level) :soldier_name(soldier_name),heal(heal),pos(pos),flag(flag),unit_id(unit_id),level(level){};
};

struct Building {
    BuildingType building_type;
    int heal;
    Position pos;
    int flag;
    int unit_id;
    int level;

	Building() {};
	Building(BuildingType building_type, int heal, Position pos, int flag, int unit_id,int level):building_type(building_type),heal(heal),pos(pos),flag(flag),unit_id(unit_id),level(level) {};
};

//API for players
void updateAge();
void construct(BuildingType building_type, Position pos, Position soldier_pos=Position(0,0)); //The solier position is not set now
void upgrade(int unit_id);
void sell(int unit_id);
void toggleMaintain(int unit_id);
