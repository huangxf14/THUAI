#pragma once
#include<WinSock2.h>
#pragma comment(lib,"ws2_32.lib")
#include"api_player.h"
#define _mapsize 200
#include<vector>

struct _resource
{
    int building_point;
    int resource;
    _resource(int building_point = -1, int resource = -1) :building_point(building_point), resource(resource) {}
};



struct State
{
    int turn;
    int winner;
    _resource resource[2];
    Age age[2];
    std::vector<Building> building[2];
    std::vector<Soldier> soldier[2];
};

struct command1
{
    int id;
    int commandid;
    command1(int id, int commandid) :id(id), commandid(commandid) {}
};
struct command2
{
    int building_type;
    int commandid;
    int bx;
    int by;
    int sx;
    int sy;
    command2(int building_type, int commandid, int bx, int by, int sx, int sy) :building_type(building_type), commandid(commandid), bx(bx), by(by), sx(sx), sy(sy) {}
};

class MyClient
{
private:
    WSADATA wsaData;
    SOCKET sockClient;
    SOCKADDR_IN addrServer;
    bool fflag;
    char* smap;
    void change_map(char* map0);
    char* change_command(bool _update, std::vector<command1> &v1, std::vector<command2>&v2);

public:
    int flag;
    int **map;

public:
    MyClient();
    void start_connection();
    void send_command(bool _update, std::vector<command1> &v1, std::vector<command2>&v2);
    State* recv_state();
};
enum CommandID
{
    UpdateAge,
    Construct,
    Upgrade,
    Sell,
    Maintain,
    Noupdateage

};
