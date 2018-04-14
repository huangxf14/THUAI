#include"api_player.h"
#include<vector>
#include"communication.h"
using namespace std;

bool ts19_updageAge;
vector<command1> ts19_c1;
vector<command2>ts19_c2;

void updateAge()
{
    ts19_updageAge = true;
}
void construct(BuildingType building_type, Position pos, Position soldier_pos)
{
    ts19_c2.push_back(command2(int(building_type), 1, pos.x, pos.y, soldier_pos.x, soldier_pos.y));
}
void upgrade(int unit_id)
{
    ts19_c1.push_back(command1(unit_id, 2));
}
void sell(int unit_id)
{
    ts19_c1.push_back(command1(unit_id, 3));
}
void toggleMaintain(int unit_id)
{
    ts19_c1.push_back(command1(unit_id, 4));
}
