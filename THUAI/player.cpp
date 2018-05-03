/*
TODO List
- Finish defense inbase
- Add defense for outside strategy

*/

#define DEBUG
#ifdef DEBUG
#define tic(); clock_t start=clock();
#define toc(); cout<<double(clock()-start)/CLOCKS_PER_SEC<<endl;
#define LOG(s); cout<<s;
#define LOGN(s); cout<<s<<endl;
#else
#define tic();
#define toc();
#define LOG(s); 
#define LOGN(s); 
#endif // DEBUG

#include"communication.h"
#include<vector>
#include<list>
#include<string.h>
#include <math.h>  
#include<stdlib.h>

#ifdef DEBUG
#include <ctime>
#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
using std::cout;
using std::endl;
std::ofstream file("log.txt", 'w');
#endif // DEBUG


#define ROADMAX 10
#define BUILDINGMAX 200
#define MAX_SOLDIER_DIS 10000
#define MAX_FIRE 420
#define PRUDUCE_SCORE 10
#define Norton_SCORE 15
#define Berners_Lee_SCORE 18
#define Kuen_Kao_SCORE 20
#define AI_SCORE 25
#define FIX_ROAD_SCORE 100
#define NEED_ROAD_SCORE 10
#define FULL_ROAD_SCORE 1

extern State* state;
extern std::vector<State* > all_state;
extern int** ts19_map;
extern bool ts19_flag;
int command_num = 0;
int tx4[4] = { -1,0,1,0 }, ty4[4] = { 0,-1,0,1 };
int colormap[MAP_SIZE][MAP_SIZE] = { 0 };//Seg area by road
int mindismap[MAP_SIZE][MAP_SIZE] = { 0 };//the min dis for every point to the nearest road
int buildmap[MAP_SIZE][MAP_SIZE] = { 0 };//0 - useless;1 - road;2 - building&base;3 - useful;
int enemy_buildmap[MAP_SIZE][MAP_SIZE] = { 0 };//0 - useless;1 - road;2 - building&base;3 - useful;
bool buildprogrammer_queueflag[MAP_SIZE][MAP_SIZE];
int savearea_everypoint_dis[ROADMAX+1][MAP_SIZE][MAP_SIZE];
bool savearea_everypoint_flag[ROADMAX];
Position queue[MAP_SIZE*MAP_SIZE];
Position* root = new Position[MAP_SIZE*MAP_SIZE];
Position nextpos[MAP_SIZE][MAP_SIZE];
Position roadroot[ROADMAX];
int discount[41] = { 0 };
int ROADCNT = 0;//count from 1,ROADCNT = the max number of road
int ENEMY_EDGE = MAP_SIZE - BASE_SIZE,MY_EDGE = BASE_SIZE - 1;
int my_base_hp, enemy_base_hp;
int my_bd_num = 0;
int max_bd_point = 0;
Age coming_age = BIT;

int area_dis = 25;// 40;//the min distance for save area
int *minareadis;//min distance from enemybase of every area
int *areacount;//the cnt of allowed point of every area
int *areasortid;//sort area by the dis from enemybase(short to long)
std::vector<Position>* saveArea;
std::vector<int>* saveArea_enemydis;
float science_factor = 1;
int myresource = 0, mybdpoint = 0;
int resource_num = MAX_BD_NUM, my_resource_num = 0, enemy_resource_num = 0, can_upgrade_reource_num = 0;//the number of resource building
bool *resource_area;//whether this area have resource building
//int resource_limit[6] = { 40,30,40,60,40,10 };
int can_upgrade_produce_num = 0;//the number of produce
int can_upgrade_defense_num = 0;//the number of defense
int nearest_enemybuilding_dis = 10000;
int enemy_procude_num = 0, my_produce_num = 0, my_defense_num = 0;
int my_building_num[Building_Type] = { 0 }, enemy_building_num[Building_Type] = { 0 };
int sell_num = 0;
bool sell_list[BUILDINGMAX];
int building_base_dis[Building_Type] = { 1,1,1,1,1,1,1,1,1,15,15,25,30,30,30,32,30,1 };

inline int trans(int x)//Use this everywhere when x means pos
{
	return ts19_flag ? (MAP_SIZE - 1 - x) : x;
}

struct SoldierPoint
{
	Position pos;
	int cd;
	SoldierName soldier_type;
	float fire;
	bool musk;
	SoldierPoint(Position pos = Position(0, 0), int cd = 0, SoldierName soldiertype = Soldier_Type, float fire = 0,bool musk=false) :
		pos(Position(trans(pos.x), trans(pos.y))), cd(cd),soldier_type(soldiertype),fire(fire),musk(musk) {}
};

std::list<SoldierPoint> SoldierPointList;
int soldier_distrubutemap[MAP_SIZE][MAP_SIZE][Soldier_Type];
int maxsoliderid = -1;
int SoldierCD[Soldier_Type];
int soldiercount[ROADMAX+1][Soldier_Type] = {0};
int nearest_soldierdis[ROADMAX + 1][Soldier_Type];
int soldier_save_dis = MAP_SIZE >> 1;

int firemap_inbase[ROADMAX + 1][Building_Type], needfiremap_inbase[ROADMAX + 1][Building_Type];//the fire from each kind of building in each road
int soldier_freq[ROADMAX + 1][Soldier_Type];//soldier num in one road in one turn
int building_firemap[BUILDINGMAX][ROADMAX + 1];
bool sell_for_defense = false, sell_for_produce = false;

//var for defense_mode
bool setup_control = false, defense_control = false;
Position last_pos = Position(-10, -10);
int defense_road[MAP_SIZE][MAP_SIZE];
Position my_soldier_pos[MAP_SIZE][MAP_SIZE];
int soldiernum_in_road[ROADMAX + 1][Building_Type];
Position kernel_defense[ROADMAX + 1], kernel_target[ROADMAX + 1];
int defense_gap = 0;
int army_road = 1;
Age last_age = BIT;
bool enemy_inbase = false;
float musk_heal[ROADMAX + 1];


//remember Musk when building
const BuildingType defense_choise[Soldier_Type][AI+1] = {
	{ Bool,    Bool,  Bool, Bool,        Larry_Roberts, Larry_Roberts },//BIT_STREAM
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Larry_Roberts },// VOLTAGE_SOURCE
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Larry_Roberts },//CURRENT_SOURCE
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin },//ENIAC
	{ Bool,    Bool, Bool, Bool,        Larry_Roberts, Larry_Roberts },//PACKET
	{ __Base,  Ohm,  Ohm,  Monte_Carlo, Larry_Roberts,   Larry_Roberts },//OPTICAL_FIBER,
	{ Bool,    Bool, Mole, Mole,        Mole,   Hawkin },//TURING_MACHINE
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin }//ULTRON
};
const BuildingType defense_choice_outside[Soldier_Type][AI + 1] = {
	{ Bool,    Bool,  Bool, Bool,        Larry_Roberts, Hawkin },//BIT_STREAM
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Hawkin },// VOLTAGE_SOURCE
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Hawkin },//CURRENT_SOURCE
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin },//ENIAC
	{ Bool,    Bool, Bool, Bool,        Larry_Roberts, Hawkin },//PACKET
	{ __Base,  Ohm,  Ohm,  Ohm, Larry_Roberts,   Hawkin },//OPTICAL_FIBER,
	{ Bool,    Bool, Mole, Mole,        Mole,   Hawkin },//TURING_MACHINE
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin }//ULTRON
};


int mymin(int a, int b)
{
	if (a < b) return a;
	else return b;
}

int mymax(int a, int b)
{
	if (a > b) return a;
	else return b;
}

int enemy_base_dist(int x, int y)//use my dis
{
	if ((x >= ENEMY_EDGE) && (y >= ENEMY_EDGE))
		return 0;
	if (x >= ENEMY_EDGE)
		return ENEMY_EDGE - y;
	if (y >= ENEMY_EDGE)
		return ENEMY_EDGE - x;
	return (ENEMY_EDGE << 1) - x - y;
}

int my_base_dist(int x, int y)//use my dis
{
	if ((x <= MY_EDGE) && (y <= MY_EDGE))
		return 0;
	if (x <= MY_EDGE)
		return y - MY_EDGE;
	if (y <= MY_EDGE)
		return x - MY_EDGE;
	return x + y - (MY_EDGE << 1);
}

int mydist(Position posa,Position posb)
{
	return abs(posa.x - posb.x) + abs(posa.y - posb.y);
}

void init()
{
	colormap[0][0] = 0;

	for (BuildingType i = Shannon; i <= Tony_Stark; i = BuildingType(i + 1))
		SoldierCD[OriginalBuildingAttribute[i][TARGET]] = OriginalBuildingAttribute[i][CD];
	for (int i = 0; i < ROADMAX; ++i) savearea_everypoint_flag[i] = false;
	for (int i = 1; i <= ROADCNT; ++i)
	{
		kernel_defense[i] = Position(-1, -1);
		kernel_target[i] = Position(-1, -1);
	}

	//BFS for color map generation

	int head = 0, tail = 0;
	int tmpx, tmpy;
	//Get Road first
	LOGN("Get Road first");
	for (int i = 0; i < BASE_SIZE + 1; ++i)
		if (ts19_map[trans(7)][trans(i)] == 1)
		{
			++ROADCNT;
			roadroot[ROADCNT] = Position(7, i);
			colormap[7][i] = ROADCNT;
			queue[tail] = Position(7,i);
			root[tail] = queue[tail];
			++tail;
			int nowx = 7, nowy = i, tnowx, tnowy;
			bool find = true;
			while (find)
			{
				find = false;
				for (int j = 0; j < 4; ++j)
					if ((nowx + tx4[j] >= 0) && (nowx + tx4[j] < MAP_SIZE) && (nowy + ty4[j] >= 0) && (nowy + ty4[j] < MAP_SIZE))
						if ((colormap[nowx + tx4[j]][nowy + ty4[j]] == 0) && (ts19_map[trans(nowx + tx4[j])][trans(nowy + ty4[j])] == 1))
						{
							tnowx = nowx + tx4[j];
							tnowy = nowy + ty4[j];
							colormap[tnowx][tnowy] = ROADCNT;
							queue[tail] = Position(tnowx,tnowy);
							root[tail] = root[tail - 1];
							++tail;
							find = true;
							if (i != 7) break;
						}
				nextpos[nowx][nowy] = Position(tnowx, tnowy);
				if (find)
				{
					nowx = tnowx;
					nowy = tnowy;
				}
			}	
			nextpos[nowx][nowy] = Position(-1, -1);
		}
	for (int i = BASE_SIZE-1; i >= 0; --i)
		if (ts19_map[trans(i)][trans(7)] == 1)
		{
			++ROADCNT;
			roadroot[ROADCNT] = Position(i, 7);
			colormap[i][7] = ROADCNT;
			queue[tail] = Position(i,7);
			root[tail] = queue[tail];
			++tail;
			int nowx = i, nowy = 7;
			bool find = true;
			while (find)
			{
				find = false;
				for (int j = 0; j < 4; ++j)
					if ((nowx + tx4[j] >= 0) && (nowx + tx4[j] < MAP_SIZE) && (nowy + ty4[j] >= 0) && (nowy + ty4[j] < MAP_SIZE))
						if ((colormap[nowx + tx4[j]][nowy + ty4[j]] == 0) && (ts19_map[trans(nowx + tx4[j])][trans(nowy + ty4[j])] == 1))
						{
							nextpos[nowx][nowy] = Position(nowx + tx4[j], nowy + ty4[j]);
							nowx += tx4[j];
							nowy += ty4[j];
							colormap[nowx][nowy] = ROADCNT;
							queue[tail] = Position(nowx,nowy);
							root[tail] = root[tail - 1];
							++tail;
							find = true;
							break;
						}
			}
			nextpos[nowx][nowy] = Position(-1, -1);
		}
	//BFS to get color and dis
	LOGN("BFS to get color and dis");
	while (head < tail)
	{
		int now = 0, nowx = queue[head].x, nowy = queue[head].y;
		if (mindismap[nowx][nowy] > 0) break;
		for (; now < 4; ++now)
		{
			if ((nowx + tx4[now] < 0) || (nowx + tx4[now] >= MAP_SIZE) || (nowy + ty4[now] < 0) || (nowy + ty4[now] >= MAP_SIZE))
				continue;
			if (head == 0)
				if (ts19_map[trans(nowx + tx4[now])][trans(nowy + ty4[now])] != 2)
					continue;
				else break;
			if ((root[head].x!=root[head-1].x)||(root[head].y != root[head - 1].y))
				if (ts19_map[trans(nowx + tx4[now])][trans(nowy + ty4[now])] != 2)
					continue;
				else break;
			if ((nowx + tx4[now]== queue[head - 1].x)&&(nowy + ty4[now] == queue[head - 1].y))
				break;
		}
		int flag = -1;
		for (int i = 0; i < 3; ++i)
		{
			++now;
			if (now >= 4) now -= 4;
			if ((nowx + tx4[now] < 0) || (nowx + tx4[now] >= MAP_SIZE) || (nowy + ty4[now] < 0) || (nowy + ty4[now] >= MAP_SIZE))
				continue;
			if (((root[head+1].x==root[head].x) && (root[head + 1].y == root[head].y) &&(nowx + tx4[now] == queue[head + 1].x)&&(nowy + ty4[now] == queue[head + 1].y))
				|| (((root[head + 1].x != root[head].x)||(root[head + 1].y != root[head].y)) && ((ts19_map[trans(nowx + tx4[now])][trans(nowy + ty4[now])] == 2))))
			{
				flag = 0;
				continue;
			}
			if ((colormap[nowx + tx4[now]][nowy + ty4[now]] == 0)&&(ts19_map[trans(nowx + tx4[now])][trans(nowy + ty4[now])] == 0))
			{
				queue[tail] = Position(nowx + tx4[now], nowy + ty4[now]);
				root[tail] = root[head];
				mindismap[nowx + tx4[now]][nowy + ty4[now]] = 1;
				++discount[1];
				colormap[nowx + tx4[now]][nowy + ty4[now]] = ROADMAX + colormap[nowx][nowy] + flag;
				++tail;
			}
		}
		++head;
	}
	
	while (head < tail)
	{
		int nowx = queue[head].x, nowy = queue[head].y;
		for (int i = 0; i < 4; ++i)
		{
			tmpx = nowx + tx4[i];
			tmpy = nowy + ty4[i];
			if ((tmpx < 0) || (tmpx >= MAP_SIZE) || (tmpy < 0) || (tmpy >= MAP_SIZE))
				continue;
			if (colormap[tmpx][tmpy] > 0)
				continue;
			if (ts19_map[tmpx][tmpy] > 0)
				continue;
			queue[tail] = Position(tmpx, tmpy);
			colormap[tmpx][tmpy] = colormap[nowx][nowy];
			mindismap[tmpx][tmpy] = mindismap[nowx][nowy]+1;
			if (mindismap[tmpx][tmpy] > area_dis) ++discount[area_dis];
			else ++discount[mindismap[tmpx][tmpy]];
			++tail;
		}
		++head;
	}

	//Decide the distance for candidate area
	LOGN("Decide the distance for candidate area")
	int count = 0;
	for (; area_dis > 0; --area_dis)
	{
		count += discount[area_dis];
		if (count > MAX_BD_NUM + MAX_BD_NUM_PLUS * 4)
			break;
	};

	//Generate area
	minareadis = new int[ROADCNT+1];
	areacount = new int[ROADCNT+1];
	areasortid = new int[ROADCNT + 1];
	saveArea = new std::vector<Position>[ROADCNT + 1];
	saveArea_enemydis = new std::vector<int>[ROADCNT + 1];
	resource_area = new bool[ROADCNT + 1];
	int tmpcolor,tmp_enemy_base_dis;
	LOG("GenerateArea");
	for (int i = 0; i < ROADCNT + 1; ++i)
	{
		minareadis[i] = 10000;
		areacount[i] = 0;
	}
	for (int x = 0; x < MAP_SIZE; ++x)
		for (int y = 0; y < MAP_SIZE; ++y)
		{
			if (mindismap[x][y] < area_dis) continue;
			if (colormap[x][y] < ROADMAX) continue;
			tmpcolor = colormap[x][y] - ROADMAX;
			tmp_enemy_base_dis = enemy_base_dist(x, y);
			minareadis[tmpcolor] = mymin(minareadis[tmpcolor], tmp_enemy_base_dis);
			++areacount[tmpcolor];
			saveArea[tmpcolor].push_back(Position(x, y));
			saveArea_enemydis[tmpcolor].push_back(tmp_enemy_base_dis);
		}
	
	//sort area(from short to long)
	bool *areaflag = new bool[ROADCNT + 1];
	int mindis_tmp,maxareacount_tmp,minarea_tmp;
	for (int i = 0; i < ROADCNT + 1; ++i) areaflag[i] = false;
	for (int i = 0; i < ROADCNT + 1; ++i)
	{
		mindis_tmp = 10000; maxareacount_tmp = 0;
		for (int j = 0; j < ROADCNT + 1; ++j)
		{
			if (areaflag[j]) continue;
			if (areacount[j] < maxareacount_tmp) continue;
			if (areacount[j] > maxareacount_tmp)
			{
				maxareacount_tmp = areacount[j];
				mindis_tmp = minareadis[j];
				minarea_tmp = j;
				continue;
			}
			if (minareadis[j] < mindis_tmp)
			{
				mindis_tmp = minareadis[j];
				minarea_tmp = j;
			}
		}
		areasortid[i] = minarea_tmp;
		areaflag[minarea_tmp] = true;
	}

	
#ifdef DEBUG
	/*
	LOGN("PRINT INIT INFO");
	cv::Mat map(MAP_SIZE, MAP_SIZE, CV_8UC3);
	cv::Mat heatmap(MAP_SIZE, MAP_SIZE, CV_8UC1);
	cv::Mat savemap(MAP_SIZE, MAP_SIZE, CV_8UC1);
	int coef = 250 / (ROADCNT+1),maxheat=0;
	for (int  i=0;i<map.rows;++i)
		for (int j = 0; j < map.cols; ++j)
		{
			if (colormap[i][j] < ROADMAX)
			{
				map.at<cv::Vec3b>(i, j)[0] = colormap[i][j] * coef;
				map.at<cv::Vec3b>(i, j)[1] = 0;
				map.at<cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{
				map.at<cv::Vec3b>(i, j)[0] = 0;
				map.at<cv::Vec3b>(i, j)[1] = (colormap[i][j] - ROADMAX + 1) * coef;
				map.at<cv::Vec3b>(i, j)[2] = 0;
			}
			if (mindismap[i][j] > maxheat)
				maxheat = mindismap[i][j];
		}
	for (int cnt = 0; cnt < ROADCNT + 1; ++cnt)
	{
		for (int i = 0; i < saveArea[cnt].size(); ++i)
			savemap.at<byte>(saveArea[cnt][i].x, saveArea[cnt][i].y) = 255;
	}
	maxheat = 250 / maxheat;
	for (int i = 0; i < map.rows; ++i)
		for (int j = 0; j < map.cols; ++j)
			heatmap.at<byte>(i, j) = mindismap[i][j] * maxheat;
	cv::imwrite("heatmap.png", heatmap);
	cv::imwrite("map.png", map);
	cv::imwrite("savemap.png", savemap);
	*/

	file << "area_dis: " << area_dis << endl;
	for (int i = 0; i < ROADCNT + 1; ++i)
		file << "area" << i << " :(dis," << minareadis[i] << ") (cnt," << areacount[i] << ")" << endl;
	cout << "Output init" << endl;

#endif 


	delete root;
	delete areaflag;
	return;
}

void drawbuildingmap(int x, int y,bool flag)
{
	//flag 0 is myself,flag 1 is enemy
	//0 - useless;1 - road;2 - building&base;3 - useful;
	if (flag == 0)
	{
		for (int i = 0; i <= MAX_BD_RANGE; ++i)
			for (int j = 0; j + i <= MAX_BD_RANGE; ++j)
			{
				if (x - i >= 0)
				{
					if (y - j >= 0)
					{
						if (buildmap[x - i][y - j] == 0) buildmap[x - i][y - j] = 3;
					}
					if (y + j < MAP_SIZE)
					{
						if (buildmap[x - i][y + j] == 0) buildmap[x - i][y + j] = 3;
					}
				}
				if (x + i < MAP_SIZE)
				{
					if (y - j >= 0)
					{
						if (buildmap[x + i][y - j] == 0) buildmap[x + i][y - j] = 3;
					}
					if (y + j < MAP_SIZE)
					{
						if (buildmap[x + i][y + j] == 0) buildmap[x + i][y + j] = 3;
					}
				}
			}
	}
	else
	{
		for (int i = 0; i <= MAX_BD_RANGE; ++i)
			for (int j = 0; j + i <= MAX_BD_RANGE; ++j)
			{
				if (x - i >= 0)
				{
					if (y - j >= 0)
					{
						if (enemy_buildmap[x - i][y - j] == 0) enemy_buildmap[x - i][y - j] = 3;
					}
					if (y + j < MAP_SIZE)
					{
						if (enemy_buildmap[x - i][y + j] == 0) enemy_buildmap[x - i][y + j] = 3;
					}
				}
				if (x + i < MAP_SIZE)
				{
					if (y - j >= 0)
					{
						if (enemy_buildmap[x + i][y - j] == 0) enemy_buildmap[x + i][y - j] = 3;
					}
					if (y + j < MAP_SIZE)
					{
						if (enemy_buildmap[x + i][y + j] == 0) enemy_buildmap[x + i][y + j] = 3;
					}
				}
			}
	}

	buildmap[x][y] = 2;
	enemy_buildmap[x][y] = 2;
	int unlegal_dis = MIN_BD_RANGE;
	if (setup_control) unlegal_dis = MAX_BD_RANGE;
	for (int i = 0; i < unlegal_dis; ++i)
		for (int j = 0; j + i < unlegal_dis; ++j)
		{
			if (x - i >= 0)
			{
				if (y - j >= 0) { buildmap[x - i][y - j] = 2; enemy_buildmap[x - i][y - j] = 2; }
				if (y + j < MAP_SIZE) { buildmap[x - i][y + j] = 2; enemy_buildmap[x - i][y + j] = 2; }
			}
			if (x + i < MAP_SIZE)
			{
				if (y - j >= 0) { buildmap[x + i][y - j] = 2; enemy_buildmap[x + i][y - j] = 2; }
				if (y + j < MAP_SIZE) { buildmap[x + i][y + j] = 2; enemy_buildmap[x + i][y + j] = 2; }
			}
		}

	//if (x >= 1) { buildmap[x - 1][y] = 2; enemy_buildmap[x - 1][y] = 2; }
	//if (y >= 1) { buildmap[x][y - 1] = 2; enemy_buildmap[x][y - 1] = 2; }
	//if (x + 1 < MAP_SIZE) { buildmap[x + 1][y] = 2; enemy_buildmap[x + 1][y] = 2; }
	//if (y + 1 < MAP_SIZE) { buildmap[x][y + 1] = 2; enemy_buildmap[x][y + 1] = 2; }
	//
	return;
}

int initBuildmap()
{
	//buildmap: 0 - useless;1 - road;2 - building&base;3 - useful;
	//memcpy((void*)buildmap, (void*)ts19_map, MAP_SIZE*MAP_SIZE * sizeof(int));
	for (int i = 0; i < MAP_SIZE; ++i)
		for (int j = 0; j < MAP_SIZE; ++j)
		{
			buildmap[i][j] = ts19_map[i][j]; 
			enemy_buildmap[i][j] = ts19_map[i][j];
		}
	//init for data
	sell_for_defense = false;
	sell_for_produce = false;
	sell_num = 0;
	my_bd_num = state->building[ts19_flag].size() - 1;
	my_resource_num = 0; 
	enemy_resource_num = 0;
	enemy_procude_num = 0;
	my_produce_num = 0;
	my_defense_num = 0;
	can_upgrade_reource_num = 0;
	can_upgrade_produce_num = 0;
	can_upgrade_defense_num = 0;
	for (int i = 0; i < BUILDING_TYPE; ++i) { my_building_num[i] = 0; enemy_building_num[i] = 0;}
	for (int i = 0; i < state->building[ts19_flag].size(); ++i) sell_list[i] = false;
	for (int i = 1; i <= ROADCNT; ++i) musk_heal[i] = 0;
	for (int i = 0; i < ROADCNT + 1; ++i) resource_area[i] = false;
	
	myresource = state->resource[ts19_flag].resource;
	mybdpoint = state->resource[ts19_flag].building_point;
	science_factor = 1 + state->age[ts19_flag] * AGE_INCREASE;
	
	//Cal my produce in every road
	for (BuildingType building = __Base; building < Building_Type; building = BuildingType(building + 1))
		for (int i = 0; i <= ROADMAX; ++i)
			soldiernum_in_road[i][building] = 0;
	Position soldier_produce_pos,kernel_pos,target_pos;
	int defense_roadnum;

	// draw my buildingmap and enemy's buildingmap
	for (int i = 0; i < BASE_SIZE + BD_RANGE_FROM_BASE; ++i)
		for (int j = 0; j < BASE_SIZE + BD_RANGE_FROM_BASE; ++j)
		{
			if (buildmap[i][j] == 0) buildmap[i][j] = 3;
		}
	for (int i = MAP_SIZE - 1; i > MAP_SIZE - 1 - (BASE_SIZE + BD_RANGE_FROM_BASE); --i)
		for (int j = MAP_SIZE - 1; j > MAP_SIZE - 1 - (BASE_SIZE + BD_RANGE_FROM_BASE); --j)
		{
			if (enemy_buildmap[i][j] == 0) enemy_buildmap[i][j] = 3;
		}
	
	for (int i = 0; i < state->building[ts19_flag].size(); ++i)
	{
		++my_building_num[state->building[ts19_flag][i].building_type];
		if (state->building[ts19_flag][i].building_type == __Base)
			continue;
		drawbuildingmap(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y),0);
		if (state->building[ts19_flag][i].building_type == Programmer)
		{
			++my_resource_num;
			if (state->building[ts19_flag][i].level < state->age[ts19_flag]) ++can_upgrade_reource_num;
			if (mindismap[trans(state->building[ts19_flag][i].pos.x)][trans(state->building[ts19_flag][i].pos.y)]>area_dis)
			resource_area[colormap[trans(state->building[ts19_flag][i].pos.x)][trans(state->building[ts19_flag][i].pos.y)] - ROADMAX] = true;
		}
		if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == PRODUCTION_BUILDING)
		{
			++my_produce_num;
			if (state->building[ts19_flag][i].level < state->age[ts19_flag]) ++can_upgrade_produce_num;
			soldier_produce_pos = state->building[ts19_flag][i].pos;
			soldier_produce_pos = my_soldier_pos[trans(soldier_produce_pos.x)][trans(soldier_produce_pos.y)];
			++soldiernum_in_road[colormap[soldier_produce_pos.x][soldier_produce_pos.y]][state->building[ts19_flag][i].building_type];
		}
		if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == DEFENSIVE_BUILDING)
		{
			++my_defense_num;
			if (state->building[ts19_flag][i].level < state->age[ts19_flag]) ++can_upgrade_defense_num;
			if (defense_control)
			{
				soldier_produce_pos = state->building[ts19_flag][i].pos;
				soldier_produce_pos = Position(trans(soldier_produce_pos.x), trans(soldier_produce_pos.y));
				defense_roadnum = defense_road[soldier_produce_pos.x][soldier_produce_pos.y];
				kernel_pos = kernel_defense[defense_roadnum];
				if ((kernel_pos.x != -1) || (kernel_pos.y != -1))
				{
					target_pos = kernel_target[defense_roadnum];
					if ((soldier_produce_pos.x <= target_pos.x) && (soldier_produce_pos.y <= target_pos.y) &&
						(soldier_produce_pos.x + soldier_produce_pos.y >= kernel_pos.x + kernel_pos.y) &&
						(soldier_produce_pos.x + soldier_produce_pos.y <= kernel_pos.x + kernel_pos.y + defense_gap))
					{
						++soldiernum_in_road[defense_roadnum][state->building[ts19_flag][i].building_type];
						if (state->building[ts19_flag][i].building_type == Musk)
							musk_heal[defense_roadnum] += double(state->building[ts19_flag][i].heal) / (OriginalBuildingAttribute[Musk][ORIGINAL_HEAL] * (1 + 0.5*state->building[ts19_flag][i].level));
					}
						
				}	
			}
		}
	}
	for (int i = 1; i <= ROADCNT; ++i)
		if (musk_heal[i] < 0.5)
			soldiernum_in_road[i][Musk] = 0;

	for (int i = 0; i < state->building[1-ts19_flag].size(); ++i)
	{
		++enemy_building_num[state->building[1-ts19_flag][i].building_type];
		if (state->building[1-ts19_flag][i].building_type == __Base)
			continue;
		if (OriginalBuildingAttribute[state->building[1 - ts19_flag][i].building_type][BUILDING_TYPE] == PRODUCTION_BUILDING)
			++enemy_procude_num;
		if (state->building[1 - ts19_flag][i].building_type == Programmer)
			++enemy_resource_num;
		drawbuildingmap(trans(state->building[1-ts19_flag][i].pos.x), trans(state->building[1-ts19_flag][i].pos.y), 1);
	}

	//Calculate mindis of enemy's building
	nearest_enemybuilding_dis = 10000;
	int dis;
	for (int i = 0; i < MAP_SIZE; ++i) 
		for (int j = 0; j < MAP_SIZE; ++j)
			if (enemy_buildmap[i][j] == 3)
			{
				dis = i + j;
				if (dis < nearest_enemybuilding_dis) nearest_enemybuilding_dis = dis;
			}

	return 0;
}

void drawfiremap_inbase(Position pos, BuildingType building_type, int id = -1)//this function is so difficult to finish
{
	//-defense strategy in base
	//	- firemapinbase, just consider the building inside base
	//	- also, just consider the soldier across half of the map
	//	- to determine the num of defense building, we cal defense CD * soldier speed to find max soldier freq
	if (OriginalBuildingAttribute[building_type][BUILDING_TYPE] != DEFENSIVE_BUILDING) return;
	int x = pos.x, y = pos.y;
	if (my_base_dist(x, y) > MAP_SIZE) return;
	bool* roadflag = new bool[ROADCNT + 1];
	for (int i = 1; i <= ROADCNT; ++i) roadflag[i] = false;
	int dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	//find road that the building can reach
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (y - ty >= 0)
					if (ts19_map[x - tx][y - ty] == 1) roadflag[colormap[x - tx][y - ty]] = true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x - tx][y + ty] == 1) roadflag[colormap[x - tx][y + ty]] = true;
			}
			if (x + tx < MAP_SIZE)
			{
				if (y - ty >= 0)
					if (ts19_map[x + tx][y - ty] == 1) roadflag[colormap[x + tx][y - ty]] = true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x + tx][y + ty] == 1) roadflag[colormap[x + tx][y + ty]] = true;
			}
		}

	int cnt = 0;
	bool soldierflag;
	//delete useless road
	for (int i=1;i<=ROADCNT;++i)
		if (roadflag[i])
		{
			soldierflag = false;
			for (SoldierName soldier = BIT_STREAM; soldier < Soldier_Type; soldier = SoldierName(soldier + 1))
			{
				if ((OriginalBuildingAttribute[building_type][TARGET] != ALL) &&
					(OriginalBuildingAttribute[building_type][TARGET] != OriginalSoldierAttribute[soldier][SOLDIER_TYPE]))
					continue;
				if ((building_type == Musk) && (soldier != TURING_MACHINE) && (soldier != ULTRON)) continue;
				if (OriginalSoldierAttribute[soldier][ATTACK_RANGE] > OriginalBuildingAttribute[building_type][ORIGINAL_RANGE])
					continue;
				if (nearest_soldierdis[i][soldier] < MAP_SIZE)
				{
					soldierflag = true;
					break;
				}
			}
			roadflag[i] = soldierflag;
			if (soldierflag) ++cnt;
		}
	//distribute fire to every road
	if (cnt != 0)
		cnt = MAX_FIRE / cnt;
	if (building_type == Musk) cnt = MAX_FIRE;
	for (int i = 1; i <= ROADCNT; ++i)
		if (roadflag[i])
		{
			firemap_inbase[i][building_type] += cnt;
			if (id >= 0) building_firemap[id][i] += cnt;
		}
			

	delete roadflag;
	return;
}

void drawfiremap_outside(Position pos, BuildingType building_type)
{
	if (OriginalBuildingAttribute[building_type][BUILDING_TYPE] != DEFENSIVE_BUILDING) return;
	int x = pos.x, y = pos.y;
	int cnt = 0;
	int dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	std::list<SoldierPoint>::iterator iter = SoldierPointList.begin(),tmpiter;
	bool possame_flag;
	if (building_type == Musk)
	{
		for (; iter != SoldierPointList.end(); ++iter)
		{
			if ((iter->soldier_type!=TURING_MACHINE)&&(iter->soldier_type!=ULTRON))
				continue;
			if (mydist(pos, iter->pos) > dis) continue;
			iter->musk = true;
		}
		return;
	}
	for (; iter != SoldierPointList.end(); ++iter)
	{
		if ((OriginalBuildingAttribute[building_type][TARGET] != ALL) &&
			(OriginalBuildingAttribute[building_type][TARGET] != OriginalSoldierAttribute[iter->soldier_type][SOLDIER_TYPE]))
			continue;
		if (mydist(pos, iter->pos) > dis) continue;
		if (OriginalBuildingAttribute[building_type][AOE] > 0)
		{

			possame_flag = false;
			for (tmpiter = SoldierPointList.begin(); tmpiter != iter; ++tmpiter)
				if (mydist(tmpiter->pos,iter->pos)< OriginalBuildingAttribute[building_type][AOE])
				{
					possame_flag = true;
					break;
				}
			if (!possame_flag) ++cnt;
		}
		else ++cnt;
	}
	if (cnt == 0) return;
	float fire_distribute = 0;
	
	if ((building_type!=Musk)&&(building_type!=Hawkin))
		fire_distribute = float(OriginalBuildingAttribute[building_type][ORIGINAL_ATTACK]) / (cnt*OriginalBuildingAttribute[building_type][CD]);
	//if (OriginalBuildingAttribute[building_type][AOE] > 0) fire_distribute *= 2;

	//distribute fire to every point
	if (building_type==Hawkin)
		for (; iter != SoldierPointList.end(); ++iter)
		{
			if ((OriginalBuildingAttribute[building_type][TARGET] != ALL) &&
				(OriginalBuildingAttribute[building_type][TARGET] != OriginalSoldierAttribute[iter->soldier_type][SOLDIER_TYPE]))
				continue;
			if (mydist(pos, iter->pos) > dis) continue;
			iter->fire += float(OriginalSoldierAttribute[iter->soldier_type][SOLDIER_ORIGINAL_HEAL]) / cnt;
		}
	for (; iter != SoldierPointList.end(); ++iter)
	{
		if ((OriginalBuildingAttribute[building_type][TARGET] != ALL) &&
			(OriginalBuildingAttribute[building_type][TARGET] != OriginalSoldierAttribute[iter->soldier_type][SOLDIER_TYPE]))
			continue;
		if (mydist(pos, iter->pos) > dis) continue;
		iter->fire += fire_distribute;
	}

	return;
}

int initFiremap()
{
	for (int i = 1; i <= ROADCNT; ++i)
		for (BuildingType j = Bool; j <= Hawkin; j = BuildingType(j + 1))
			firemap_inbase[i][j] = 0;
	for (int i = 0; i < BUILDINGMAX; ++i)
		for (int j = 0; j <= ROADMAX; ++j)
			building_firemap[i][j] = 0;
	//what i have
	Position pos;
	for (int i = 0; i < state->building[ts19_flag].size(); ++i)
	{
		pos = state->building[ts19_flag][i].pos;
		drawfiremap_inbase(Position(trans(pos.x), trans(pos.y)), state->building[ts19_flag][i].building_type,i);
	}

	//what i need
	//cal soldier freq
	int roadlen = 0, soldier_count, max_soldier_count;
	Position head, tail;
	for (SoldierName soldier = BIT_STREAM; soldier < Soldier_Type; soldier = SoldierName(soldier + 1))
	{
		roadlen = OriginalSoldierAttribute[soldier][SPEED] * SoldierCD[soldier];
		for (int i = 1; i <= ROADCNT; ++i)
		{
			head = roadroot[i];
			tail = head;
			soldier_count = soldier_distrubutemap[head.x][head.y][soldier];
			for (int j = 1; j < roadlen; ++j)
			{
				tail = nextpos[tail.x][tail.y];
				soldier_count += soldier_distrubutemap[tail.x][tail.y][soldier];
			}
			max_soldier_count = soldier_count;
			while (my_base_dist(head.x, head.y) < MAP_SIZE)
			{
				tail = nextpos[tail.x][tail.y];
				if ((tail.x < 0) || (tail.y < 0)) break;
				soldier_count -= soldier_distrubutemap[head.x][head.y][soldier];
				soldier_count += soldier_distrubutemap[tail.x][tail.y][soldier];
				head = nextpos[head.x][head.y];
				if (soldier_count > max_soldier_count) max_soldier_count = soldier_count;
			}
			soldier_freq[i][soldier] = max_soldier_count;
		}
	}
	//Set fire
	for (int i = 1; i <= ROADCNT; ++i)
		for (BuildingType building_type = Bool; building_type <= Hawkin; building_type = BuildingType(building_type + 1))
			needfiremap_inbase[i][building_type] = 0;
	
	if (state->age[ts19_flag] == BIT)
	{
		for (int i = 1; i <= ROADCNT; ++i)
			needfiremap_inbase[i][Bool] = soldier_freq[i][BIT_STREAM] * MAX_FIRE;
		return 0;
	}
	if (state->age[ts19_flag] == CIRCUIT)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			needfiremap_inbase[i][Bool] = soldier_freq[i][BIT_STREAM] * MAX_FIRE;
			needfiremap_inbase[i][Ohm] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE];
			if (needfiremap_inbase[i][Ohm] == 0) continue;
			if (needfiremap_inbase[i][Ohm] < ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
				else needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			if (needfiremap_inbase[i][Ohm] > OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = mymin(needfiremap_inbase[i][Ohm], OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]));
				else needfiremap_inbase[i][Ohm] = OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Ohm] *= MAX_FIRE;
		}	
		return 0;
	} 
	if (state->age[ts19_flag] == PROCESSOR)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			needfiremap_inbase[i][Bool] = soldier_freq[i][BIT_STREAM] * MAX_FIRE;
			needfiremap_inbase[i][Mole] = soldier_freq[i][ENIAC] * MAX_FIRE / 2;
			needfiremap_inbase[i][Ohm] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE];
			if (needfiremap_inbase[i][Ohm] == 0) continue;
			if (needfiremap_inbase[i][Ohm] < ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
				else needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			if (needfiremap_inbase[i][Ohm] > OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = mymin(needfiremap_inbase[i][Ohm], OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]));
				else needfiremap_inbase[i][Ohm] = OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Ohm] *= MAX_FIRE;
			
		}
		return 0;
	}
	if (state->age[ts19_flag] == ALGORITHM)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			needfiremap_inbase[i][Bool] = soldier_freq[i][BIT_STREAM] * MAX_FIRE;
			needfiremap_inbase[i][Mole] = soldier_freq[i][ENIAC] * MAX_FIRE / 2;
			needfiremap_inbase[i][Monte_Carlo] = soldier_freq[i][OPTICAL_FIBER] * MAX_FIRE * 2 / 3;
			needfiremap_inbase[i][Ohm] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE];
			if (needfiremap_inbase[i][Ohm] == 0) continue;
			if (needfiremap_inbase[i][Ohm] < ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
				else needfiremap_inbase[i][Ohm] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			if (needfiremap_inbase[i][Ohm] > OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]))
			{
				if (soldier_freq[i][VOLTAGE_SOURCE] > 0)
					needfiremap_inbase[i][Ohm] = mymin(needfiremap_inbase[i][Ohm], OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[VOLTAGE_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]));
				else needfiremap_inbase[i][Ohm] = OriginalBuildingAttribute[Ohm][CD] * ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Ohm][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Ohm] *= MAX_FIRE;

		}
		return 0;
	}
	if (state->age[ts19_flag] == NETWORK)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			needfiremap_inbase[i][Mole] = (soldier_freq[i][ENIAC] + soldier_freq[i][TURING_MACHINE] + soldier_freq[i][ULTRON]) * MAX_FIRE / 2;
			//needfiremap_inbase[i][Monte_Carlo] = soldier_freq[i][OPTICAL_FIBER] * MAX_FIRE * 2 / 3;
			needfiremap_inbase[i][Larry_Roberts] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE]
				+ soldier_freq[i][BIT_STREAM] + soldier_freq[i][PACKET] + soldier_freq[i][OPTICAL_FIBER];
			if (needfiremap_inbase[i][Larry_Roberts] == 0) continue;
			if (needfiremap_inbase[i][Larry_Roberts] < ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3)))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3));
			}
			if (needfiremap_inbase[i][Larry_Roberts] > ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]);
				needfiremap_inbase[i][Larry_Roberts] = 2 * ceil(OriginalSoldierAttribute[OPTICAL_FIBER][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Larry_Roberts] *= MAX_FIRE;
		}
		return 0;
	}
	if (state->age[ts19_flag] == AI)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			//needfiremap_inbase[i][Monte_Carlo] = soldier_freq[i][OPTICAL_FIBER] * MAX_FIRE * 2 / 3;
			needfiremap_inbase[i][Hawkin] = (soldier_freq[i][ENIAC] + soldier_freq[i][TURING_MACHINE] + soldier_freq[i][ULTRON]);
			if (needfiremap_inbase[i][Hawkin] > 0) needfiremap_inbase[i][Hawkin] = MAX_FIRE;
			if ((soldier_freq[i][TURING_MACHINE] > 0) || (soldier_freq[i][ULTRON] > 0)) needfiremap_inbase[i][Musk] = MAX_FIRE;
			needfiremap_inbase[i][Larry_Roberts] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE]
				+ soldier_freq[i][BIT_STREAM] + soldier_freq[i][PACKET] + soldier_freq[i][OPTICAL_FIBER];
			if (needfiremap_inbase[i][Larry_Roberts] == 0) continue;
			if (needfiremap_inbase[i][Larry_Roberts] < ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3)))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3));
			}
			if (needfiremap_inbase[i][Larry_Roberts] > ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]);
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[OPTICAL_FIBER][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Larry_Roberts] *= MAX_FIRE;
		}
		return 0;
	}
	return 0;
}

int initFiremap_point()
{
	std::list<SoldierPoint>::iterator iter = SoldierPointList.begin();
	for (; iter != SoldierPointList.end(); ++iter) 
	{
		iter->fire = 0; iter->musk = false;
	}
	//what i have
	Position pos;
	for (int i = 0; i < state->building[ts19_flag].size(); ++i)
	{
		pos = state->building[ts19_flag][i].pos;
		drawfiremap_outside(Position(trans(pos.x), trans(pos.y)), state->building[ts19_flag][i].building_type);
	}
	
	return 0;
}

void Getsoldierpoint()
{
	// clear List
	std::list<SoldierPoint>::iterator iter = SoldierPointList.begin();
	for (; iter != SoldierPointList.end();)
	{
		--iter->cd;
		if (iter->cd <= 0) iter = SoldierPointList.erase(iter++);
		else ++iter;
	}
	// Update List
	for (int i = 0; i < state->soldier[1 - ts19_flag].size(); ++i)
	{
		if (state->soldier[1 - ts19_flag][i].unit_id <= maxsoliderid) continue;
		maxsoliderid = state->soldier[1 - ts19_flag][i].unit_id;
		SoldierPointList.push_back(SoldierPoint(state->soldier[1 - ts19_flag][i].pos,
			SoldierCD[state->soldier[1 - ts19_flag][i].soldier_name], 
			state->soldier[1 - ts19_flag][i].soldier_name));
	}
	return;
}

int soldierSituation()
{
	// Update count
	for (int i = 0; i < Soldier_Type; ++i)
		for (int j = 0; j < ROADMAX; ++j)
		{
			soldiercount[j][i] = 0;
			nearest_soldierdis[j][i] = MAX_SOLDIER_DIS;
		}
	int road,dis;
	for (SoldierName soldier = BIT_STREAM; soldier < Soldier_Type; soldier = SoldierName(soldier + 1))
		for (int x = 0; x < MAP_SIZE; ++x)
			for (int y = 0; y < MAP_SIZE; ++y)
				soldier_distrubutemap[x][y][soldier] = 0;
	Position pos;
	for (int i = 0; i < state->soldier[1 - ts19_flag].size(); ++i)
	{
		road = colormap[trans(state->soldier[1 - ts19_flag][i].pos.x)][trans(state->soldier[1 - ts19_flag][i].pos.y)];
		if (road > ROADCNT) continue;
		++soldier_distrubutemap[trans(state->soldier[1 - ts19_flag][i].pos.x)][trans(state->soldier[1 - ts19_flag][i].pos.y)][state->soldier[1 - ts19_flag][i].soldier_name];
		++soldiercount[road][state->soldier[1 - ts19_flag][i].soldier_name];
		dis = my_base_dist(trans(state->soldier[1 - ts19_flag][i].pos.x), trans(state->soldier[1 - ts19_flag][i].pos.y));
		if (dis < nearest_soldierdis[road][state->soldier[1 - ts19_flag][i].soldier_name])
			nearest_soldierdis[road][state->soldier[1 - ts19_flag][i].soldier_name] = dis;
	}
	return 0;
}

bool myConstruct(BuildingType building_type, Position pos, Position soldier_pos = Position(0, 0))//input my pos,it will change pos into real pos
{
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE])
		return false;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT])
		return false;
	if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag]) return false;
	construct(building_type, Position(trans(pos.x),trans(pos.y)), Position(trans(soldier_pos.x),trans(soldier_pos.y)));
	++command_num;
	drawbuildingmap(pos.x, pos.y,0);
	drawfiremap_inbase(pos, building_type,-1);
	my_soldier_pos[pos.x][pos.y] = soldier_pos;
	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE];// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];// *science_factor;
	last_pos = pos;
	++my_bd_num;

#ifdef DEBUG
	
	file << "turn: " << state->turn << " type:" << building_type << " pos: (" << pos.x << " ," <<pos.y<<") solider:("<< soldier_pos.x<<","<<soldier_pos.y<<")"<<endl;

#endif // DEBUG


	return true;
}

bool myupgrade(int unit_id, BuildingType building_type, int id = 0)
{
	if (sell_list[id]) return 0;
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE] * 0.5)
		return false;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT] * 0.5)
		return false;
	upgrade(unit_id);
	++command_num;
	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE] * 0.5;// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT] * 0.5;// *science_factor;

#ifdef DEBUG

	file << "turn: " << state->turn << " type:" << building_type << " upgrade: " << unit_id << endl;

#endif // DEBUG
	
	return true;
}

bool mysell(int id)
{
	if (sell_list[id]) return 0;
	sell(state->building[ts19_flag][id].unit_id);
	++command_num;
	++sell_num;
	sell_list[id] = true;
	BuildingType building_type = state->building[ts19_flag][id].building_type;
	if (OriginalBuildingAttribute[building_type][BUILDING_TYPE]==DEFENSIVE_BUILDING)
	for (int road = 1; road <= ROADCNT; ++road)
		firemap_inbase[road][building_type] -= building_firemap[id][road];
#ifdef DEBUG

	file << "turn: " << state->turn << " type:" << building_type << " sell: " << state->building[ts19_flag][id].unit_id << endl;

#endif // DEBUG
	return 0;
}

bool mymaintain(int id, BuildingType building_type, int age)
{
	if (sell_list[id]) return false;
	int cost = OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE] * (1 + float(age) * 2);
	int maxheal = OriginalBuildingAttribute[building_type][ORIGINAL_HEAL] * (1 + float(age) * 2);
	float percent = float(maxheal - state->building[ts19_flag][id].heal) / maxheal;
	if (percent > 0.2)percent = 0.2;
	if (myresource < cost*percent) return false;
	toggleMaintain(state->building[ts19_flag][id].unit_id);
	myresource -= ceil(cost*percent);
	++command_num;
	return true;
#ifdef DEBUG

	file << "turn: " << state->turn << " type:" << building_type << " maintain: " << state->building[ts19_flag][id].unit_id << "heal :" << state->building[ts19_flag][id].heal << endl;

#endif // DEBUG
}

bool getdis_for_savearea(int cnt)
{
	int head = 0, tail = 0;
	memset((void*)buildprogrammer_queueflag, 0, sizeof(bool)*MAP_SIZE*MAP_SIZE);
	for (int i = 0; i < saveArea[cnt].size(); ++i)
	{
		if (buildmap[saveArea[cnt][i].x][saveArea[cnt][i].y] == 2) continue;
		queue[tail++] = saveArea[cnt][i];
		buildprogrammer_queueflag[saveArea[cnt][i].x][saveArea[cnt][i].y] = true;
		savearea_everypoint_dis[cnt][saveArea[cnt][i].x][saveArea[cnt][i].y] = 0;
	}
	int tmpx, tmpy;
	while (head < tail)
	{
		for (int i = 0; i < 4; ++i)
		{
			tmpx = queue[head].x + tx4[i];
			tmpy = queue[head].y + ty4[i];
			if ((tmpx < 0) || (tmpx >= MAP_SIZE) || (tmpy < 0) || (tmpy >= MAP_SIZE))
				continue;
			if (buildprogrammer_queueflag[tmpx][tmpy]) continue;
			savearea_everypoint_dis[cnt][tmpx][tmpy] = savearea_everypoint_dis[cnt][queue[head].x][queue[head].y] + 1;
			queue[tail++] = Position(tmpx, tmpy);
			buildprogrammer_queueflag[tmpx][tmpy] = true;
		}
		++head;
	}
	savearea_everypoint_flag[cnt] = true;
	return true;
}

int build_programmer(int n)
{
	if (myresource < OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]) return 0;

	Position buildingPos;
	int rest_num = n;
	bool flag;
	int mindis,dis;

	while (rest_num > 0)
	{
		mindis = 10000;
		flag = false;
		for (int cnt = 0; cnt <= ROADCNT; ++cnt)
		{
			int realcnt = areasortid[cnt];
			for (int i = 0; i < saveArea[realcnt].size(); ++i)
			{
				if (buildmap[saveArea[realcnt][i].x][saveArea[realcnt][i].y] != 3) continue;
				dis = saveArea_enemydis[realcnt][i];
				if (dis < mindis)
				{
					mindis = dis;
					flag = true;
					buildingPos = saveArea[realcnt][i];
				}
			}
			if (!flag)
			{
				if (state->turn == 0) break;
				else continue;
			}
			if (!myConstruct(Programmer, buildingPos)) return 0;
			else ++my_resource_num;
			--rest_num;
			break;
		}
		if (!flag) break;
	}
	
	int cnt = 0,real_cnt=0;
	
	while (rest_num > 0)
	{
		if (real_cnt == ROADCNT + 1) break;
		cnt = areasortid[real_cnt];
		if (resource_area[cnt]) {++real_cnt; continue;}
		
		//if (areacount[areasortid[cnt]] == 0) continue;
		//BFS
		if (!savearea_everypoint_flag[cnt]) 
			if (!getdis_for_savearea(cnt)) { ++real_cnt; continue; }
		mindis = 10000;
		for (int x=0;x<MAP_SIZE;++x)
			for (int y=0;y<MAP_SIZE;++y)
				if (buildmap[x][y] == 3)
				{
					if (savearea_everypoint_dis[cnt][x][y] < mindis)
					{
						mindis = savearea_everypoint_dis[cnt][x][y];
						buildingPos = Position(x, y);
					}
				}
		if (!myConstruct(Programmer, buildingPos)) return 0;
		else ++my_resource_num;
		--rest_num;
		if (savearea_everypoint_dis[cnt][buildingPos.x][buildingPos.y] > MAX_BD_RANGE) continue;
		return rest_num;
		/*
		while (head < tail)
		{
			for (int i = 0; i < 4; ++i)
			{
				tmpx = queue[head].x + tx4[i];
				tmpy = queue[head].y + ty4[i];
				if ((tmpx < 0) || (tmpx >= MAP_SIZE) || (tmpy < 0) || (tmpy >= MAP_SIZE))
					continue;
				if (buildprogrammer_queueflag[tmpx][tmpy]) continue;
				if (buildmap[tmpx][tmpy] == 3)
				{
					buildingPos = Position(tmpx, tmpy);
					if (!myConstruct(Programmer, buildingPos)) return 0;
					else ++my_resource_num;
					--rest_num;
					return rest_num;
				}
				queue[tail++] = Position(tmpx, tmpy);
				buildprogrammer_queueflag[tmpx][tmpy] = true;
			}
			++head;
		}*/
		++real_cnt;
	}
	return 0;
}

int build_programmer_inbase(int n)
{
	if (myresource < OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]) return 0;

	Position buildingPos;
	int rest_num = n;
	bool flag;
	int y;
	while (rest_num > 0)
	{
		flag = false;
		for (int dis = 1; dis < MAP_SIZE<<1; ++dis)
		{
			for (int x = 0; x < dis + BASE_SIZE; ++x)
			{
				if (x >= MAP_SIZE) break;
				if (x < BASE_SIZE) y = BASE_SIZE;
				else y = 0;
				for (; y < MAP_SIZE; ++y)
				{
					if (my_base_dist(x, y) > dis) break;
					if (buildmap[x][y] != 3) continue;
					flag = true;
					buildingPos = Position(x, y);
					break;
				}
				if (flag) break;
			}
			if (!flag) continue;
			if (!myConstruct(Programmer, buildingPos)) return 0;
			else ++my_resource_num;
			--rest_num;
			break;
		}
		if (!flag) break;
	}
	return 0;
}

bool produce_setup(Position buildingpos)
{
	if (!setup_control) return false;
	if (state->age[ts19_flag] > CIRCUIT) return false;

	BuildingType building_type;
	if (state->age[ts19_flag] == CIRCUIT) building_type = Thevenin;
	else building_type = Shannon;

	//check whether need more produce
	bool findflag = false;
	//for (int i = 0; i <= ROADMAX; ++i)
	//	if (soldiernum_in_road[i][building_type] == 0) findflag = true;
	if (soldiernum_in_road[army_road][building_type] < 5) findflag = true;
	if (!findflag) return false;

	int dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	bool roadflag[ROADMAX + 1];
	for (int i = 0; i <= ROADMAX; ++i) roadflag[i] = false;
	int x = buildingpos.x, y = buildingpos.y;

	//find  road to put soldier
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (y - ty >= 0)
					if (ts19_map[x - tx][y - ty] == 1) roadflag[colormap[x - tx][y - ty]] = true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x - tx][y + ty] == 1) roadflag[colormap[x - tx][y + ty]] = true;
			}
			if (x + tx < MAP_SIZE)
			{
				if (y - ty >= 0)
					if (ts19_map[x + tx][y - ty] == 1) roadflag[colormap[x + tx][y - ty]] = true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x + tx][y + ty] == 1) roadflag[colormap[x + tx][y + ty]] = true;
			}
		}

	//delete road with soldier
	for (int i = 0; i <= ROADMAX; ++i)
		if (i != army_road)
		//if (soldiernum_in_road[i][building_type] > 0)
			roadflag[i] = false;

	//produce soldier
	for (int i = 0; i <= ROADMAX; ++i)
		if (roadflag[i])
		{
			Position soldier_pos;
			bool soldierflag = false;
			for (int tx = 0; tx <= dis; ++tx)
				if (!soldierflag)
				for (int ty = 0; tx + ty <= dis; ++ty)
				{
					if (x - tx >= 0)
					{
						if (y - ty >= 0)
							if (colormap[x - tx][y - ty] == i) 
							{
								soldier_pos = Position(x - tx, y - ty); 
								soldierflag	= true;
								break;
							}
						if (y + ty < MAP_SIZE)
							if (colormap[x - tx][y + ty] == i)
							{
								soldier_pos = Position(x - tx, y + ty);
								soldierflag = true;
								break;
							}
					}
					if (x + tx < MAP_SIZE)
					{
						if (y - ty >= 0)
							if (colormap[x + tx][y - ty] == i)
							{
								soldier_pos = Position(x = tx, y - ty);
								soldierflag = true;
								break;
							}
						if (y + ty < MAP_SIZE)
							if (colormap[x + tx][y + ty] == i)
							{
								soldier_pos = Position(x + tx, y + ty);
								soldierflag = true;
								break;
							}
					}
				}
			if (!soldierflag) return false;
			if (!myConstruct(building_type, buildingpos, soldier_pos)) return false;
			else
			{
				++soldiernum_in_road[i][building_type];
				return true;
			}
		}

	return false;
}

bool can_produce_soldier(Position pos)
{
	int dis = OriginalBuildingAttribute[Kuen_Kao][ORIGINAL_RANGE];
	int x = pos.x, y = pos.y;
	//find  road to put soldier
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (y - ty >= 0)
					if (ts19_map[x - tx][y - ty] == 1) 
						if (colormap[x-tx][y-ty]==army_road)
							return true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x - tx][y + ty] == 1)
						if (colormap[x - tx][y + ty] == army_road)
							return true;
			}
			if (x + tx < MAP_SIZE)
			{
				if (y - ty >= 0)
					if (ts19_map[x + tx][y - ty] == 1)
						if (colormap[x + tx][y - ty] == army_road)
							return true;
				if (y + ty < MAP_SIZE)
					if (ts19_map[x + tx][y + ty] == 1)
						if (colormap[x + tx][y + ty] == army_road)
							return true;
			}
		}
	return false;
}

int build_programmer_defense(int n)
{
	if (myresource < OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]) return 0;

	Position buildingPos;
	int rest_num = n;
	bool flag, produceflag = false;
	int y;
	while (rest_num > 0)
	{
		flag = false;
		for (int dis = 7; dis < MAP_SIZE; ++dis)
		{
			for (int x = 0; x <= dis; ++x)
			{
				if (x >= MAP_SIZE) break;
				y = dis - x;
				if (buildmap[x][y] != 3) continue;
				if (setup_control)
					if (mydist(Position(x, y), last_pos) <= MAX_BD_RANGE) continue;
				flag = true;
				buildingPos = Position(x, y);
				break;
			}
			if (!flag) continue;
			if (my_resource_num >= 10)
				produceflag = produce_setup(buildingPos);
			if (produceflag) continue;
			if (!myConstruct(Programmer, buildingPos)) return 0;
			else ++my_resource_num;
			--rest_num;
			break;
		}
		if (!flag) break;
	}
	return 0;
}

int upgrade_programmer(int n)
{
	int rest_num = n;
	bool flag;
	int maxdis, minenemydis, dis, enemydis,index;
	bool *updateflag = new bool[state->building[ts19_flag].size()];
	for (int i = 0; i < state->building[ts19_flag].size(); ++i) updateflag[i] = false;
	while (rest_num > 0)
	{
		maxdis = 0;
		minenemydis = 10000;
		flag = false;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (state->building[ts19_flag][i].building_type != Programmer) continue;
			if (state->building[ts19_flag][i].level >= state->age[ts19_flag]) continue;
			if (updateflag[i]) continue;
			flag = true;
			dis = mindismap[trans(state->building[ts19_flag][i].pos.x)][trans(state->building[ts19_flag][i].pos.y)];
			enemydis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
			if (dis > maxdis)
			{
				maxdis = dis;
				minenemydis = enemydis;
				index = i;
			}
			else if ((dis == maxdis) && (enemydis < minenemydis))
			{
				maxdis = dis;
				minenemydis = enemydis;
				index = i;
			}
		}
		if (!flag) break;
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type,index)) break;
		else --can_upgrade_reource_num;
		updateflag[index] = true;
		--rest_num;
	}
	return rest_num;
}

int build_produce(int n, BuildingType building_type)
{
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT]) return 0;

	int rest_num = n;
	int mindis, roaddis, dis;
	int build_dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	Position pos,soldier_pos;
	bool flag;
	while (rest_num > 0)
	{
		flag = false;
		mindis = 10000;
		for (int x = 0; x < MAP_SIZE; ++x)
			for (int y = 0; y < MAP_SIZE; ++y)
			{
				if (buildmap[x][y] != 3) continue;
				flag = true;
				dis = enemy_base_dist(x, y);
				if (dis < mindis)
				{
					mindis = dis;
					roaddis = mindismap[x][y];
					pos = Position(x, y);
					continue;
				}
				if (dis == mindis)
				{
					if (roaddis <= build_dis) continue;
					if ((mindismap[x][y] <= build_dis) ||
						(mindismap[x][y] > roaddis))
					{
						mindis = dis;
						roaddis = mindismap[x][y];
						pos = Position(x, y);
					}
				}
			}
		if (!flag) break;
		if (roaddis <= build_dis)
		{
			int x = pos.x, y = pos.y;
			flag = false;
			for (int tx = 0; tx <= build_dis; ++tx)
				for (int ty = 0; tx + ty <= build_dis; ++ty)
				{
					if (x - tx >= 0)
					{
						if (y - ty >= 0)
						{
							if (ts19_map[x - tx][y - ty] == 1) 
							{
								if (!flag)
								{
									soldier_pos = Position(x - tx, y - ty); flag = true;
								}
								else
								{
									if (enemy_base_dist(x-tx,y-ty)<enemy_base_dist(soldier_pos.x, soldier_pos.y)) 
										soldier_pos = Position(x - tx, y - ty);
								}
							}
						}
						if (y + ty < MAP_SIZE)
						{
							if (ts19_map[x - tx][y + ty] == 1)
							{
								if (!flag)
								{
									soldier_pos = Position(x - tx, y + ty); flag = true;
								}
								else
								{
									if (enemy_base_dist(x - tx, y + ty)<enemy_base_dist(soldier_pos.x, soldier_pos.y))
										soldier_pos = Position(x - tx, y + ty);
								}
							}
						}
					}
					if (x + tx < MAP_SIZE)
					{
						if (y - ty >= 0)
						{
							if (ts19_map[x + tx][y - ty] == 1)
							{
								if (!flag)
								{
									soldier_pos = Position(x + tx, y - ty); flag = true;
								}
								else
								{
									if (enemy_base_dist(x + tx, y - ty)<enemy_base_dist(soldier_pos.x, soldier_pos.y))
										soldier_pos = Position(x + tx, y - ty);
								}
							}
						}
						if (y + ty < MAP_SIZE)
						{
							if (ts19_map[x + tx][y + ty] == 1)
							{
								if (!flag)
								{
									soldier_pos = Position(x + tx, y + ty); flag = true;
								}
								else
								{
									if (enemy_base_dist(x + tx, y + ty)<enemy_base_dist(soldier_pos.x, soldier_pos.y))
										soldier_pos = Position(x + tx, y + ty);
								}
							}
						}
					}
				}
			if (!flag) break;
			if (!myConstruct(building_type, pos, soldier_pos)) break;
		}
		else
		{
			if (!myConstruct(Programmer,pos)) break;
			else ++my_resource_num;
		}
		--rest_num;
	}
	
	return 0;
}

int buildingScore(BuildingType building_type)
{
	if (building_type == Programmer) return 1;
	if (building_type == Shannon) return PRUDUCE_SCORE;
	if (building_type == Thevenin) return PRUDUCE_SCORE;
	if (building_type == Norton) return Norton_SCORE;
	if (building_type == Von_Neumann) return PRUDUCE_SCORE;
	if (building_type == Berners_Lee) return Berners_Lee_SCORE;
	if (building_type == Kuen_Kao) return Kuen_Kao_SCORE;
	if (building_type == Turing) return AI_SCORE;
	if (building_type == Tony_Stark) return AI_SCORE;

	return 0;
}

int build_produce_inbase(int n, BuildingType building_type)
{
	if (n <= 0) return 0;
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT]) return 0;

	int dis = OriginalSoldierAttribute[OriginalBuildingAttribute[building_type][TARGET]][ATTACK_RANGE];
	int* building_num = new int[ROADCNT + 1];
	bool* addflag = new bool[ROADCNT + 1];
	int x, y,color;
	for (int i = 0; i <= ROADCNT; ++i) *(building_num + i) = 0;

	//calculate the score of each road
	for (int i = 0; i < state->building[1 - ts19_flag].size(); ++i)
	{
		for (int j = 0; j <= ROADCNT; ++j) addflag[j] = false;
		x = trans(state->building[1 - ts19_flag][i].pos.x);
		y = trans(state->building[1 - ts19_flag][i].pos.y);
		for (int tx = 0; tx <= dis; ++tx)
			for (int ty = 0; tx + ty <= dis; ++ty)
			{
				if (x - tx >= 0)
				{
					if (y - ty >= 0)
					{
						if (ts19_map[x - tx][y - ty] == 1) color = colormap[x - tx][y - ty];
						else color = 100000;
						if (color<=ROADCNT)
							if (!addflag[color])
							{
								addflag[color] = true;
								building_num[color] += buildingScore(state->building[1 - ts19_flag][i].building_type);
							}
					}
					if (y + ty < MAP_SIZE)
					{
						if (ts19_map[x - tx][y + ty] == 1) color = colormap[x - tx][y + ty];
						else color = 100000;
						if (color <= ROADCNT)
							if (!addflag[color])
							{
								addflag[color] = true;
								building_num[color] += buildingScore(state->building[1 - ts19_flag][i].building_type);
							}
					}
				}
				if (x + tx < MAP_SIZE)
				{
					if (y - ty >= 0)
					{
						if (ts19_map[x + tx][y - ty] == 1) color = colormap[x + tx][y - ty];
						else color = 100000;
						if (color <= ROADCNT)
							if (!addflag[color])
							{
								addflag[color] = true;
								building_num[color] += buildingScore(state->building[1 - ts19_flag][i].building_type);
							}
					}
					if (y + ty < MAP_SIZE)
					{
						if (ts19_map[x + tx][y + ty] == 1) color = colormap[x + tx][y + ty];
						else color = 100000;
						if (color <= ROADCNT)
							if (!addflag[color])
							{
								addflag[color] = true;
								building_num[color] += buildingScore(state->building[1 - ts19_flag][i].building_type);
							}
					}
				}
			}
	}
	delete addflag;

	//get the road with highest score
	int road = 1, maxnum = *(building_num + 1);
	bool* roadflag = new bool[ROADCNT + 1];
	for (int i = 1; i <= ROADCNT; ++i) roadflag[i] = false;
	roadflag[0] = true;
	for (int i = 2; i <= ROADCNT; ++i)
		if (!roadflag[i])
		if (maxnum < *(building_num + i))
		{
			road = i;
			maxnum = *(building_num + i);
		}

	while (road > ROADCNT) --road;
	while (road < 1) ++road;
	if (roadflag[road]) return 0;
	roadflag[road] = true;
	//road = -1;

#ifdef DEBUG
	for (int i = 1; i <= ROADCNT; ++i)
		file << "road " << i << ": " << *(building_num + i) << endl;
	file << "road " << ": " << road << "  " << *(building_num + road) << endl;
#endif // DEBUG

	int rest_num = n;
	bool flag;
	Position pos,soldier_pos;
	dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	while (rest_num > 0)
	{
		flag = false;
		//tmpdis means the dis from base
		for (int tmpdis = 1; tmpdis < MAP_SIZE << 1; ++tmpdis)
		{
			for (x = 0; x < tmpdis + BASE_SIZE; ++x)
			{
				if (x >= MAP_SIZE) break;
				if (x < BASE_SIZE) y = BASE_SIZE;
				else y = 0;
				for (; y < MAP_SIZE; ++y)
				{
					if (my_base_dist(x, y) > tmpdis) break;
					if (buildmap[x][y] != 3) continue;
					for (int tx = 0; tx <= dis; ++tx)
						if (!flag)
							for (int ty = 0; tx + ty <= dis; ++ty)
							{
								if (x - tx >= 0)
								{
									if (y - ty >= 0)
									{
										if (ts19_map[x - tx][y - ty] == 1) color = colormap[x - tx][y - ty];
										else color = 100000;
										if ((color < 10000) && (road== -1)) { pos = Position(x, y); flag = true; break; }
										if (color == road) {  pos = Position(x, y); flag = true; break; }
									}
									if (y + ty < MAP_SIZE)
									{
										if (ts19_map[x - tx][y + ty] == 1) color = colormap[x - tx][y + ty];
										else color = 100000;
										if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
										if (color == road) {  pos = Position(x, y); flag = true; break; }
									}
								}
								if (x + tx < MAP_SIZE)
								{
									if (y - ty >= 0)
									{
										if (ts19_map[x + tx][y - ty] == 1) color = colormap[x + tx][y - ty];
										else color = 100000;
										if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
										if (color == road) {  pos = Position(x, y); flag = true; break; }
									}
									if (y + ty < MAP_SIZE)
									{
										if (ts19_map[x + tx][y + ty] == 1) color = colormap[x + tx][y + ty];
										else color = 100000;
										if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
										if (color == road) { pos = Position(x, y); flag = true; break; }
									}
								}
							}
					if (flag) break;
				}
				if (flag) break;
			}
			if (!flag) continue;
			flag = false;
			x = pos.x;
			y = pos.y;
			for (int tx = 0; tx <= dis; ++tx)
					for (int ty = 0; tx + ty <= dis; ++ty)
					{
						if (x - tx >= 0)
						{
							if (y - ty >= 0)
							{
								if (ts19_map[x - tx][y - ty] == 1) color = colormap[x - tx][y - ty];
								else color = 100000;
								if ((color == road) || ((color < 10000) && (road == -1)))
								{
									if (!flag) 
									{
										soldier_pos = Position(x - tx, y - ty); flag = true;
									}
									else
									{
										if ((x - tx <= soldier_pos.x) && (y - ty <= soldier_pos.y)) soldier_pos = Position(x - tx, y - ty);
									}
								}
							}
							if (y + ty < MAP_SIZE)
							{
								if (ts19_map[x - tx][y + ty] == 1) color = colormap[x - tx][y + ty];
								else color = 100000;
								if ((color == road) || ((color < 10000) && (road == -1)))
								{
									if (!flag)
									{
										soldier_pos = Position(x - tx, y + ty); flag = true;
									}
									else
									{
										if ((x - tx <= soldier_pos.x) && (y + ty <= soldier_pos.y)) soldier_pos = Position(x - tx, y + ty);
									}
								}
							}
						}
						if (x + tx < MAP_SIZE)
						{
							if (y - ty >= 0)
							{
								if (ts19_map[x + tx][y - ty] == 1) color = colormap[x + tx][y - ty];
								else color = 100000;
								if ((color == road) || ((color < 10000) && (road == -1)))
								{
									if (!flag)
									{
										soldier_pos = Position(x + tx, y - ty); flag = true;
									}
									else
									{
										if ((x + tx <= soldier_pos.x) && (y - ty <= soldier_pos.y)) soldier_pos = Position(x + tx, y - ty);
									}
								}
							}
							if (y + ty < MAP_SIZE)
							{
								if (ts19_map[x + tx][y + ty] == 1) color = colormap[x + tx][y + ty];
								else color = 100000;
								if ((color == road) || ((color < 10000) && (road == -1)))
								{
									if (!flag)
									{
										soldier_pos = Position(x + tx, y + ty); flag = true;
									}
									else
									{
										if ((x + tx <= soldier_pos.x) && (y + ty <= soldier_pos.y)) soldier_pos = Position(x + tx, y + ty);
									}
								}
							}
						}
					}
			if (!flag) continue;
			if (!myConstruct(building_type, pos, soldier_pos)) 
			{
				delete roadflag;
				delete building_num; 
				return 0; 
			}
			--rest_num;
			break;
		}
		//fine new road with highest score
		if (!flag)
		{
			maxnum = 0;
			for (int i = 1; i <= ROADCNT; ++i)
				if (!roadflag[i])
				{
					flag = true;
					if (maxnum < *(building_num + i))
					{
						road = i;
						maxnum = *(building_num + i);
					}
				}
			roadflag[road] = true;
		}
		if (!flag) break;
	}

	delete roadflag;
	delete building_num;
	return 0;
}

int build_produce_defense(int n, BuildingType building_type,int road=1)
{
	if (n <= 0) return 0;
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT]) return 0;

	int dis = OriginalSoldierAttribute[OriginalBuildingAttribute[building_type][TARGET]][ATTACK_RANGE];
	int x, y, color;
	//int road = 1;

	int rest_num = n;
	bool flag;
	Position pos, soldier_pos;
	dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	while (rest_num > 0)
	{
		flag = false;
		//tmpdis means the dis from base
		for (int tmpdis = 1; tmpdis < MAP_SIZE << 1; ++tmpdis)
		{
			for (x = 0; x < tmpdis; ++x)
			{
				if (x >= MAP_SIZE) break;
				y = tmpdis - x;
				if (y >= MAP_SIZE) continue;
				if (buildmap[x][y] != 3) continue;
				for (int tx = 0; tx <= dis; ++tx)
					if (!flag)
						for (int ty = 0; tx + ty <= dis; ++ty)
						{
							if (x - tx >= 0)
							{
								if (y - ty >= 0)
								{
									if (ts19_map[x - tx][y - ty] == 1) color = colormap[x - tx][y - ty];
									else color = 100000;
									if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
									if (color == road) { pos = Position(x, y); flag = true; break; }
								}
								if (y + ty < MAP_SIZE)
								{
									if (ts19_map[x - tx][y + ty] == 1) color = colormap[x - tx][y + ty];
									else color = 100000;
									if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
									if (color == road) { pos = Position(x, y); flag = true; break; }
								}
							}
							if (x + tx < MAP_SIZE)
							{
								if (y - ty >= 0)
								{
									if (ts19_map[x + tx][y - ty] == 1) color = colormap[x + tx][y - ty];
									else color = 100000;
									if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
									if (color == road) { pos = Position(x, y); flag = true; break; }
								}
								if (y + ty < MAP_SIZE)
								{
									if (ts19_map[x + tx][y + ty] == 1) color = colormap[x + tx][y + ty];
									else color = 100000;
									if ((color < 10000) && (road == -1)) { pos = Position(x, y); flag = true; break; }
									if (color == road) { pos = Position(x, y); flag = true; break; }
								}
							}
						}
				if (flag) break;
			}
			if (!flag) continue;
			flag = false;
			x = pos.x;
			y = pos.y;
			for (int tx = 0; tx <= dis; ++tx)
				for (int ty = 0; tx + ty <= dis; ++ty)
				{
					if (x - tx >= 0)
					{
						if (y - ty >= 0)
						{
							if (ts19_map[x - tx][y - ty] == 1) color = colormap[x - tx][y - ty];
							else color = 100000;
							if ((color == road) || ((color < 10000) && (road == -1)))
							{
								if (!flag)
								{
									soldier_pos = Position(x - tx, y - ty); flag = true;
								}
								else
								{
									if ((x - tx >= soldier_pos.x) && (y - ty >= soldier_pos.y)) soldier_pos = Position(x - tx, y - ty);
								}
							}
						}
						if (y + ty < MAP_SIZE)
						{
							if (ts19_map[x - tx][y + ty] == 1) color = colormap[x - tx][y + ty];
							else color = 100000;
							if ((color == road) || ((color < 10000) && (road == -1)))
							{
								if (!flag)
								{
									soldier_pos = Position(x - tx, y + ty); flag = true;
								}
								else
								{
									if ((x - tx >= soldier_pos.x) && (y + ty >= soldier_pos.y)) soldier_pos = Position(x - tx, y + ty);
								}
							}
						}
					}
					if (x + tx < MAP_SIZE)
					{
						if (y - ty >= 0)
						{
							if (ts19_map[x + tx][y - ty] == 1) color = colormap[x + tx][y - ty];
							else color = 100000;
							if ((color == road) || ((color < 10000) && (road == -1)))
							{
								if (!flag)
								{
									soldier_pos = Position(x + tx, y - ty); flag = true;
								}
								else
								{
									if ((x + tx >= soldier_pos.x) && (y - ty >= soldier_pos.y)) soldier_pos = Position(x + tx, y - ty);
								}
							}
						}
						if (y + ty < MAP_SIZE)
						{
							if (ts19_map[x + tx][y + ty] == 1) color = colormap[x + tx][y + ty];
							else color = 100000;
							if ((color == road) || ((color < 10000) && (road == -1)))
							{
								if (!flag)
								{
									soldier_pos = Position(x + tx, y + ty); flag = true;
								}
								else
								{
									if ((x + tx >= soldier_pos.x) && (y + ty >= soldier_pos.y)) soldier_pos = Position(x + tx, y + ty);
								}
							}
						}
					}
				}
			if (!flag) continue;
			if (!myConstruct(building_type, pos, soldier_pos)) return 0;
			--rest_num;
			break;
		}
		//fine new road with highest score
		
		if (!flag) break;
	}
	return 0;
}

int build_defense_inbase(int road, BuildingType building_type)
{
	if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag])
	{
		sell_for_defense = true;
		return 0;
	}
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT]) return 0;

	bool flag;
	int x, y;
	int dis = int(OriginalBuildingAttribute[building_type][ORIGINAL_RANGE] * 0.5);
	Position pos;
	int maxscore, nowdis;
	int disgap = 4;
	int color, score;
	bool* roadflag = new bool[ROADCNT + 1];
	while (needfiremap_inbase[road][building_type] > firemap_inbase[road][building_type])
	{
		if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag])
		{
			sell_for_defense = true;
			return 0;
		}
		flag = false;
		maxscore = FIX_ROAD_SCORE - 1;
		nowdis = 10000;
		//tmpdis means the dis from base
		for (int tmpdis = 1; tmpdis < MAP_SIZE << 1; ++tmpdis)
		{
			if (tmpdis > nowdis + disgap) break;
			for (x = 0; x < tmpdis + BASE_SIZE; ++x)
			{
				if (x >= MAP_SIZE) break;
				if (x < BASE_SIZE) y = BASE_SIZE;
				else y = 0;
				for (; y < MAP_SIZE; ++y)
					if (my_base_dist(x, y) >= tmpdis) break;
				for (; y < MAP_SIZE; ++y)
				{
					if (my_base_dist(x, y) < tmpdis) continue;
					if (my_base_dist(x, y) > tmpdis) break;
					if (buildmap[x][y] != 3) continue;
					score = 0;
					for (int i = 1; i <= ROADCNT; ++i) roadflag[i] = false;
					for (int tx = 0; tx <= dis; ++tx)
							for (int ty = 0; tx + ty <= dis; ++ty)
							{
								if (x - tx >= 0)
								{
									if (y - ty >= 0)
									{
										if (ts19_map[x - tx][y - ty] == 1)
										{
											color = colormap[x - tx][y - ty];
											if (!roadflag[color])
											{
												roadflag[color] = true;
												if (color == road) score += FIX_ROAD_SCORE;
												else if (needfiremap_inbase[color][building_type] > 0)
												{
													if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
													else score += FULL_ROAD_SCORE;
												}
											}
										}
									}
									if (y + ty < MAP_SIZE)
									{
										if (ts19_map[x - tx][y + ty] == 1)
										{
											color = colormap[x - tx][y + ty];
											if (!roadflag[color])
											{
												roadflag[color] = true;
												if (color == road) score += FIX_ROAD_SCORE;
												else if (needfiremap_inbase[color][building_type] > 0)
												{
													if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
													else score += FULL_ROAD_SCORE;
												}
											}
										}
									}
								}
								if (x + tx < MAP_SIZE)
								{
									if (y - ty >= 0)
									{
										if (ts19_map[x + tx][y - ty] == 1)
										{
											color = colormap[x + tx][y - ty];
											if (!roadflag[color])
											{
												roadflag[color] = true;
												if (color == road) score += FIX_ROAD_SCORE;
												else if (needfiremap_inbase[color][building_type] > 0)
												{
													if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
													else score += FULL_ROAD_SCORE;
												}
											}
										}
									}
									if (y + ty < MAP_SIZE)
									{
										if (ts19_map[x + tx][y + ty] == 1)
										{
											color = colormap[x + tx][y + ty];
											if (!roadflag[color])
											{
												roadflag[color] = true;
												if (color == road) score += FIX_ROAD_SCORE;
												else if (needfiremap_inbase[color][building_type] > 0)
												{
													if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
													else score += FULL_ROAD_SCORE;
												}
											}
										}
									}
								}
							}
					if (score > maxscore)
					{
						maxscore = score;
						pos = Position(x, y);
						nowdis = my_base_dist(x, y);
						flag = true;
					}
				}
			}	
		}

		if (!flag) 
		for (int tmpdis = 1; tmpdis <  building_base_dis[building_type]; ++tmpdis)
		{
			if (tmpdis > nowdis + disgap) break;
			for (x = 0; x < tmpdis + BASE_SIZE; ++x)
			{
				if (x >= MAP_SIZE) break;
				if (x < BASE_SIZE) y = BASE_SIZE;
				else y = 0;
				for (; y < MAP_SIZE; ++y)
				{
					if (my_base_dist(x, y) > tmpdis) break;
					if (buildmap[x][y] != 3) continue;
					score = 0;
					for (int i = 1; i <= ROADCNT; ++i) roadflag[i] = false;
					for (int tx = 0; tx <= dis; ++tx)
						for (int ty = 0; tx + ty <= dis; ++ty)
						{
							if (x - tx >= 0)
							{
								if (y - ty >= 0)
								{
									if (ts19_map[x - tx][y - ty] == 1)
									{
										color = colormap[x - tx][y - ty];
										if (!roadflag[color])
										{
											roadflag[color] = true;
											if (color == road) score += FIX_ROAD_SCORE;
											else if (needfiremap_inbase[color][building_type] > 0)
											{
												if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
												else score += FULL_ROAD_SCORE;
											}
										}
									}
								}
								if (y + ty < MAP_SIZE)
								{
									if (ts19_map[x - tx][y + ty] == 1)
									{
										color = colormap[x - tx][y + ty];
										if (!roadflag[color])
										{
											roadflag[color] = true;
											if (color == road) score += FIX_ROAD_SCORE;
											else if (needfiremap_inbase[color][building_type] > 0)
											{
												if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
												else score += FULL_ROAD_SCORE;
											}
										}
									}
								}
							}
							if (x + tx < MAP_SIZE)
							{
								if (y - ty >= 0)
								{
									if (ts19_map[x + tx][y - ty] == 1)
									{
										color = colormap[x + tx][y - ty];
										if (!roadflag[color])
										{
											roadflag[color] = true;
											if (color == road) score += FIX_ROAD_SCORE;
											else if (needfiremap_inbase[color][building_type] > 0)
											{
												if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
												else score += FULL_ROAD_SCORE;
											}
										}
									}
								}
								if (y + ty < MAP_SIZE)
								{
									if (ts19_map[x + tx][y + ty] == 1)
									{
										color = colormap[x + tx][y + ty];
										if (!roadflag[color])
										{
											roadflag[color] = true;
											if (color == road) score += FIX_ROAD_SCORE;
											else if (needfiremap_inbase[color][building_type] > 0)
											{
												if (needfiremap_inbase[color][building_type] > firemap_inbase[color][building_type]) score += NEED_ROAD_SCORE;
												else score += FULL_ROAD_SCORE;
											}
										}
									}
								}
							}
						}
					if (score > maxscore)
					{
						maxscore = score;
						pos = Position(x, y);
						nowdis = my_base_dist(x, y);
						flag = true;
					}
				}
			}
		}

		if (!flag) { delete roadflag; return -1; }
#ifdef DEBUG
		file << "defense_score " << ": " << maxscore << "  type:" << building_type << "  (" << pos.x << "," << pos.y << ")" << endl;
#endif // DEBUG
		if (!myConstruct(building_type, pos)) { delete roadflag; return -1; }
	}
	
	delete roadflag;
	return 0;
}

bool check_kernel_withbuilding(Position pos)
{
	int dis = 4 * MIN_BD_RANGE;
	int x = pos.x, y = pos.y;
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (ty >= tx)
					if (y + ty < MAP_SIZE)
						if (buildmap[x - tx][y + ty] <= 2) return true;
			}
			if (x + tx < MAP_SIZE)
			{
				if (tx >= ty)
					if (y - ty >= 0)
						if (buildmap[x + tx][y - ty] <= 2) return true;
				if (y + ty < MAP_SIZE)
					if (buildmap[x + tx][y + ty] <= 2) return true;
			}
		}
	return false;
}

bool check_kernel(Position pos)
{
	int dis = MIN_BD_RANGE;
	int x = pos.x, y = pos.y;
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (ty>=tx)
				if (y + ty < MAP_SIZE)
					if (buildmap[x - tx][y + ty] == 3) return true;
			}
			if (x + tx < MAP_SIZE)
			{
				if (tx>=ty)
				if (y - ty >= 0)
					if (buildmap[x + tx][y - ty] == 3) return true;
				if (y + ty < MAP_SIZE)
					if (buildmap[x + tx][y + ty] == 3) return true;
			}
		}
	return false;
}

bool get_kernel(int road)
{
	bool have_kernel = false;
	if ((kernel_defense[road].x >= 0) && (kernel_defense[road].y >= 0))
	{
		if ((state->age[ts19_flag] < NETWORK) ||
			((state->age[ts19_flag] >= NETWORK) && (last_age >= NETWORK)))
			have_kernel = true;
		if ((state->age[ts19_flag] == AI) && (last_age < AI))
			if (road == army_road)
				have_kernel = false;
	}
	if (have_kernel)
		if (check_kernel_withbuilding(kernel_defense[road])) return true;
	last_age = state->age[ts19_flag];
	Position root = roadroot[road];
	int cnt = 0;
	while (++cnt <= MAP_SIZE * 0.75) root = nextpos[root.x][root.y];
	while (!check_kernel(root))
	{
		if (cnt <= 0) return false;
		--cnt;
		if (root.x > 0)
		{
			if (ts19_map[root.x - 1][root.y] == 1)
				root = Position(root.x - 1, root.y);
			else root = Position(root.x, root.y - 1);
		}
		else root = Position(root.x, root.y - 1);
	}
	kernel_defense[road] = root;
	cnt = 0;
	int defense_dist = 0;
	if (state->age[ts19_flag] < NETWORK) defense_dist = OriginalBuildingAttribute[Ohm][ORIGINAL_RANGE];
	else defense_dist = OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_RANGE];
	while (++cnt <= defense_dist) root = nextpos[root.x][root.y];
	kernel_target[road] = root;
	if (state->age[ts19_flag] < NETWORK) 
		defense_gap = OriginalBuildingAttribute[Ohm][ORIGINAL_RANGE] - OriginalSoldierAttribute[VOLTAGE_SOURCE][ATTACK_RANGE];
	else 
		defense_gap = OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_RANGE] - OriginalSoldierAttribute[OPTICAL_FIBER][ATTACK_RANGE];
	return true;
}

int build_defense_road(int road, BuildingType building_type)
{
#ifdef DEBUG
	file << "need defense: " << road << "  " << building_type << endl;
#endif // DEBUG
	if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag])
	{
		sell_for_defense = true;
		return 0;
	}

	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE]) return 0;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT]) return 0;

	bool flag;
	int x, y;
	Position pos;
	int minbuilddis, maxbuilddis;
	while (needfiremap_inbase[road][building_type] > soldiernum_in_road[road][building_type] * MAX_FIRE)
	{
		if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag])
		{
			sell_for_defense = true;
			return 0;
		}
		if (!get_kernel(road)) return 0;
		minbuilddis = kernel_defense[road].x + kernel_defense[road].y;
		maxbuilddis = kernel_defense[road].x + kernel_defense[road].y + defense_gap;
		if (building_type == Ohm) maxbuilddis += defense_gap;
		if ((building_type == Musk) || (building_type == Hawkin))
			if (minbuilddis + MIN_BD_RANGE <= maxbuilddis) minbuilddis += MIN_BD_RANGE;
		flag = false;
		//tmpdis means the dis from base
		if (building_type == Mole) defense_gap <<= 1;
		for (int tmpdis = minbuilddis; tmpdis <= maxbuilddis; ++tmpdis)
		{
			if (flag)
				if ((building_type != Musk) && (building_type != Hawkin))
					break;
			for (x = 0; x <= kernel_target[road].x + (defense_gap >> 1); ++x)
			{
				if (x >= MAP_SIZE) break;
				y = tmpdis - x;
				if (y < 0) break;
				if (y >= MAP_SIZE) continue;
				if (y > kernel_target[road].y + (defense_gap >> 1)) continue;
				if (buildmap[x][y] != 3) continue;
				if (flag)
				{
					if (mydist(Position(x, y), kernel_defense[road]) < mydist(pos, kernel_defense[road]))
						pos = Position(x, y);
				}
				else pos = Position(x, y);
				flag = true;
				if ((building_type == Musk) || (building_type == Hawkin)) continue;
				break;
			}
		}
		if (building_type == Mole) defense_gap >>= 1;
		if (!flag)
		{
			if ((building_type == Musk) || (building_type == Hawkin))
			{
				maxbuilddis = minbuilddis - 1;
				minbuilddis -= MIN_BD_RANGE;
				for (int tmpdis = minbuilddis; tmpdis <= maxbuilddis; ++tmpdis)
				{
					for (x = 0; x <= kernel_target[road].x; ++x)
					{
						y = tmpdis - x;
						if (y < 0) break;
						if (y > kernel_target[road].y) continue;
						if (buildmap[x][y] != 3) continue;
						if (flag)
						{
							if (mydist(Position(x, y), kernel_defense[road]) < mydist(pos, kernel_defense[road]))
								pos = Position(x, y);
						}
						else pos = Position(x, y);
						flag = true;
					}
				}
			}
			if (!flag) return -1;
		}
			
		if (!myConstruct(building_type, pos)) return -1;
		soldiernum_in_road[road][building_type] += 1;
		defense_road[pos.x][pos.y] = road;
	}

	return 0;
}

int build_defense_outside(Position pos, BuildingType building_type)
{
	int dis = OriginalBuildingAttribute[building_type][ORIGINAL_RANGE];
	if (my_bd_num >= MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag]) return 0;
	//LOGN("TRY DEFENSE");
	int x = pos.x, y = pos.y;
	for (int tx = 0; tx <= dis; ++tx)
		for (int ty = 0; tx + ty <= dis; ++ty)
		{
			if (x - tx >= 0)
			{
				if (y - ty >= 0)
				{
					if (buildmap[x - tx][y - ty] == 3)
					{
						if (!myConstruct(building_type, Position(x - tx, y - ty))) return 0;
						else { drawfiremap_outside(Position(x - tx, y - ty), building_type); return 1; };
					}
				}
				if (y + ty < MAP_SIZE)
				{
					if (buildmap[x - tx][y + ty] == 3)
					{
						if (!myConstruct(building_type, Position(x - tx, y + ty))) return 0;
						else { drawfiremap_outside(Position(x - tx, y + ty), building_type); return 1; };
					}
				}
			}
			if (x + tx < MAP_SIZE)
			{
				if (y - ty >= 0)
				{
					if (buildmap[x + tx][y - ty] == 3)
					{
						if (!myConstruct(building_type, Position(x + tx, y - ty))) return 0;
						else { drawfiremap_outside(Position(x + tx, y - ty), building_type); return 1; };
					}
				}
				if (y + ty < MAP_SIZE)
				{
					if (buildmap[x + tx][y + ty] == 3)
					{
						if (!myConstruct(building_type, Position(x + tx, y + ty))) return 0;
						else { drawfiremap_outside(Position(x + tx, y + ty), building_type); return 1; };
					}
				}
			}
		}
	return 1;
}

int upgrade_attack(int n)
{
	int rest_num = n;
	int mindis;
	BuildingType level;
	bool flag;
	int index;
	bool updateflag[200] = { false };
	while (rest_num > 0)
	{
		flag = false;
		mindis = 10000;
		level = __Base;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (updateflag[i]) continue;
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] != PRODUCTION_BUILDING)
			{
				updateflag[i] = true; continue;
			}
			if (state->building[ts19_flag][i].level >= state->age[ts19_flag])
			{
				updateflag[i] = true; continue;
			}
			if ((myresource < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_RESOURCE] * 0.5) ||
				(mybdpoint < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_BUILDING_POINT] * 0.5))
			{
				updateflag[i] = true; continue;
			};
			flag = true;
			if (state->building[ts19_flag][i].building_type > level)
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level)&&
				(state->building[ts19_flag][i].heal>state->building[ts19_flag][index].heal))
			//	(mindis > enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level) &&
				(state->building[ts19_flag][i].heal == state->building[ts19_flag][index].heal)&&
				(mindis > enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
		}
		if (!flag) break;
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type,index)) break;
		updateflag[index] = true;
		--rest_num;
	}
	return 0;
}

int upgrade_attack_inbase(int n)
{
	int rest_num = n;
	int mindis;
	BuildingType level;
	bool flag;
	int index;
	bool updateflag[200] = { false };
	while (rest_num > 0)
	{
		flag = false;
		mindis = 10000;
		level = __Base;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (updateflag[i]) continue;
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] != PRODUCTION_BUILDING)
			{
				updateflag[i] = true; continue;
			}
			if (state->building[ts19_flag][i].level >= state->age[ts19_flag])
			{
				updateflag[i] = true; continue;
			}
			if ((myresource < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_RESOURCE] * 0.5) ||
				(mybdpoint < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_BUILDING_POINT] * 0.5))
			{
				updateflag[i] = true; continue;
			};
			flag = true;
			if (state->building[ts19_flag][i].building_type > level)
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = my_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level) &&
				(state->building[ts19_flag][i].heal>state->building[ts19_flag][index].heal))
				//	(mindis > my_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = my_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level) &&
				(state->building[ts19_flag][i].heal == state->building[ts19_flag][index].heal) &&
				(mindis > my_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = my_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
		}
		if (!flag) break;
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type,index)) break;
		updateflag[index] = true;
		--rest_num;
	}
	return 0;
}

int upgrade_defense(int n)
{
	int rest_num = n;
	int mindis;
	BuildingType level;
	bool flag;
	int index;
	bool updateflag[200] = { false };
	while (rest_num > 0)
	{
		flag = false;
		mindis = 10000;
		level = __Base;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (updateflag[i]) continue;
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] != DEFENSIVE_BUILDING)
			{
				updateflag[i] = true; continue;
			}
			if (state->building[ts19_flag][i].level >= state->age[ts19_flag])
			{
				updateflag[i] = true; continue;
			}
			if ((myresource < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_RESOURCE] * 0.5) ||
				(mybdpoint < OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_BUILDING_POINT] * 0.5))
			{
				updateflag[i] = true; continue;
			};
			flag = true;
			if ((state->building[ts19_flag][i].building_type > level) || 
				((state->building[ts19_flag][i].building_type==Musk)&&(level!=Musk)))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level) &&
				(state->building[ts19_flag][i].heal>state->building[ts19_flag][index].heal))
				//	(mindis > enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
			if ((state->building[ts19_flag][i].building_type == level) &&
				(state->building[ts19_flag][i].heal == state->building[ts19_flag][index].heal) &&
				(mindis > enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
			{
				index = i;
				level = state->building[ts19_flag][i].building_type;
				mindis = enemy_base_dist(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y));
				continue;
			}
		}
		if (!flag) break;
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type,index)) break;
		updateflag[index] = true;
		--rest_num;
	}
	return 0;
}

int resource_produce_strategy()
{
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, resource_num - my_resource_num);
	while (bd_num > 0)
		bd_num = build_programmer(bd_num);
	return 0;
}

int resource_produce_inbase_strategy()
{
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, resource_num - my_resource_num);
	while (bd_num > 0)
		bd_num = build_programmer_inbase(bd_num);
	return 0;
}

int resource_produce_defense_strategy()
{
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, resource_num - my_resource_num);
	while (bd_num > 0)
		bd_num = build_programmer_defense(bd_num);
	return 0;
}

int resource_update_strategy()
{
	int bd_num = mymin(floor(2 * myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(2 * mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, can_upgrade_reource_num);
	upgrade_programmer(bd_num);
	return 0;
}

int attack_produce_strategy()
{
	if (my_resource_num < resource_num) return 0;
	//if (can_upgrade_reource_num > 0) return 0;
	int bool_count = 0, ohm_count = 0, mole_count = 0, mc_count = 0, larry_count = 0, robert_count = 0, musk_count = 0, hawkin_count = 0;
	Position tmppos;
	for (int i = 0; i < state->building[1 - ts19_flag].size(); ++i)
	{
		tmppos = state->building[1 - ts19_flag][i].pos;
		if (enemy_base_dist(trans(tmppos.x), trans(tmppos.y)) >= (MAP_SIZE >> 1)) continue;
		if (state->building[1 - ts19_flag][i].building_type == Bool) ++bool_count;
		else if (state->building[1 - ts19_flag][i].building_type == Ohm) ++ohm_count;
		else if (state->building[1 - ts19_flag][i].building_type == Mole) ++mole_count;
		else if (state->building[1 - ts19_flag][i].building_type == Monte_Carlo) ++mc_count;
		else if (state->building[1 - ts19_flag][i].building_type == Larry_Roberts) ++larry_count;
		else if (state->building[1 - ts19_flag][i].building_type == Robert_Kahn) ++robert_count;
		else if (state->building[1 - ts19_flag][i].building_type == Musk) ++musk_count;
		else if (state->building[1 - ts19_flag][i].building_type == Hawkin) ++hawkin_count;
	}

	//Construct produce building
	int bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] + 1 - state->building[ts19_flag].size();
	BuildingType building_type;
	if (ts19_flag <= 2)
	{
		building_type = Shannon;
		if (state->age[ts19_flag] > BIT)
			building_type = Norton;
		if (ohm_count >= 10) building_type = Shannon;
		if (state->age[ts19_flag] == NETWORK)
		{
			if (larry_count >= 6)
			{
				if (ohm_count < 8 ) building_type = Norton;
				else building_type = Shannon;
			}
			else building_type = Berners_Lee;
		}
		if (state->age[ts19_flag] == AI)
		{
			building_type = Tony_Stark;
			if (musk_count>0)
			{
				//if (ohm_count > 0) building_type = Shannon;
				//else
				//{
				//	if (mc_count>bool_count + robert_count)
				//		building_type = Shannon;
				//	else building_type = Kuen_Kao;
				//}
				building_type = Shannon;
			}
		}
	}
	else
	{
		building_type = Shannon;
		if (bool_count > 0)	building_type = Thevenin;
		if (ohm_count > 0) building_type = Shannon;
		if (state->age[ts19_flag] == PROCESSOR)
		{
			if ((ohm_count > 0) && (mole_count > 0)) building_type = Shannon;
			else if (ohm_count > 0)
			{
				if (bool_count > 0) building_type = Von_Neumann;
				else building_type = Shannon;
			}
			else if (mole_count > 0)
			{
				if (bool_count<mole_count) building_type = Shannon;
				else building_type = Thevenin;
			}
		}
		if (state->age[ts19_flag] == NETWORK)
		{
			if (larry_count >= 3)
			{
				if (ohm_count > 0) building_type = Shannon;
				else
				{
					if (mc_count>bool_count+robert_count)
						building_type = Shannon;
					else building_type = Kuen_Kao;
				}
			}
			else building_type = Berners_Lee;
		}
		if (state->age[ts19_flag] == AI)
		{
			building_type = Tony_Stark;
			if (musk_count>0)
			{
				if (ohm_count > 0) building_type = Shannon;
				else
				{
					if (mc_count>bool_count + robert_count)
						building_type = Shannon;
					else building_type = Kuen_Kao;
				}
			}
		}
	}

	if (state->age[ts19_flag] >= PROCESSOR)
	{
		int von_num = (int(state->age[ts19_flag]) - 1) - my_building_num[Von_Neumann];
		while (von_num > 0)
			von_num= build_produce(von_num, Von_Neumann);
	}

	while (bd_num > 0)
		bd_num = build_produce(bd_num, building_type);
	return 0;
}

BuildingType destroy_choice()
{
	BuildingType building_type;
	building_type = Shannon;
	if (my_building_num[Shannon] < enemy_building_num[Bool]) building_type = Thevenin;
	if (enemy_building_num[Ohm] >= 9) building_type = Shannon;
	if (state->age[ts19_flag] >= NETWORK)
		if (my_building_num[Shannon] < enemy_building_num[Bool])
			building_type = Kuen_Kao;
	//if (state->age[ts19_flag] == AI)
	//	if (enemy_building_num[Musk] == 0) building_type = Tony_Stark;
	return building_type;
}

int attack_produce_army_strategy()
{
	if (my_resource_num < resource_num) return 0;
	//if (can_upgrade_reource_num > 0) return 0;
	if (state->age[ts19_flag]==BIT) return -1;
	setup_control = false;

	//Construct produce building
	int bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] - my_bd_num;
	if ((bd_num <= 0) && (state->age[ts19_flag] == AI)) { sell_for_produce = true; return 0; }
	BuildingType building_type = destroy_choice();
	if (soldiernum_in_road[1][Von_Neumann] < 3) 
		build_produce_defense(mymin(3- soldiernum_in_road[1][Von_Neumann],bd_num), Von_Neumann,1);
	bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] - my_bd_num;
	//if (soldiernum_in_road[ROADCNT][Von_Neumann] < 3) 
	//	build_produce_defense(mymin(3 - soldiernum_in_road[ROADCNT][Von_Neumann], bd_num), Von_Neumann, ROADCNT);
	bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] - my_bd_num;
	while (bd_num > 0)
		if (soldiernum_in_road[1][building_type]<=soldiernum_in_road[ROADCNT][building_type])
			bd_num = build_produce_defense(bd_num, building_type,1);
		else
			bd_num = build_produce_defense(bd_num, building_type, 1);

	return 0;
}

int attack_produce_inbase_strategy()
{
	if (my_resource_num < resource_num) return 0;
	//if (can_upgrade_reource_num > 0) return 0;
	if (state->age[ts19_flag] < PROCESSOR)
		if (enemy_procude_num == 0)
			return -1;

	//Construct produce building
	if (nearest_enemybuilding_dis < (MAP_SIZE >> 1))
	{
		int bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] - my_bd_num;
		if ((bd_num <= 0) && (state->age[ts19_flag] == AI)) { sell_for_produce = true; return 0; }
		BuildingType building_type = destroy_choice();
		while (bd_num > 0)
			bd_num = build_produce_inbase(bd_num, building_type);
	}
	else
	{
		if (state->age[ts19_flag] < PROCESSOR)
		{
			if (state->age[ts19_flag] == CIRCUIT)
				if (soldiernum_in_road[ROADCNT][Thevenin]==0)
					build_produce_defense(1, Thevenin, army_road);
			if (soldiernum_in_road[ROADCNT][Shannon] == 0)
				build_produce_defense(1, Shannon, army_road);
			return 0;
		}
		int bd_num = 3 - soldiernum_in_road[1][Von_Neumann];
		while (bd_num > 0)
			if (MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] <= my_bd_num) { sell_for_produce = true; return 0;}
			else bd_num = build_produce_defense(bd_num, Von_Neumann, army_road);
		bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] - my_bd_num;
		if (state->age[ts19_flag] >= AI) 
			if (soldiernum_in_road[1][Tony_Stark] < 1)
				build_produce_defense(bd_num, Tony_Stark, army_road);
		if (state->age[ts19_flag] >= NETWORK)
			while (bd_num > 0)
				bd_num = build_produce_defense(bd_num, Kuen_Kao, army_road);
				//bd_num = build_produce_defense(bd_num, Kuen_Kao,(state->turn%ROADCNT)+1);
		else
		{
			if (soldiernum_in_road[ROADCNT][Shannon] == 0)
				build_produce_defense(1, Shannon,army_road);
		}
				
	}

	return 0;
}

int attack_update_strategy()
{
	//Upgrade produce building
	int bd_num = can_upgrade_produce_num;
	while (bd_num > 0)
		bd_num = upgrade_attack(bd_num);
	return 0;
}

bool defense_order(SoldierName x, SoldierName y)
{
	return OriginalSoldierAttribute[x][SOLDIER_ORIGINAL_ATTACK] > OriginalSoldierAttribute[y][SOLDIER_ORIGINAL_ATTACK];
}

int defense_produce_strategy()
{
	//if (my_produce_num + (MAX_BD_NUM + MAX_BD_NUM_PLUS*state->age[ts19_flag]) - state->building[ts19_flag].size() < 5) return 0;
	if (my_produce_num < 2 * my_defense_num) return 0;
	initFiremap_point();
	
	std::list<SoldierPoint>::iterator iter = SoldierPointList.begin();
	for (; iter != SoldierPointList.end(); ++iter)
	{
		if (OriginalSoldierAttribute[iter->soldier_type][ACTION_MODE] == MOVING_ATTACK) continue;
		//LOG(iter->fire); LOG("  "); LOGN(float(OriginalSoldierAttribute[iter->soldier_type][SOLDIER_ORIGINAL_HEAL]) / SoldierCD[iter->soldier_type]);
		if (iter->fire >= float(OriginalSoldierAttribute[iter->soldier_type][SOLDIER_ORIGINAL_HEAL])/SoldierCD[iter->soldier_type]-0.0001) continue;
		if (!build_defense_outside(iter->pos, defense_choice_outside[iter->soldier_type][state->age[ts19_flag]])) return 0;
		if (iter->soldier_type==ULTRON)
			if (!iter->musk)
				if (!build_defense_outside(iter->pos, Musk)) return 0;
	}
	
	return 0;
}

int defense_produce_inbase_strategy()
{
	// check whether need defense
	bool defense_flag = false;
	int save_turn = 5;
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = 0; j < Soldier_Type; ++j)
			if ((nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] -  MAP_SIZE/10) <= save_turn * OriginalSoldierAttribute[j][SPEED])
				defense_flag = true;
	if (!defense_flag) return 0;
	
	// Get my fire now
	initFiremap();

	//Set defense building according to soldier dist
	bool defense_road_soldier[ROADMAX + 1][Soldier_Type];
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = 0; j < Soldier_Type; ++j)
			defense_road_soldier[i][j] = false;
	bool continueflag;
	int mindis,road;
	SoldierName soldier_type;
	BuildingType building_type;
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = TURING_MACHINE; j <= ULTRON; ++j)
		{
			mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - MAP_SIZE / 10) / OriginalSoldierAttribute[j][SPEED];
			if (mindis <= save_turn)
			{
				build_defense_inbase(i, Musk);
				break;
			}
		}

	while (true)
	{
		//determine the road and soldier to defense
		continueflag = false;
		for (int i=1;i<=ROADCNT;++i)
			for (int j=0;j< Soldier_Type;++j)
				if (!defense_road_soldier[i][j])
				{
					if (!continueflag)
					{
						continueflag = true;
						mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - MAP_SIZE / 10) / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis >  (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - MAP_SIZE / 10) / OriginalSoldierAttribute[j][SPEED])
					{
						mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - MAP_SIZE / 10) / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis == (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - MAP_SIZE / 10) / OriginalSoldierAttribute[j][SPEED])
					{
						if (defense_order(SoldierName(j), soldier_type))
						{
							//mindis = nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][ATTACK_RANGE];
							road = i;
							soldier_type = SoldierName(j);
							continue;
						}
					}
				}
		if (!continueflag) break;
		if (mindis >= save_turn) break;
		defense_road_soldier[road][soldier_type] = true;
		building_type = defense_choise[soldier_type][state->age[ts19_flag]];
		if (building_type == __Base) continue;
		//if ((soldier_type == TURING_MACHINE) || (soldier_type = ULTRON)) build_defense_inbase(road, Musk);
		build_defense_inbase(road, building_type);	
	}
	return 0;
}

int defense_produce_defense_strategy()
{

	// check whether need defense
	bool defense_flag = false;
	int save_turn = 5;
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = 0; j < Soldier_Type; ++j)
			if (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1) <= save_turn * OriginalSoldierAttribute[j][SPEED])
				defense_flag = true;
	if (!defense_flag) return 0;

	if (setup_control)
	{
		setup_control = false;
		initBuildmap();
	}

	// Get my fire now
	initFiremap();

	//Set defense building according to soldier dist
	bool defense_road_soldier[ROADMAX + 1][Soldier_Type];
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = 0; j < Soldier_Type; ++j)
			defense_road_soldier[i][j] = false;
	bool continueflag;
	int mindis, road;
	SoldierName soldier_type;
	BuildingType building_type;
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = TURING_MACHINE; j <= ULTRON; ++j)
		{
			mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1)) / OriginalSoldierAttribute[j][SPEED];
			if (mindis <= save_turn)
			{
				build_defense_road(i, Musk);
				break;
			}
		}

	while (true)
	{
		//determine the road and soldier to defense
		continueflag = false;
		for (int i = 1; i <= ROADCNT; ++i)
			for (int j = 0; j < Soldier_Type; ++j)
				if (!defense_road_soldier[i][j])
				{
					if (!continueflag)
					{
						continueflag = true;
						mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1)) / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis > (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1)) / OriginalSoldierAttribute[j][SPEED])
					{
						mindis = (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1)) / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis == (nearest_soldierdis[i][j] - OriginalSoldierAttribute[j][ATTACK_RANGE] - (MAP_SIZE >> 1)) / OriginalSoldierAttribute[j][SPEED])
					{
						if (defense_order(SoldierName(j), soldier_type))
						{
							//mindis = nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][ATTACK_RANGE];
							road = i;
							soldier_type = SoldierName(j);
							continue;
						}
					}
				}
		if (!continueflag) break;
		if (mindis >= save_turn) break;
		defense_road_soldier[road][soldier_type] = true;
		building_type = defense_choise[soldier_type][state->age[ts19_flag]];
		if (building_type == __Base) continue;
		//if ((soldier_type == TURING_MACHINE) || (soldier_type = ULTRON)) build_defense_inbase(road, Musk);
		build_defense_road(road, building_type);
	}
	return 0;
}

int defense_update_strategy()
{
	int bd_num = can_upgrade_defense_num;
	while (bd_num > 0)
		bd_num = upgrade_defense(bd_num);
	return 0;
}

int update_age()
{
	coming_age = state->age[ts19_flag];
	if ((myresource >= UPDATE_COST + UPDATE_COST_SQUARE * state->age[ts19_flag] * state->age[ts19_flag]) && (state->age[ts19_flag]<AI))
	{
		updateAge();
		++command_num;
		myresource -= UPDATE_COST + UPDATE_COST_SQUARE * state->age[ts19_flag] * state->age[ts19_flag];
		coming_age = Age(state->age[ts19_flag] + 1);
#ifdef DEBUG

		file << "turn: " << state->turn << " UPDATAEAGE" << endl;

#endif // DEBUG
		return 1;
	}
	return 0;
}

int maintain()
{
	
	BuildingType maxtype;
	int maxlevel,building_id;
	bool flag = true,building_win;
	bool* maintain_flag = new bool[state->building[ts19_flag].size()];
	for (int i = 0; i < state->building[ts19_flag].size(); ++i) maintain_flag[i] = true;
	//maintain DEFENSE fisrt
	while (command_num < 50)
	{
		flag = false;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] != DEFENSIVE_BUILDING)
				continue;
			if (state->building[ts19_flag][i].heal >= -1 + (1 + float(state->building[ts19_flag][i].level)*0.5) * OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_HEAL])
				continue;
			if (maintain_flag[i]) continue;
			if (!flag)
			{
				flag = true;
				maxtype = state->building[ts19_flag][i].building_type;
				maxlevel = state->building[ts19_flag][i].level;
				building_id = i;
				continue;
			}
			building_win = false;
			if (state->building[ts19_flag][i].building_type == Musk)
			{
				if (maxtype != Musk) building_win = true;
				else if (maxlevel < state->building[ts19_flag][i].level) building_win = true;
			}
			else
			{
				if (maxtype != Musk)
				{
					if (maxtype < state->building[ts19_flag][i].building_type) building_win = true;
					else if ((maxtype == state->building[ts19_flag][i].building_type)&&(maxlevel < state->building[ts19_flag][i].level)) building_win = true;
				}
			}
			if (building_win)
			{
				maxtype = state->building[ts19_flag][i].building_type;
				maxlevel = state->building[ts19_flag][i].level;
				building_id = i;
			}
		}
		if (!flag) break;
		maintain_flag[building_id] = true;
		mymaintain(building_id, state->building[ts19_flag][building_id].building_type, state->building[ts19_flag][building_id].level);
	}

	//maintain resource second
	int maxheal;
	while (command_num < 50)
	{
		flag = false;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (state->building[ts19_flag][i].building_type != Programmer)
				continue;
			if (state->building[ts19_flag][i].heal >= -1 + (1 + float(state->building[ts19_flag][i].level)*0.5) * OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_HEAL])
				continue;
			if (maintain_flag[i]) continue;
			if (!flag)
			{
				flag = true;
				maxheal = state->building[ts19_flag][i].heal;
				building_id = i;
				continue;
			}
			if (maxheal < state->building[ts19_flag][i].heal)
			{
				maxheal = state->building[ts19_flag][i].heal;
				building_id = i;
			}
		}
		if (!flag) break;
		maintain_flag[building_id] = true;
		mymaintain(building_id, state->building[ts19_flag][building_id].building_type, state->building[ts19_flag][building_id].level);
	}

	//maintain produce third
	while (command_num < 50)
	{
		flag = false;
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] != PRODUCTION_BUILDING)
				continue;
			if (state->building[ts19_flag][i].heal >= -1 + (1 + float(state->building[ts19_flag][i].level)*0.5) * OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][ORIGINAL_HEAL])
				continue;
			if (maintain_flag[i]) continue;
			if (!flag)
			{
				flag = true;
				maxtype = state->building[ts19_flag][i].building_type;
				maxlevel = state->building[ts19_flag][i].level;
				building_id = i;
				continue;
			}
			building_win = false;
			if (maxtype < state->building[ts19_flag][i].building_type) building_win = true;
			else if ((maxtype == state->building[ts19_flag][i].building_type) && (maxlevel < state->building[ts19_flag][i].level)) building_win = true;
			if (building_win)
			{
				maxtype = state->building[ts19_flag][i].building_type;
				maxlevel = state->building[ts19_flag][i].level;
				building_id = i;
			}
		}
		if (!flag) break;
		maintain_flag[building_id] = true;
		mymaintain(building_id, state->building[ts19_flag][building_id].building_type, state->building[ts19_flag][building_id].level);
	}
	return 0;
}

int sell_trash_outside()
{
	if (my_bd_num < MAX_BD_NUM + MAX_BD_NUM_PLUS*state->age[ts19_flag]) return 0;
	int need_bd_point = OriginalBuildingAttribute[Ohm][ORIGINAL_BUILDING_POINT];
	if (state->age[ts19_flag]>=NETWORK) need_bd_point = OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_BUILDING_POINT];
	if (state->age[ts19_flag] >= AI) 
		need_bd_point = mymin(OriginalBuildingAttribute[Musk][ORIGINAL_BUILDING_POINT],OriginalBuildingAttribute[Hawkin][ORIGINAL_BUILDING_POINT]);
	int need_building = floor(max_bd_point / need_bd_point) + 1;
	//sell useless defense
	sell_num = 0;
	if (state->age[ts19_flag] >= NETWORK)
	{
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if ((state->building[ts19_flag][i].building_type == Bool)||
				(state->building[ts19_flag][i].building_type == Ohm))
					mysell(i);
			if (sell_num >= need_building) return 0;
			if (command_num >= 50) return 0;
		}
	}
	if (state->age[ts19_flag] >= AI)
	{
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if ((state->building[ts19_flag][i].building_type == Larry_Roberts) ||
				(state->building[ts19_flag][i].building_type == Mole))
				mysell(i);
			if (sell_num >= need_building) return 0;
			if (command_num >= 50) return 0;
		}
	}

	//sell useless resource
	if ((state->age[ts19_flag] == AI)&&(my_resource_num>resource_num))
	{
		int min_programmer_dis, min_programmer_level, sell_programmer_id;
		int x, y;
		while (command_num < 50)
		{
			if (my_resource_num <= resource_num) break;
			min_programmer_level = 100;
			min_programmer_dis = 10000;
			for (int i = 0; i<state->building[ts19_flag].size(); ++i)
				if (state->building[ts19_flag][i].building_type == Programmer)
					if (!sell_list[i])
					{
						x = trans(state->building[ts19_flag][i].pos.x);
						y = trans(state->building[ts19_flag][i].pos.y);
						if (min_programmer_dis < mindismap[x][y]) continue;
						if (min_programmer_dis > mindismap[x][y])
						{
							min_programmer_dis = mindismap[x][y];
							min_programmer_level = state->building[ts19_flag][i].level;
							sell_programmer_id = i;
							continue;
						}
						if (min_programmer_level > state->building[ts19_flag][i].level)
						{
							min_programmer_level = state->building[ts19_flag][i].level;
							sell_programmer_id = i;
							continue;
						}
					}
			mysell(sell_programmer_id);
			--my_resource_num;
			if (sell_num >= need_building) return 0;
		}
	}

	
	return 0;
}

bool useless_defense(BuildingType building_type)
{
	for (SoldierName i = BIT_STREAM; i < Soldier_Type; i = SoldierName(i + 1))
		if (defense_choise[i][state->age[ts19_flag]] == building_type) return false;
	return true;
}

int sell_trash()
{
	//if (state->age[ts19_flag] <= CIRCUIT) return 0;
	int need_bd_point = 10000;
	int need_building = 0;
	int firedis;
	if (sell_for_defense)
	for (BuildingType building_type = Bool; building_type <= Hawkin; building_type = BuildingType(building_type + 1))
	{
		firedis = 0;
		for (int i = 1; i <= ROADCNT; ++i)
			firedis += mymax(0, needfiremap_inbase[i][building_type] - firemap_inbase[i][building_type]);
		need_building += ceil(firedis / MAX_FIRE);
		if (firedis > 0)
			if (need_bd_point > OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT])
				need_bd_point = OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];
	}
	if (sell_for_produce)
	{
		BuildingType building_type = destroy_choice();
		if (need_bd_point > OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT])
			need_bd_point = OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];
		need_building += (enemy_procude_num - my_produce_num) / 2;
	}
	//sell useless produce
	need_building = mymin(need_building, floor(max_bd_point / need_bd_point));
	sell_num = 0;

	if (my_bd_num == MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag])
	{
		if (state->age[ts19_flag] >= NETWORK)
			for (int i = 0; i < state->building[ts19_flag].size(); ++i)
			{
				if //((state->building[ts19_flag][i].building_type == Shannon)||
					((state->building[ts19_flag][i].building_type == Thevenin) ||
					(state->building[ts19_flag][i].building_type == Norton))// ||
					//(state->building[ts19_flag][i].building_type == Von_Neumann))
				if (command_num < 50)
				{
					mysell(i);
				}
				/*if (state->age[ts19_flag] == AI)
					if (state->building[ts19_flag][i].building_type == Berners_Lee)
						if (command_num < 50)
					{
							mysell(i);
					}*/
			}	
	}

#ifdef DEBUG
	file << "sell_for defense:" << sell_for_defense << endl;
	file << "useless produce: " << sell_num << endl;
	file << "myresource :" << my_resource_num << "  resource_num:  " << resource_num << endl;
#endif // DEBUG


	if (sell_num >= need_building) return 0;
	//sell useless defense
	//sell in the sort of heal
	bool sell_flag = true, can_sell_defense = true;
	bool find_defense_list[BUILDINGMAX];
	int min_heal,sell_building;
	BuildingType building_type;
	for (int i = 0; i < BUILDINGMAX; ++i) find_defense_list[i] = false;
	while (command_num < 50)
	{
		sell_flag = false;
		min_heal = 100000;
		for (int i=0;i<state->building[ts19_flag].size();++i)
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == DEFENSIVE_BUILDING)
				if (!find_defense_list[i])
					if (state->building[ts19_flag][i].heal < min_heal)
					{
						sell_flag = true;
						min_heal = state->building[ts19_flag][i].heal;
						sell_building = i;
					}
		if (!sell_flag) break;
		find_defense_list[sell_building] = true;
		if (!sell_list[sell_building])
		{
			building_type = state->building[ts19_flag][sell_building].building_type;
			can_sell_defense = true;
			for (int road = 1; road <= ROADCNT; ++road)
				if ((firemap_inbase[road][building_type] - needfiremap_inbase[road][building_type] < building_firemap[sell_building][road]) &&
					(building_firemap[sell_building][road] > 0))
					can_sell_defense = false;
			if (can_sell_defense) mysell(sell_building);
		}
		if (sell_num >= need_building) return 0;
	}

#ifdef DEBUG
	file << "useless defense: " << sell_num << endl;
#endif // DEBUG

	//sell useless resource
	int min_programmer_level, min_programmer_heal,sell_programmer_id;
	while (command_num < 50)
	{
		if (my_resource_num <= resource_num) break;
		min_programmer_level = 100; 
		min_programmer_heal = 10000;
		for (int i = 0; i<state->building[ts19_flag].size(); ++i)
			if (state->building[ts19_flag][i].building_type == Programmer)
				if (!sell_list[i])
				{
					if (min_programmer_level < state->building[ts19_flag][i].level) continue;
					if (min_programmer_level > state->building[ts19_flag][i].level)
					{
						min_programmer_heal = state->building[ts19_flag][i].heal;
						min_programmer_level = state->building[ts19_flag][i].level;
						sell_programmer_id = i;
						continue;
					}
					if (min_programmer_heal > state->building[ts19_flag][i].heal)
					{
						min_programmer_heal = state->building[ts19_flag][i].heal;
						sell_programmer_id = i;
						continue;
					}
				}
		mysell(sell_programmer_id);
		--my_resource_num;
		if (sell_num >= need_building) return 0;
	}
	BuildingType mintype;

#ifdef DEBUG
	file << "useless resource: " << sell_num << endl;
#endif // DEBUG

	//sell produce
	while (command_num < 50)
	{
		sell_flag = false;
		mintype = Building_Type;
		for (int i = state->building[ts19_flag].size()-1;i>=0; --i)
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == PRODUCTION_BUILDING)
				if (!sell_list[i])
					if (state->building[ts19_flag][i].building_type < mintype)
					{
						sell_flag = true;
						mintype = state->building[ts19_flag][i].building_type;
						sell_building = i;
					}
		if (!sell_flag) break;
		mysell(sell_building);
		if (sell_num >= need_building) return 0;
	}

#ifdef DEBUG
	file << "sell almost everything: " << sell_num << endl;
#endif // DEBUG

	return 0;
}

int sell_defense()
{
	//if (state->age[ts19_flag] <= CIRCUIT) return 0;
	int need_bd_point = 10000;
	int need_building = 0;
	int firedis;
	if (sell_for_defense)
		for (BuildingType building_type = Bool; building_type <= Hawkin; building_type = BuildingType(building_type + 1))
		{
			firedis = 0;
			for (int i = 1; i <= ROADCNT; ++i)
				firedis += mymax(0, needfiremap_inbase[i][building_type] - soldiernum_in_road[i][building_type] * MAX_FIRE);
			need_building += ceil(firedis / MAX_FIRE);
			if (firedis > 0)
				if (need_bd_point > OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT])
					need_bd_point = OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];
		}
	if (sell_for_produce)
	{
		BuildingType building_type = destroy_choice();
		if (need_bd_point > OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT])
			need_bd_point = OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];
		need_building += (enemy_procude_num - my_produce_num) / 2;
	}
	else
	{
		if (need_bd_point > OriginalBuildingAttribute[Kuen_Kao][ORIGINAL_BUILDING_POINT])
			need_bd_point = OriginalBuildingAttribute[Kuen_Kao][ORIGINAL_BUILDING_POINT];
	}
	
	need_building = mymin(need_building, floor(max_bd_point / need_bd_point));
	sell_num = 0;
	//sell useless produce
	if ((state->age[ts19_flag] >= NETWORK)||(coming_age >= NETWORK))
		for (int i = 0; i < state->building[ts19_flag].size(); ++i)
		{
			if ((state->building[ts19_flag][i].building_type == Shannon) ||
				(state->building[ts19_flag][i].building_type == Thevenin) ||
				(state->building[ts19_flag][i].building_type == Norton))// ||
																		//(state->building[ts19_flag][i].building_type == Von_Neumann))
			{
				if ((command_num < 50) && (sell_num < floor(max_bd_point / need_bd_point)))
				{
					mysell(i);
				}
				else break;
			}
		}

	if (state->age[ts19_flag] >= AI)
		while (command_num < 50)
		{
			int i = 0;
			if (my_resource_num <= resource_num) break;
			if (sell_num >= floor(max_bd_point / need_bd_point)) break;
			for (; i < state->building[ts19_flag].size(); ++i)
				if (state->building[ts19_flag][i].building_type == Programmer)
					if (!sell_list[i])
					{
						if (can_produce_soldier(Position(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
						{
							mysell(i);
							--my_resource_num;
							break;
						}
					}
			if (i == state->building[ts19_flag].size()) break;
		}
	

#ifdef DEBUG
	file << "sell_for defense:" << sell_for_defense << endl;
	file << "myresource :" << my_resource_num << "  resource_num:  " << resource_num << endl;
#endif // DEBUG
	
	//sell useless defense
	//sell in the sort of heal
	bool sell_flag = true, can_sell_defense = true;
	bool find_defense_list[BUILDINGMAX];
	int min_heal, sell_building;
	BuildingType building_type;
	Position tmppos,kernel_pos,target_pos;
	int color;
	for (int i = 0; i < BUILDINGMAX; ++i) find_defense_list[i] = false;
	while (command_num < 50)
	{
		sell_flag = false;
		min_heal = 100000;
		for (int i = 0; i<state->building[ts19_flag].size(); ++i)
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == DEFENSIVE_BUILDING)
				if (!find_defense_list[i])
					if (state->building[ts19_flag][i].heal < min_heal)
					{
						sell_flag = true;
						min_heal = state->building[ts19_flag][i].heal;
						sell_building = i;
					}
		if (!sell_flag) break;
		find_defense_list[sell_building] = true;
		if (!sell_list[sell_building])
		{
			can_sell_defense = true;
			building_type = state->building[ts19_flag][sell_building].building_type;
			tmppos = state->building[ts19_flag][sell_building].pos;
			tmppos = Position(trans(tmppos.x), trans(tmppos.y));
			color = defense_road[tmppos.x][tmppos.y];
			kernel_pos = kernel_defense[color];
			if ((kernel_pos.x != -1) || (kernel_pos.y != -1))
			{
				int error_gap = defense_gap;
				if (building_type != Mole) error_gap >>= 1;
				target_pos = kernel_target[color];
				if ((tmppos.x <= target_pos.x + error_gap) && (tmppos.y <= target_pos.y + error_gap) &&
					(tmppos.x + tmppos.y >= kernel_pos.x + kernel_pos.y) &&
					(tmppos.x + tmppos.y <= kernel_pos.x + kernel_pos.y + defense_gap))
					can_sell_defense = false;
			}
			if (useless_defense(building_type)) can_sell_defense = true;
			if (can_sell_defense) mysell(sell_building);
		}
		//if (sell_num >= need_building) return 0;
		if (sell_num >= floor(max_bd_point / need_bd_point)) break;
	}

#ifdef DEBUG
	file << "useless defense: " << sell_num << endl;
#endif // DEBUG
	if (sell_num >= need_building) return 0;
	//sell useless resource
	int min_programmer_level, min_programmer_heal, sell_programmer_id;
	while (command_num < 50)
	{
		if (my_resource_num <= resource_num) break;
		min_programmer_level = 100;
		min_programmer_heal = 10000;
		for (int i = 0; i<state->building[ts19_flag].size(); ++i)
			if (state->building[ts19_flag][i].building_type == Programmer)
				if (!sell_list[i])
				{
					if (can_produce_soldier(Position(trans(state->building[ts19_flag][i].pos.x), trans(state->building[ts19_flag][i].pos.y))))
					{
						min_programmer_heal = state->building[ts19_flag][i].heal;
						min_programmer_level = state->building[ts19_flag][i].level;
						sell_programmer_id = i;
						break;
					}
					if (min_programmer_level < state->building[ts19_flag][i].level) continue;
					if (min_programmer_level > state->building[ts19_flag][i].level)
					{
						min_programmer_heal = state->building[ts19_flag][i].heal;
						min_programmer_level = state->building[ts19_flag][i].level;
						sell_programmer_id = i;
						continue;
					}
					if (min_programmer_heal > state->building[ts19_flag][i].heal)
					{
						min_programmer_heal = state->building[ts19_flag][i].heal;
						sell_programmer_id = i;
						continue;
					}
				}
		mysell(sell_programmer_id);
		--my_resource_num;
		if (sell_num >= need_building) return 0;
	}
	BuildingType mintype;

#ifdef DEBUG
	file << "useless resource: " << sell_num << endl;
#endif // DEBUG

	//sell produce
	while (command_num < 50)
	{
		sell_flag = false;
		mintype = Building_Type;
		for (int i = state->building[ts19_flag].size() - 1; i >= 0; --i)
			if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == PRODUCTION_BUILDING)
				if (!sell_list[i])
					if (state->building[ts19_flag][i].building_type < mintype)
					{
						sell_flag = true;
						mintype = state->building[ts19_flag][i].building_type;
						sell_building = i;
					}
		if (!sell_flag) break;
		mysell(sell_building);
		if (sell_num >= need_building) return 0;
	}

#ifdef DEBUG
	file << "sell almost everything: " << sell_num << endl;
#endif // DEBUG

	return 0;
}

void Setup()
{
	initBuildmap();
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	
	while (bd_num > 0)
	{
		bd_num = build_programmer(bd_num);		
	}
		
	return;
}

void Setup_inbase()
{
	initBuildmap();
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	while (bd_num>0)
		bd_num = build_programmer_inbase(bd_num);
	return;
}

void Setup_defense()
{
	setup_control = true;
	defense_control = true;
	initBuildmap();
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	while (bd_num>0)
		bd_num = build_programmer_defense(bd_num);
	return;
}

void excute()
{
	int resource_limit[6] = { 40,30,40,60,40,20 };
	initBuildmap();
	Getsoldierpoint();
	//soldierSituation();
	
	//resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
	// Stage 1 - Get resource
	resource_num = resource_limit[state->age[ts19_flag]];
	resource_produce_strategy();
	if (state->age[ts19_flag] == BIT) { update_age(); return; }
	// Stage 2 - Build to defense
	defense_produce_strategy();

	// Stage 3 - Create soldier to destroy all building
	//if (state->turn%10==0)
	attack_produce_strategy();

	// Stage 4 update
	update_age();
	sell_trash_outside();
	resource_update_strategy();
	defense_update_strategy();
	attack_update_strategy();
	maintain();
	// Stage 5 - Update Age
	

	// Stage 6 - sell

	return;
}

void excute_army_from_base()
{
	int resource_limit[6] = { 40,30,40,60,40,40 };
	initBuildmap();
	soldierSituation();

	//resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
	// Stage 1 - Get resource
	resource_num = resource_limit[state->age[ts19_flag]];
	resource_num = mymax(resource_num, enemy_resource_num - 10);
	resource_produce_inbase_strategy();
	//if (state->age[ts19_flag] == BIT) { update_age(); return; }
	// Stage 2 - Build to defense
	defense_produce_inbase_strategy();

	// Stage 3 - Create soldier to destroy all building
	int relax = 0;
	//if (state->turn % 10 == 0)
	//{
	if (!sell_for_defense)
	//	relax = attack_produce_inbase_strategy();//relax = -1 means we can make more programmer
	relax = attack_produce_army_strategy();
	//}
	

	// Stage 4 update
	if (relax == -1)
	{
		resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
		resource_produce_inbase_strategy();
	}
	update_age();
	sell_trash();

	resource_update_strategy();


	defense_update_strategy();
	attack_update_strategy();

	maintain();

	// Stage 5 - Update Age
	
	// Stage 6 - sell
	

	return;
}

void excute_defense()
{
	int resource_limit[6] = { 30,30,60,60,60,30 };
	initBuildmap();
	soldierSituation();

	//resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
	// Stage 1 - Get resource
	resource_num = resource_limit[state->age[ts19_flag]];
	resource_num = mymax(resource_num, enemy_resource_num - 10);
	resource_produce_defense_strategy();
	//if (state->age[ts19_flag] == BIT) { update_age(); return; }
	// Stage 2 - Build to defense
	if (nearest_enemybuilding_dis < (MAP_SIZE >> 1))
	{
		enemy_inbase = true;
		if (setup_control)
		{
			setup_control = false;
			initBuildmap();
		}
	}
	if (enemy_inbase)
		defense_produce_inbase_strategy();
	else
		defense_produce_defense_strategy();

	// Stage 3 - Create soldier to destroy all building
	int relax = 0;
	//if (state->turn % 10 == 0)
	//{
	if (!sell_for_defense)
		relax = attack_produce_inbase_strategy();//relax = -1 means we can make more programmer
												 //}


	// Stage 4 update
	if (relax == -1)
	{
		resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
		resource_produce_defense_strategy();
	}
	update_age();
	sell_defense();

	resource_update_strategy();


	defense_update_strategy();
	attack_update_strategy();

	maintain();

	// Stage 5 - Update Age

	// Stage 6 - sell


	return;
}

void f_player()
{
	for (int i = 0; i<state->building[ts19_flag].size(); ++i)
		if (state->building[ts19_flag][i].building_type == __Base)
		{
			my_base_hp = state->building[ts19_flag][i].heal;
			break;
		}
	for (int i = 0; i<state->building[1 - ts19_flag].size(); ++i)
		if (state->building[1 - ts19_flag][i].building_type == __Base)
		{
			enemy_base_hp = state->building[1 - ts19_flag][i].heal;
			break;
		}
#ifdef DEBUG
	file << "myhp: " << my_base_hp << endl;
	file << "enemyhp: " << enemy_base_hp << endl;
	file << "turn: " << state->turn << endl;
	file << "building: " << state->building[ts19_flag].size() << endl;
	file << "resource: " << state->resource[ts19_flag].resource << endl;
	file << "enemyresource: " << state->resource[1-ts19_flag].resource << endl;
#endif // DEBUG

	tic();
	command_num = 0;
	max_bd_point = state->resource[ts19_flag].building_point;
	if (ts19_flag >= -1)
	{
		if (state->turn == 0)
		{
			init();
			//Setup_inbase();
			Setup_defense();
		}
		else excute_defense();
		//else excute_army_from_base();
	}
	else
	{
		if (state->turn == 0)
		{
			init();
			Setup(); 

		}
		else excute();
	}

	toc();
};

