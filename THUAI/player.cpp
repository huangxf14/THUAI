/*
TODO List
- Finish defense inbase
- Add defense for outside strategy

*/

//#define DEBUG
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
Position queue[MAP_SIZE*MAP_SIZE];
Position* root = new Position[MAP_SIZE*MAP_SIZE];
Position nextpos[MAP_SIZE][MAP_SIZE];
Position roadroot[ROADMAX];
int discount[41] = { 0 };
int ROADCNT = 0;//count from 1,ROADCNT = the max number of road
int ENEMY_EDGE = MAP_SIZE - BASE_SIZE,MY_EDGE = BASE_SIZE - 1;
int my_base_hp, enemy_base_hp;
int my_bd_num = 0;

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
int enemy_procude_num = 0;
int my_building_num[Building_Type] = { 0 }, enemy_building_num[Building_Type] = { 0 };

inline int trans(int x)//Use this everywhere when x means pos
{
	return ts19_flag ? (MAP_SIZE - 1 - x) : x;
}

struct SoldierPoint
{
	Position pos;
	int cd;
	SoldierPoint(Position pos = Position(0, 0), int cd = 0) :
		pos(Position(trans(pos.x), trans(pos.y))), cd(cd) {}
};

std::list<SoldierPoint> SoldierPointList;
int soldier_distrubutemap[MAP_SIZE][MAP_SIZE][Soldier_Type];
int maxsoliderid = -1;
int SoldierCD[Soldier_Type];
int soldiercount[ROADMAX+1][Soldier_Type] = {0};
int nearest_soldierdis[ROADMAX + 1][Soldier_Type];
int soldier_save_dis = MAP_SIZE >> 1;
Position my_soldier_pos[MAP_SIZE][MAP_SIZE];
int firemap_inbase[ROADMAX + 1][Building_Type], needfiremap_inbase[ROADMAX + 1][Building_Type];//the fire from each kind of building in each road
int soldier_freq[ROADMAX + 1][Soldier_Type];//soldier num in one road in one turn
bool sell_for_defense = false;

//remember Musk when building
const BuildingType defense_choise[Soldier_Type][AI+1] = {
	{ Bool,    Bool,  Bool, Bool,        Larry_Roberts, Hawkin },
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Hawkin },
	{ __Base,  Ohm,  Ohm,  Ohm,         Larry_Roberts, Larry_Roberts },
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin },
	{ Bool,    Bool, Bool, Bool,        Larry_Roberts, Larry_Roberts },
	{ __Base,  Ohm,  Ohm,  Monte_Carlo, Monte_Carlo,   Monte_Carlo },
	{ Bool,    Bool, Mole, Mole,        Robert_Kahn,   Hawkin },
	{ __Base,  Ohm,  Mole, Mole,        Mole,          Hawkin }
};

int mymin(int a, int b)
{
	if (a < b) return a;
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

void init()
{
	for (BuildingType i = Shannon; i <= Tony_Stark; i = BuildingType(i + 1))
		SoldierCD[OriginalBuildingAttribute[i][TARGET]] = OriginalBuildingAttribute[i][CD];

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
		for (int i = 0; i <= BD_RANGE; ++i)
			for (int j = 0; j + i <= BD_RANGE; ++j)
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
		for (int i = 0; i <= BD_RANGE; ++i)
			for (int j = 0; j + i <= BD_RANGE; ++j)
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
	if (x >= 1) { buildmap[x - 1][y] = 2; enemy_buildmap[x - 1][y] = 2; }
	if (y >= 1) { buildmap[x][y - 1] = 2; enemy_buildmap[x][y - 1] = 2; }
	if (x + 1 < MAP_SIZE) { buildmap[x + 1][y] = 2; enemy_buildmap[x + 1][y] = 2; }
	if (y + 1 < MAP_SIZE) { buildmap[x][y + 1] = 2; enemy_buildmap[x][y + 1] = 2; }
	
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
	my_bd_num = state->building[ts19_flag].size() - 1;
	my_resource_num = 0; 
	enemy_resource_num = 0;
	enemy_procude_num = 0;
	can_upgrade_reource_num = 0;
	can_upgrade_produce_num = 0;
	can_upgrade_defense_num = 0;
	for (int i = 0; i < BUILDING_TYPE; ++i) { my_building_num[i] = 0; enemy_building_num[i] = 0;}
	

	for (int i = 0; i < ROADCNT + 1; ++i) resource_area[i] = false;
	
	myresource = state->resource[ts19_flag].resource;
	mybdpoint = state->resource[ts19_flag].building_point;
	science_factor = 1 + state->age[ts19_flag] * AGE_INCREASE;
	
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
			if (state->building[ts19_flag][i].level < state->age[ts19_flag]) ++can_upgrade_produce_num;
		}
		if (OriginalBuildingAttribute[state->building[ts19_flag][i].building_type][BUILDING_TYPE] == DEFENSIVE_BUILDING)
		{
			if (state->building[ts19_flag][i].level < state->age[ts19_flag]) ++can_upgrade_defense_num;
		}
	}

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
				dis = my_base_dist(i, j);
				if (dis < nearest_enemybuilding_dis) nearest_enemybuilding_dis = 0;
			}

	return 0;
}

void drawfiremap_inbase(Position pos, BuildingType building_type)//this function is so difficult to finish
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
			firemap_inbase[i][building_type] += cnt;

	delete roadflag;
	return;
}

int initFiremap()
{
	for (int i = 1; i <= ROADCNT; ++i)
		for (BuildingType j = Bool; j <= Hawkin; j = BuildingType(j + 1))
			firemap_inbase[i][j] = 0;
	//what i have
	Position pos;
	for (int i = 0; i < state->building[ts19_flag].size(); ++i)
	{
		pos = state->building[ts19_flag][i].pos;
		drawfiremap_inbase(Position(trans(pos.x), trans(pos.y)), state->building[ts19_flag][i].building_type);
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
			needfiremap_inbase[i][Mole] = soldier_freq[i][ENIAC] * MAX_FIRE / 2;
			needfiremap_inbase[i][Robert_Kahn] = soldier_freq[i][TURING_MACHINE] * MAX_FIRE / 2;
			needfiremap_inbase[i][Monte_Carlo] = soldier_freq[i][OPTICAL_FIBER] * MAX_FIRE * 2 / 3;
			needfiremap_inbase[i][Larry_Roberts] = soldier_freq[i][VOLTAGE_SOURCE] + soldier_freq[i][CURRENT_SOURCE] 
													+ soldier_freq[i][BIT_STREAM] + soldier_freq[i][PACKET];
			if (needfiremap_inbase[i][Larry_Roberts] == 0) continue;
			if (needfiremap_inbase[i][Larry_Roberts] < ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3)))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3));
			}
			if (needfiremap_inbase[i][Larry_Roberts] > ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]))
			{
				needfiremap_inbase[i][Larry_Roberts] = ceil(OriginalSoldierAttribute[CURRENT_SOURCE][SOLDIER_ORIGINAL_HEAL] / OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK]);
			}
			needfiremap_inbase[i][Larry_Roberts] *= MAX_FIRE;
		}
		return 0;
	}
	if (state->age[ts19_flag] == AI)
	{
		for (int i = 1; i <= ROADCNT; ++i)
		{
			needfiremap_inbase[i][Monte_Carlo] = soldier_freq[i][OPTICAL_FIBER] * MAX_FIRE * 2 / 3;
			if ((soldier_freq[i][TURING_MACHINE] > 0) || (soldier_freq[i][ULTRON] > 0)) needfiremap_inbase[i][Musk] = MAX_FIRE;
			for (SoldierName soldier=BIT_STREAM;soldier<Soldier_Type;soldier=SoldierName(soldier+1))
				if (soldier != OPTICAL_FIBER)
				{
					if (soldier_freq[i][soldier] > 0)
					{
						needfiremap_inbase[i][Larry_Roberts] = MAX_FIRE * ceil(OriginalSoldierAttribute[PACKET][SOLDIER_ORIGINAL_HEAL] / (OriginalBuildingAttribute[Larry_Roberts][ORIGINAL_ATTACK] * 3));
						needfiremap_inbase[i][Hawkin] = MAX_FIRE;
						break;
					}
				}
		}
		return 0;
	}
	return 0;
}

int soldierSituation()
{
	//// clear List
	//std::list<SoldierPoint>::iterator iter = SoldierPointList.begin();
	//for (; iter != SoldierPointList.end(); ++iter)
	//{
	//	--iter->cd;
	//	if (iter->cd <= 0) SoldierPointList.erase(iter);
	//}
	//// Update List
	//for (int i = 0; i < state->soldier[1 - ts19_flag].size(); ++i)
	//{
	//	if (state->soldier[1 - ts19_flag][i].unit_id <= maxsoliderid) continue;
	//	SoldierPointList.push_back(SoldierPoint(state->soldier[1 - ts19_flag][i].pos,
	//		SoldierCD[state->soldier[1 - ts19_flag][i].soldier_name]));

	//}

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
	construct(building_type, Position(trans(pos.x),trans(pos.y)), Position(trans(soldier_pos.x),trans(soldier_pos.y)));
	++command_num;
	drawbuildingmap(pos.x, pos.y,0);
	drawfiremap_inbase(pos, building_type);
	my_soldier_pos[pos.x][pos.y] = soldier_pos;
	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE];// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];// *science_factor;
	++my_bd_num;

#ifdef DEBUG
	
	file << "turn: " << state->turn << " type:" << building_type << " pos: (" << pos.x << " ," <<pos.y<<") solider:("<< soldier_pos.x<<","<<soldier_pos.y<<")"<<endl;

#endif // DEBUG


	return true;
}

bool myupgrade(int unit_id, BuildingType building_type)
{
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
			if (!flag) continue;
			if (!myConstruct(Programmer, buildingPos)) return 0;
			else ++my_resource_num;
			--rest_num;
			break;
		}
		if (!flag) break;
	}
	
	int head, tail, cnt = 0,real_cnt=0;
	int tmpx, tmpy;
	
	while (rest_num > 0)
	{
		if (real_cnt == ROADCNT + 1) break;
		cnt = areasortid[real_cnt];
		if (resource_area[cnt]) {++real_cnt; continue;}
				
		//if (areacount[areasortid[cnt]] == 0) continue;
		//BFS
		head = 0;
		tail = 0;
		memset((void*)buildprogrammer_queueflag, 0, sizeof(bool)*MAP_SIZE*MAP_SIZE);
		for (int i = 0; i < saveArea[cnt].size(); ++i)
		{
			if (buildmap[saveArea[cnt][i].x][saveArea[cnt][i].y] == 2) continue;
			queue[tail++] = saveArea[cnt][i];
			buildprogrammer_queueflag[saveArea[cnt][i].x][saveArea[cnt][i].y] = true;
		}
		
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
		}
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
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type)) break;
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
		//LOG("turn: "); LOG(state->turn); LOG("  "); LOGN(rest_num);
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
	//for (int i = 1; i <= ROADCNT; ++i)
	//	if (*(building_num + i) == 0) roadflag[i] = true;
	for (int i = 2; i <= ROADCNT; ++i)
		if (!roadflag[i])
		if (maxnum < *(building_num + i))
		{
			road = i;
			maxnum = *(building_num + i);
		}
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
	int dis = int(OriginalBuildingAttribute[building_type][ORIGINAL_RANGE] * 0.8);
	Position pos;
	int maxscore, nowdis;
	int disgap = 4;
	int color, score;
	bool* roadflag = new bool[ROADCNT + 1];
	while (needfiremap_inbase[road][building_type] > firemap_inbase[road][building_type])
	{
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
		if (!myConstruct(building_type, pos)) { delete roadflag; return -1; }
	}
	
	delete roadflag;
	return 0;
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
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type)) break;
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
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type)) break;
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
			if (state->building[ts19_flag][i].building_type > level)
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
		if (!myupgrade(state->building[ts19_flag][index].unit_id, state->building[ts19_flag][index].building_type)) break;
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
	if (ts19_flag == 0)
	{
		building_type = Norton;
		//if (ohm_count >= 5) building_type = Shannon;
		if (state->age[ts19_flag] == NETWORK)
		{
			if (larry_count >= 3)
			{
				if (ohm_count < 5 ) building_type = Norton;
				else building_type = Shannon;
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

	while (bd_num > 0)
		bd_num = build_produce(bd_num, building_type);
	return 0;
}

int attack_produce_inbase_strategy()
{
	if (my_resource_num < resource_num) return 0;
	//if (can_upgrade_reource_num > 0) return 0;
	if (enemy_procude_num == 0)
	{
		if (state->age[ts19_flag] < AI) return -1;
		if (my_base_hp >= enemy_base_hp)
		{
			if (state->turn <= 800) return -1;
			if (state->resource[ts19_flag].resource > state->resource[1 - ts19_flag].resource) return -1;
			if (state->resource[ts19_flag].resource + (myresource - enemy_resource_num)*OriginalBuildingAttribute[Programmer][ORIGINAL_ATTACK] * (1 + AI*0.5)
				> state->resource[1 - ts19_flag].resource)
				return -1;
		}	
	}

	//Construct produce building
	int bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] + 1 - state->building[ts19_flag].size();
	BuildingType building_type;
	building_type = Shannon;
	if (my_building_num[Shannon] < enemy_building_num[Bool]) building_type = Thevenin;
	if (state->age[ts19_flag] == NETWORK)
		building_type = Kuen_Kao;
	if (state->age[ts19_flag] == AI)
	{
		if (enemy_building_num[Musk] == 0) building_type = Tony_Stark;
		else
		{
			if (my_building_num[Kuen_Kao] <= my_building_num[Tony_Stark]) building_type = Kuen_Kao;
			else building_type = Tony_Stark;
		}
	}
	while (bd_num > 0)
		bd_num = build_produce_inbase(bd_num, building_type);

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
	return 0;
}

int defense_produce_inbase_strategy()
{
	// check whether need defense
	bool defense_flag = false;
	int save_turn = 3;
	for (int i = 1; i <= ROADCNT; ++i)
		for (int j = 0; j < Soldier_Type; ++j)
			if (nearest_soldierdis[i][j] <= save_turn * OriginalSoldierAttribute[j][ATTACK_RANGE])
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
	while (true)
	{
		//determine the road and soldier to defense
		continueflag = false;
		for (int i=1;i<=ROADCNT;++i)
			for (int j=0;j<= Soldier_Type;++j)
				if (!defense_road_soldier[i][j])
				{
					if (!continueflag)
					{
						continueflag = true;
						mindis = nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis > nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][SPEED])
					{
						mindis = nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][SPEED];
						road = i;
						soldier_type = SoldierName(j);
						continue;
					}
					if (mindis == nearest_soldierdis[i][j] / OriginalSoldierAttribute[j][SPEED])
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
		if (mindis >= 5) break;
		defense_road_soldier[road][soldier_type] = true;
		building_type = defense_choise[soldier_type][state->age[ts19_flag]];
		if (building_type == __Base) continue;
		if ((soldier_type == TURING_MACHINE) || (soldier_type = ULTRON)) build_defense_inbase(road, Musk);
		build_defense_inbase(road, building_type);	
	}
	return 0;
}

int defense_update_strategy()
{
	return 0;
	int bd_num = can_upgrade_defense_num;
	while (bd_num > 0)
		bd_num = upgrade_defense(bd_num);
	return 0;
}

int update_age()
{
	if ((myresource > UPDATE_COST + UPDATE_COST_PLUS*state->age[ts19_flag]) && (state->age[ts19_flag]<AI))
	{
		updateAge();
		++command_num;
		myresource -= UPDATE_COST + UPDATE_COST_PLUS*state->age[ts19_flag];
#ifdef DEBUG

		file << "turn: " << state->turn << " UPDATAEAGE" << endl;

#endif // DEBUG
		return 1;
	}
	return 0;
}

int maintain()
{
	return 0;
}

int sell_trash()
{
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

void excute()
{
	int resource_limit[6] = { 40,30,40,60,40,10 };
	command_num = 0;
	initBuildmap();
	soldierSituation();
	
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
	resource_update_strategy();
	defense_update_strategy();
	attack_update_strategy();
	
	// Stage 5 - Update Age
	update_age();

	// Stage 6 - sell

	sell_trash();


	return;
}

void excute_army_from_base()
{
	int resource_limit[6] = { 40,30,40,60,40,10 };
	command_num = 0;
	initBuildmap();
	soldierSituation();

	//resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
	// Stage 1 - Get resource
	resource_num = resource_limit[state->age[ts19_flag]];
	resource_produce_inbase_strategy();
	//if (state->age[ts19_flag] == BIT) { update_age(); return; }
	// Stage 2 - Build to defense
	defense_produce_inbase_strategy();

	// Stage 3 - Create soldier to destroy all building
	int relax = 0;
	//if (state->turn % 10 == 0)
	//{
		relax = attack_produce_inbase_strategy();//relax = -1 means we can make more programmer
	//}
	

	// Stage 4 update
	resource_update_strategy();

	if (relax == -1)
	{
		resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
		resource_produce_inbase_strategy();
	}

	defense_update_strategy();
	attack_update_strategy();

	maintain();

	// Stage 5 - Update Age
	update_age();
	// Stage 6 - sell
	sell_trash();

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

#endif // DEBUG

	tic();
	if (state->turn == 0)
	{
		init();
		//Setup(); 
		Setup_inbase();

	}
	//else excute();
	else excute_army_from_base();
	toc();
};

