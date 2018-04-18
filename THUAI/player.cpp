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



extern State* state;
extern std::vector<State* > all_state;
extern int** ts19_map;
extern bool ts19_flag;
int colormap[MAP_SIZE][MAP_SIZE] = { 0 };//Seg area by road
int mindismap[MAP_SIZE][MAP_SIZE] = { 0 };//the min dis for every point to the nearest road
int minposmap[MAP_SIZE][MAP_SIZE] = { 0 };
int buildmap[MAP_SIZE][MAP_SIZE] = { 0 };//0 - useless;1 - road;2 - building&base;3 - useful;
int discount[41] = { 0 };
int ROADCNT = 0;
int ENEMY_EDGE = MAP_SIZE - BASE_SIZE;
int area_dis = 24;// 40;//the min distance for save area
int *minareadis;//min distance from enemybase of every area
int *areacount;//the cnt of allowed point of every area
int *areasortid;//sort area by the dis from enemybase(short to long)
std::vector<Position>* saveArea;
int game_stage = 0;//Change strategy arccording to stage
int myresource = 0, mybdpoint = 0;
int resource_num=MAX_BD_NUM,my_resource_num=0,can_upgrade_reource_num=0;//the number of resource building
int resource_limit[6] = { 40,30,40,60,40,10 };
int can_upgrade_produce_num = 0;//the number of produce
bool *resource_area;//whether this area have resource building
float science_factor = 1;
int *test;

#define ROADMAX 10
#define MAX_BD_POINT 80
#define MAX_BD_POINT_PLUS 60

//Use this everywhere when x means pos

inline int trans(int x)
{
	return ts19_flag ? (MAP_SIZE - 1 - x) : x;
}

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

void init()
{
	LOGN("INIT");
	//BFS for color map generation
	int *queue = new int[MAP_SIZE*MAP_SIZE];
	int *root = new int[MAP_SIZE*MAP_SIZE];
	int *rootpos = new int[MAP_SIZE*MAP_SIZE];
	int head = 0, tail = 0;
	int tx[4] = { -1,0,1,0 }, ty[4] = { 0,-1,0,1 };
	
	//Get Road first
	LOGN("Get Road first");
	for (int i = 0; i < BASE_SIZE + 1; ++i)
		if (ts19_map[trans(7)][trans(i)] == 1)
		{
			++ROADCNT;
			colormap[7][i] = ROADCNT;
			queue[tail] = 7 * MAP_SIZE + i;
			root[tail] = queue[tail];
			rootpos[tail] = queue[tail];
			++tail;
			int nowx = 7, nowy = i, tnowx, tnowy;
			bool find = true;
			while (find)
			{
				find = false;
				for (int j = 0; j < 4; ++j)
					if ((nowx + tx[j] >= 0) && (nowx + tx[j] < MAP_SIZE) && (nowy + ty[j] >= 0) && (nowy + ty[j] < MAP_SIZE))
						if ((colormap[nowx + tx[j]][nowy + ty[j]] == 0) && (ts19_map[trans(nowx + tx[j])][trans(nowy + ty[j])] == 1))
						{
							tnowx = nowx + tx[j];
							tnowy = nowy + ty[j];
							colormap[tnowx][tnowy] = ROADCNT;
							queue[tail] = tnowx * MAP_SIZE + tnowy;
							root[tail] = root[tail - 1];
							rootpos[tail] = queue[tail];
							++tail;
							find = true;
							if (i != 7) break;
						}	
				if (find)
				{
					nowx = tnowx;
					nowy = tnowy;
				}
			}	
		}
	for (int i = BASE_SIZE-1; i >= 0; --i)
		if (ts19_map[trans(i)][trans(7)] == 1)
		{
			++ROADCNT;
			colormap[i][7] = ROADCNT;
			queue[tail] = i * MAP_SIZE + 7;
			root[tail] = queue[tail];
			rootpos[tail] = queue[tail];
			++tail;
			int nowx = i, nowy = 7;
			bool find = true;
			while (find)
			{
				find = false;
				for (int j = 0; j < 4; ++j)
					if ((nowx + tx[j] >= 0) && (nowx + tx[j] < MAP_SIZE) && (nowy + ty[j] >= 0) && (nowy + ty[j] < MAP_SIZE))
						if ((colormap[nowx + tx[j]][nowy + ty[j]] == 0) && (ts19_map[trans(nowx + tx[j])][trans(nowy + ty[j])] == 1))
						{
							nowx += tx[j];
							nowy += ty[j];
							colormap[nowx][nowy] = ROADCNT;
							queue[tail] = nowx * MAP_SIZE + nowy;
							root[tail] = root[tail - 1];
							rootpos[tail] = queue[tail];
							++tail;
							find = true;
							break;
						}
			}
		}
	//BFS to get color and dis
	LOGN("BFS to get color and dis");
	while (head < tail)
	{
		int now = 0, nowx = queue[head] / MAP_SIZE, nowy = queue[head] % MAP_SIZE;
		if (mindismap[nowx][nowy] > 0) break;
		for (; now < 4; ++now)
		{
			if ((nowx + tx[now] < 0) || (nowx + tx[now] >= MAP_SIZE) || (nowy + ty[now] < 0) || (nowy + ty[now] >= MAP_SIZE))
				continue;
			if (head == 0)
				if (ts19_map[trans(nowx + tx[now])][trans(nowy + ty[now])] != 2)
					continue;
				else break;
			if (root[head]!=root[head-1])
				if (ts19_map[trans(nowx + tx[now])][trans(nowy + ty[now])] != 2)
					continue;
				else break;
			if ((nowx + tx[now])*MAP_SIZE + (nowy + ty[now]) == queue[head - 1])
				break;
		}
		int flag = -1;
		for (int i = 0; i < 3; ++i)
		{
			++now;
			if (now >= 4) now -= 4;
			if ((nowx + tx[now] < 0) || (nowx + tx[now] >= MAP_SIZE) || (nowy + ty[now] < 0) || (nowy + ty[now] >= MAP_SIZE))
				continue;
			if (((root[head+1]==root[head])&&((nowx + tx[now])*MAP_SIZE + (nowy + ty[now]) == queue[head + 1]))
				|| ((root[head + 1] != root[head]) && ((ts19_map[trans(nowx + tx[now])][trans(nowy + ty[now])] == 2))))
			{
				flag = 0;
				continue;
			}
			if ((colormap[nowx + tx[now]][nowy + ty[now]] == 0)&&(ts19_map[trans(nowx + tx[now])][trans(nowy + ty[now])] == 0))
			{
				queue[tail] = (nowx + tx[now])*MAP_SIZE + (nowy + ty[now]);
				root[tail] = root[head];
				rootpos[tail] = rootpos[head];
				mindismap[nowx + tx[now]][nowy + ty[now]] = 1;
				++discount[1];
				minposmap[nowx + tx[now]][nowy + ty[now]] = rootpos[tail];
				colormap[nowx + tx[now]][nowy + ty[now]] = ROADMAX + colormap[nowx][nowy] + flag;
				++tail;
			}
		}
		++head;
	}
	
	while (head < tail)
	{
		int nowx = queue[head] / MAP_SIZE, nowy = queue[head] % MAP_SIZE;
		for (int i = 0; i < 4; ++i)
		{
			if ((nowx + tx[i] < 0) || (nowx + tx[i] >= MAP_SIZE) || (nowy + ty[i] < 0) || (nowy + ty[i] >= MAP_SIZE))
				continue;
			if (colormap[nowx + tx[i]][nowy + ty[i]] > 0)
				continue;
			if (ts19_map[trans(nowx + tx[i])][trans(nowy + ty[i])] > 0)
				continue;
			queue[tail] = (nowx + tx[i])*MAP_SIZE + (nowy + ty[i]);
			root[tail] = root[head];
			rootpos[tail] = rootpos[head];
			colormap[nowx + tx[i]][nowy + ty[i]] = colormap[nowx][nowy];
			mindismap[nowx + tx[i]][nowy + ty[i]] = mindismap[nowx][nowy]+1;
			if (mindismap[nowx + tx[i]][nowy + ty[i]] > area_dis) ++discount[area_dis];
			else ++discount[mindismap[nowx + tx[i]][nowy + ty[i]]];
			minposmap[nowx + tx[i]][nowy + ty[i]] = rootpos[tail];
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
	resource_area = new bool[ROADCNT + 1];
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
			minareadis[colormap[x][y] - ROADMAX] = mymin(minareadis[colormap[x][y] - ROADMAX], enemy_base_dist(x, y));
			++areacount[colormap[x][y] - ROADMAX];
			saveArea[colormap[x][y] - ROADMAX].push_back(Position(x, y));
		}

	//sort area(from short to long)
	bool *flag = new bool[ROADCNT + 1];
	int mindis_tmp,minarea_tmp;
	for (int i = 0; i < ROADCNT + 1; ++i) flag[i] = false;
	for (int i = 0; i < ROADCNT + 1; ++i)
	{
		mindis_tmp = 10000;
		for (int j = 0; j < ROADCNT + 1; ++j)
		{
			if (flag[j]) continue;
			if (minareadis[j] < mindis_tmp)
			{
				mindis_tmp = minareadis[j];
				minarea_tmp = j;
			}
		}
		areasortid[i] = minarea_tmp;
		flag[minarea_tmp] = true;
	}
	
	
#ifdef DEBUG
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


	file << "area_dis: " << area_dis << endl;
	for (int i = 0; i < ROADCNT + 1; ++i)
		file << "area" << i << " :(dis," << minareadis[i] << ") (cnt," << areacount[i] << ")" << endl;
	cout << "Output init" << endl;
#endif 

	delete queue;
	delete root;
	delete rootpos;
	delete flag;
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

	buildmap[x][y] = 2;
	if (x >= 1) buildmap[x - 1][y] = 2;
	if (y >= 1)  buildmap[x][y - 1] = 2;
	if (x + 1 < MAP_SIZE) buildmap[x + 1][y] = 2;
	if (y + 1 < MAP_SIZE) buildmap[x][y + 1] = 2;
	
	return;
}

int initBuildmap()
{
	//buildmap: 0 - useless;1 - road;2 - building&base;3 - useful;
	//memcpy((void*)buildmap, (void*)ts19_map, MAP_SIZE*MAP_SIZE * sizeof(int));
	for (int i = 0; i < MAP_SIZE; ++i)
		for (int j = 0; j < MAP_SIZE; ++j)
			buildmap[i][j] = ts19_map[i][j];
	//init for resource
	my_resource_num = 0; 
	can_upgrade_reource_num = 0;
	can_upgrade_produce_num = 0;
	for (int i = 0; i < ROADCNT + 1; ++i) resource_area[i] = false;
	
	myresource = state->resource[ts19_flag].resource;
	mybdpoint = state->resource[ts19_flag].building_point;
	science_factor = 1 + state->age[ts19_flag] * AGE_INCREASE;
	

	for (int i = 0; i < BASE_SIZE + BD_RANGE_FROM_BASE; ++i)
		for (int j = 0; j < BASE_SIZE + BD_RANGE_FROM_BASE; ++j)
		{
			if (buildmap[i][j] == 0) buildmap[i][j] = 3;
		}
	
	for (int i = 0; i < state->building[ts19_flag].size(); ++i)
	{
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
	}

	for (int i = 0; i < state->building[1-ts19_flag].size(); ++i)
	{
		if (state->building[1-ts19_flag][i].building_type == __Base)
			continue;
		drawbuildingmap(trans(state->building[1-ts19_flag][i].pos.x), trans(state->building[1-ts19_flag][i].pos.y), 1);
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

	drawbuildingmap(pos.x, pos.y,0);
	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE];// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];// *science_factor;

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

	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE] * 0.5;// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT] * 0.5;// *science_factor;

#ifdef DEBUG

	file << "turn: " << state->turn << " type:" << building_type << " upgrade: " << unit_id << endl;

#endif // DEBUG
	
	return true;
}

int build_programmer(int n)
{
	Position buildingPos;
	int rest_num = n;
	bool flag;
	int mindis,dis;
	while (rest_num > 0)
	{
		mindis = 10000;
		flag = false;
		for (int cnt = 0; cnt < ROADCNT + 1; ++cnt)
		{
			for (int i = 0; i < saveArea[areasortid[cnt]].size(); ++i)
			{
				if (buildmap[saveArea[areasortid[cnt]][i].x][saveArea[areasortid[cnt]][i].y] != 3) continue;
				dis = enemy_base_dist(saveArea[areasortid[cnt]][i].x, saveArea[areasortid[cnt]][i].y);
				if (dis < mindis)
				{
					mindis = dis;
					flag = true;
					buildingPos = saveArea[areasortid[cnt]][i];
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
	
	Position *queue = new Position[MAP_SIZE*MAP_SIZE];
	bool *queueflag = new bool[MAP_SIZE*MAP_SIZE];
	int head, tail, cnt = 0,real_cnt=0;
	int tx[4] = { -1,0,1,0 }, ty[4] = { 0,-1,0,1 };
	while (rest_num > 0)
	{
		for (int i = 0; i < MAP_SIZE*MAP_SIZE; ++i) queueflag[i] = false;
		//memset((void*)queueflag, 0, sizeof(bool)*MAP_SIZE*MAP_SIZE);
		if (real_cnt == ROADCNT + 1) break;
		if (resource_area[areasortid[real_cnt]]) {++real_cnt; continue;}
		cnt = real_cnt;
		//if (areacount[areasortid[cnt]] == 0) continue;
		//BFS
		head = 0;
		tail = 0;
		for (int i = 0; i < saveArea[cnt].size(); ++i)
		{
			if (buildmap[saveArea[cnt][i].x][saveArea[cnt][i].y] == 2) continue;
			queue[tail++] = saveArea[cnt][i];
			queueflag[saveArea[cnt][i].x*MAP_SIZE + saveArea[cnt][i].y] = true;
		}
		while (head < tail)
		{
			for (int i = 0; i < 4; ++i)
			{
				if ((queue[head].x + tx[i] < 0) || (queue[head].x + tx[i] >= MAP_SIZE) || (queue[head].y + ty[i] < 0) || (queue[head].y + ty[i] >= MAP_SIZE))
					continue;
				if (queueflag[(queue[head].x + tx[i])*MAP_SIZE + queue[head].y + ty[i]]) continue;
				if (buildmap[queue[head].x + tx[i]][queue[head].y + ty[i]] == 3)
				{
					buildingPos = Position(queue[head].x + tx[i], queue[head].y + ty[i]);
					if (!myConstruct(Programmer, buildingPos)) return 0;
					else ++my_resource_num;
					--rest_num;
					delete queue;
					return rest_num;
				}
				queue[tail++] = Position(queue[head].x + tx[i], queue[head].y + ty[i]);
				queueflag[(queue[head].x + tx[i])*MAP_SIZE + queue[head].y + ty[i]] = true;
			}
			++head;
		}
		++real_cnt;
	}
	delete queue;
	delete queueflag;
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
	int rest_num = n;
	int mindis, roaddis, dis;
	Position pos,solider_pos;
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
					if (roaddis <= OriginalBuildingAttribute[building_type][ORIGINAL_RANGE]) continue;
					if ((mindismap[x][y] <= OriginalBuildingAttribute[building_type][ORIGINAL_RANGE]) ||
						(mindismap[x][y] > roaddis))
					{
						mindis = dis;
						roaddis = mindismap[x][y];
						pos = Position(x, y);
					}
				}
			}
		if (!flag) break;
		if (roaddis <= OriginalBuildingAttribute[building_type][ORIGINAL_RANGE])
		{
			//solider_pos = Position(trans(minposmap[pos.x][pos.y] / MAP_SIZE), trans(minposmap[pos.x][pos.y] % MAP_SIZE));
			solider_pos = Position(minposmap[pos.x][pos.y] / MAP_SIZE, minposmap[pos.x][pos.y] % MAP_SIZE);
			if (!myConstruct(building_type, pos, solider_pos)) break;
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

int upgrade_produce(int n)
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

int resource_strategy()
{
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, resource_num - my_resource_num);
	LOGN(bd_num);
	while (bd_num>0)
		bd_num = build_programmer(bd_num);
	bd_num = mymin(floor(2 * myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(2 * mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	bd_num = mymin(bd_num, can_upgrade_reource_num);
	upgrade_programmer(bd_num);
	return 0;
}

int produce_strategy()
{
	if (my_resource_num < resource_num) return 0;
	if (can_upgrade_reource_num > 0) return 0;

	//Construct produce building
	int bd_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag] + 1 - state->building[ts19_flag].size();
	BuildingType building_type;
	if (ts19_flag == 0)
	{
		building_type = Norton;
		if (state->age[ts19_flag] >= NETWORK)
			building_type = Berners_Lee;
	}
	else
	{
		building_type = Thevenin;
		if (state->age[ts19_flag] == NETWORK)
			building_type = Berners_Lee;
		if (state->age[ts19_flag] == AI)
			building_type = Tony_Stark;
	}
	while (bd_num > 0)
		bd_num = build_produce(bd_num, building_type);

	//Upgrade produce building
	bd_num = can_upgrade_produce_num;
	while (bd_num > 0)
		bd_num = upgrade_produce(bd_num);

}

void Setup()
{
	initBuildmap();
	int bd_num = 6;
	while (bd_num>0)
		bd_num = build_programmer(bd_num);
	return;
}

void excute()
{
	
	
	initBuildmap();
	
	// Stage 0 - Build to defense

	//resource_num = MAX_BD_NUM + MAX_BD_NUM_PLUS * state->age[ts19_flag];
	// Stage 1 - Get resource
	resource_num = resource_limit[state->age[ts19_flag]];
	resource_strategy();

	// Stage 2 - Create soldier to destroy building for producing
	// Stage 3 - Create soldier to destroy all building (I think it is useless)
	produce_strategy();
	// Stage 4 - Time for attack base
	
	// Stage 5 - Update Age
	if ((myresource > UPDATE_COST + UPDATE_COST_PLUS*state->age[ts19_flag]) && (state->age[ts19_flag]<AI))
	{
		updateAge();
		myresource -= UPDATE_COST + UPDATE_COST_PLUS*state->age[ts19_flag];
#ifdef DEBUG

		file << "turn: " << state->turn << " UPDATAEAGE" << endl;

#endif // DEBUG
	}


	return;
}

void f_player()
{
#ifdef DEBUG
	file << "turn: " << state->turn << endl;
	file << "building: " << state->building[ts19_flag].size() << endl;
	file << "resource: " << state->resource[ts19_flag].resource << endl;

#endif // DEBUG

	tic();
	if (state->turn == 0)
	{
		init();
		Setup();
	}
	else excute();
	toc();
};

