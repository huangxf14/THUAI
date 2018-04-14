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
#include <math.h>  

#ifdef DEBUG
#include <ctime>
#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
using std::cout;
using std::endl;
#endif // DEBUG



extern State* state;
extern std::vector<State* > all_state;
extern int** ts19_map;
extern bool ts19_flag;
int colormap[MAP_SIZE][MAP_SIZE] = { 0 };
int mindismap[MAP_SIZE][MAP_SIZE] = { 0 };
int minposmap[MAP_SIZE][MAP_SIZE] = { 0 };
int buildmap[MAP_SIZE][MAP_SIZE] = { 0 };//0 - useless;1 - road;2 - building&base;3 - useful;
int discount[41] = { 0 };
int ROADCNT = 0;
int ENEMY_EDGE = MAP_SIZE - BASE_SIZE;
int area_dis = 40;//the min distance for save area
int *minareadis;//min distance from enemybase of every area
int *areacount;//the cnt of allowed point of every area
int *areasortid;//sort area by the dis from enemybase(short to long)
std::vector<Position>* saveArea;
int game_stage = 0;//Change strategy arccording to stage
int myresource = 0, mybdpoint = 0;
int resource_num=MAX_BD_NUM,my_resource_num=0;//the number of resource building
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


int dist(Position p1, Position p2) {
	return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

int enemy_base_dist(int x, int y)
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

	std::ofstream file("log.txt",'w');
	file << "area_dis: " << area_dis << endl;
	for (int i = 0; i < ROADCNT + 1; ++i)
		file << "area" << i << " :(dis," << minareadis[i] << ") (cnt," << areacount[i] << ")" << endl;
	file.close();
	cout << "Output init" << endl;
#endif 

	delete queue;
	delete root;
	delete rootpos;
	delete flag;
	return;
}

void Setup()
{
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

	if (x >= 1)
	{
		if (y >= 1)
		{
			buildmap[x - 1][y - 1] = 2;
		}
		if (y + 1 < MAP_SIZE)
		{
			buildmap[x - 1][y + 1] = 2;
		}
	}
	if (x + 1 < MAP_SIZE)
	{
		if (y >= 1)
		{
			buildmap[x + 1][y - 1] = 2;
		}
		if (y + 1 < MAP_SIZE)
		{
			buildmap[x + 1][y + 1] = 2;
		}
	}

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
			resource_area[colormap[trans(state->building[ts19_flag][i].pos.x)][trans(state->building[ts19_flag][i].pos.y)] - ROADMAX] = true;
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

bool myConstruct(BuildingType building_type, Position pos, Position soldier_pos = Position(0, 0))
{
	if (myresource < OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE] * science_factor)
		return false;
	if (mybdpoint < OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT] * science_factor)
		return false;
	construct(building_type, Position(trans(pos.x),trans(pos.y)), Position(trans(soldier_pos.x),trans(soldier_pos.y)));

	drawbuildingmap(pos.x, pos.y,0);
	myresource -= OriginalBuildingAttribute[building_type][ORIGINAL_RESOURCE];// *science_factor;
	mybdpoint -= OriginalBuildingAttribute[building_type][ORIGINAL_BUILDING_POINT];// *science_factor;

#ifdef DEBUG
	std::ofstream file("log.txt", 'a');
	file << "turn: " << state->turn << " type:" << building_type << " pos: (" << pos.x << " ," <<pos.y<<") "<< endl;
	file.close();
#endif // DEBUG


	return true;
}

int build_programmer(int n)
{
	Position buildingPos;
	int rest_num = n;
	bool flag;
	int mindis,dis;
	LOGN(n);
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
			myConstruct(Programmer, buildingPos);
			--rest_num;
			break;
		}
		if (!flag) break;
		
	}

	Position *queue = new Position[MAP_SIZE*MAP_SIZE];
	bool *queueflag = new bool[MAP_SIZE*MAP_SIZE];
	int head, tail, cnt = 0;
	int tx[4] = { -1,0,1,0 }, ty[4] = { 0,-1,0,1 };
	while (rest_num > 0)
	{
		memset((void*)queueflag, 0, sizeof(bool)*MAP_SIZE*MAP_SIZE);
		if (cnt == ROADCNT + 1) break;
		if (resource_area[areasortid[cnt]]) {++cnt; continue;}
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
				if ((queue[head].x + tx[i] < 0) || (queue[head].x + tx[i] >= MAP_SIZE) || (queue[head].y + ty[i] < 0) || (queue[head].y + ty[i] > MAP_SIZE))
					continue;
				if (queueflag[(queue[head].x + tx[i])*MAP_SIZE + queue[head].y + ty[i]]) continue;
				if (buildmap[queue[head].x + tx[i]][queue[head].y + ty[i]] == 3)
				{
					buildingPos = Position(queue[head].x + tx[i], queue[head].y + ty[i]);
					myConstruct(Programmer, buildingPos);
					--rest_num;
					delete queue;
					return rest_num;
				}
				queue[tail++] = Position(queue[head].x + tx[i], queue[head].y + ty[i]);
				queueflag[(queue[head].x + tx[i])*MAP_SIZE + queue[head].y + ty[i]] = true;
			}
			++head;
		}
		++cnt;
	}
	delete queue;
	delete queueflag;
	return 0;
}

int resource_strategy()
{
	int bd_num = mymin(floor(myresource / OriginalBuildingAttribute[Programmer][ORIGINAL_RESOURCE]),
		floor(mybdpoint / OriginalBuildingAttribute[Programmer][ORIGINAL_BUILDING_POINT]));
	LOGN(bd_num);
	bd_num = mymin(bd_num, resource_num - my_resource_num);
	while (bd_num>0)
		bd_num = build_programmer(bd_num);
	return 0;
}

void excute()
{
	
	
	initBuildmap();
	
	// Stage 0 - Build to defense
	
	// Stage 1 - Get resource
	resource_strategy();

	// Stage 2 - Create soldier to destroy building for producing
	// Stage 3 - Create soldier to destroy all building (I think it is useless)
	// Stage 4 - Time for attack base
	// Stage 5 - 


	return;
}

void f_player()
{
	cout << "turn: " << state->turn << endl;
	cout << "building:" << state->building[ts19_flag].size() << endl;
	cout << "resource: " << state->resource[ts19_flag].resource << endl;
	tic();
	if (state->turn == 0)
	{
		init();
		Setup();
	}
	else excute();
	toc();
};

