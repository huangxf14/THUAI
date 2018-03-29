#include"communication.h"
#include<thread>
#include<vector>
#include<iostream>
#include<windows.h>
//#include<ctime>
using namespace std;

extern	bool _updateAge;
extern vector<command1> c1;
extern vector<command2> c2;
void f_player();

State* state = NULL;
vector<State* > all_state;
MyClient cilent;
int** map;
bool flag;
bool goon = true;
bool use = false;
HANDLE signal;


void Listen()
{
    State* t;
    while (goon)
    {
        State* s = cilent.recv_state();
        state = s;
        all_state.push_back(state);
        if(state->winner!=2||state->turn>=1000)
            break;
        /*t=state;
        state=s;
        delete t;*/
        ReleaseSemaphore(signal, 1, NULL);

    }
}

int main()
{
    signal = CreateSemaphore(NULL, 0, 1, NULL);
    cilent.start_connection();
    map = cilent.map;
    flag = cilent.flag;
    int turn = 0;
    thread th_communication(Listen);
    WaitForSingleObject(signal, INFINITE);
    State* laststate = NULL;
    while (state->turn < 1000)
    {
        if (state->winner != 2)
            break;
        f_player();
        if (!use)
            cilent.send_command(_updateAge, c1, c2);
        _updateAge = false;
        c1.clear();
        c2.clear();
        WaitForSingleObject(signal, INFINITE);
    }
    if (state->winner == 1)
        cout << "Winner is 1" << endl;
    else if (state->winner == 0)
        cout << "Winner is 0" << endl;
    else if (state->winner == 2)
        cout << "Draw" << endl;
    goon = false;
    Sleep(1000);
    delete state;
    th_communication.join();
}
