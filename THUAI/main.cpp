#include"communication.h"
#include<thread>
#include<vector>
#include<iostream>
#include<windows.h>
//#include<ctime>
using namespace std;

extern	bool ts19_updageAge;
extern vector<command1> ts19_c1;
extern vector<command2> ts19_c2;
void f_player();

State* state = NULL;
State* _state=NULL;
vector<State* > all_state;
MyClient cilent;
int** ts19_map;
bool ts19_flag;
bool goon = true;
bool use = false;
HANDLE signal;


void Listen()
{
    State* t;
    while (goon)
    {
        State* s = cilent.recv_state();
        _state = s;
        all_state.push_back(_state);
        if(_state->winner!=2||_state->turn>=1000)
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
    ts19_map = cilent.map;
    ts19_flag = cilent.flag;
    int turn = 0;
    thread th_communication(Listen);
    WaitForSingleObject(signal, INFINITE);
    State* laststate = NULL;
    while (_state->turn < 1000)
    {
        if (_state->winner != 2)
            break;
        state=_state;
        f_player();
        if (!use)
            cilent.send_command(ts19_updageAge, ts19_c1, ts19_c2);
        ts19_updageAge = false;
        ts19_c1.clear();
        ts19_c2.clear();
        WaitForSingleObject(signal, INFINITE);
    }
    if (_state->winner == 1)
        cout << "Winner is 1" << endl;
    else if (_state->winner == 0)
        cout << "Winner is 0" << endl;
    else if (_state->winner == 2)
        cout << "Draw" << endl;
    goon = false;
    Sleep(1000);
    delete _state;
    th_communication.join();
}
