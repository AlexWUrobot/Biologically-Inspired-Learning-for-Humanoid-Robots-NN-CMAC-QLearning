using namespace std;
#include <iostream>
#include <vector>
#include <fstream>

// DOne by Hajer Chebil, Philipp Jakovleski, Chao Dong, Siyuan Li, Li-fan Wu
const int VisionStates =5;
const int LegStates = 5;
const int Actions= 3;
const int Goals = 5;

//Create Q matrix

class RL
{
private:



  // Q (states , actions , GK)
  vector < vector <vector<double> > >  Q =vector< vector <vector<double> > >(5, vector<vector <double> >(3, vector<double>(5)));
  // N (states , actions , GK)
  vector< vector <vector<int> > > N =  vector< vector <vector<int> > >(5, vector<vector <int> >(3, vector<int>(5)));
  // P (states , actions , GK)
  vector< vector <vector<double> > > P = vector< vector <vector<double> > >(5, vector<vector <double> >(3, vector<double>(5)));

  float GK_pose[5];
  double N_sum;

public:

    RL();

    int Action;
    int maxAction;
    int LegState;
    int LegStateNew;
    int GK;

    string goal_Q,data_file_location;
    string filetype;
    int action_step;


    //void performAction_exploration(int VS, int LS);
    void performAction(int GKP, int LS);

    vector < vector <vector<double> > > updateQ(int LS, int A);

//  void updateN(int LS, int A);
    void bestAction(int LS);

    void P_Calculation();
    void N_Calculation(int LS, int A);
    void initP();
    void initN();
    void initQN(int GKP);
    void saveQ();
    void saveN();
    void readQ();
    void readN();



    void getGoalKeeperPosition();
    int getAction();
    int receiveReward();
};


RL::RL(){
    goal_Q = "_0_";
    action_step = 0;
    data_file_location = "Q_table/";
    filetype = ".txt";
}


// Not necessary here, we process the goal keeper's position in the central_node
void RL::getGoalKeeperPosition(){

  // Get the 5 initial 5 postions for goal keeper // try to get messagr from vision_node
//  GK_pose[0]=-0.251083;
//  GK_pose[1]=-0.0800439;
//  GK_pose[2]=0.0909951;
//  GK_pose[3]=0.262034;
//  GK_pose[4]=0.433073;

}

// Initial Q Matrix
void RL::initQN(int GKP){
    GK=GKP;
    //Initialize Q and N___________//
    for(int i = 0;i<LegStates;i++){
        for(int j = 0; j<Actions;j++){
          for(int k = 0; k<Goals; k++)
                 Q[i][j][k] = 0;

        }
    }
}

//Initial N Matrix, the number for motion pair <state action next_state>
void RL::initN(){
    N_sum = 0;
    for(int i = 0;i<5;i++){
        for(int j = 0;j<3;j++){
            for(int k = 0;k<5;k++){
                N[i][j][k] = 0;
            }
        }
    }
}


// Initial P Matrix
void RL::initP(){
    for(int i = 0;i<LegStates;i++){
        for(int j = 0;j<Actions;j++){
            for(int k = 0;k<LegStates;k++){
                P[i][j][k] = 0;
            }
        }
    }
}

// Receive reward from keyboard
int RL::receiveReward(){

//  Protect
  int R;
  char sign;
  string input;

  char tryagain = 'y';
  do{
    cout << "please enter the Reward for the performed Action: ";
    getline(cin,input);
    if(input=="-"){
      cout << "you type the minus !! %s " << input << endl;
      R = -666;
    }else{
      R = atoi(input.c_str());
    }

    if(R== -1 || R== 1 || R== 0 || R== -5 || R==5 || R==-20 || R==20 || R==10 || R==-10){
      //cout << "You cin the reward value is " << R << endl;
      tryagain = 'n';
    }else{
      cout << "The reward never show " << R <<endl;
      cout << "Would you like to give the value again: ";
      cin >> tryagain;
    }
  }while(tryagain =='y');


  //R = (int)(sign);
  cout << "Reward : "<<R<<endl;
  return R;

//    int R;
//    cout << "please enter the Reward for the performed Action: ";
//    cin >> R;
//    cout << "Reward : "<<R<<endl;
//    return R;
}

// Calculate the number for each action pair
void RL::N_Calculation(int LS, int A){
    //First dimension: Start state, second: Action, 3rd: New state S'
    N_sum ++;
    N[LS][A][GK] +=1;
}

// Calculate the P Matrix according to N Matrix
void RL::P_Calculation(){
  for(int i = 0; i < LegStates; i++){
      for(int j = 0; j < Actions; j++){

          P[i][j][GK] = 1.0*N[i][j][GK]/N_sum;

          cout<<"P"<<P[i][j][GK]<<endl;

      }
  }
}





//Input the state, depending on the state, choose best action

void RL::bestAction(int LS){
    maxAction = 0;
    int compare = Q[LS][0][GK];
    for(int i = 0;i<3;i++){
        if(Q[LS][i][GK] > compare){
            compare = Q[LS][i][GK];
            maxAction = i;
        }
    }
}

// Update Q table
vector < vector <vector<double> > > RL::updateQ(int LS, int A){

    int reward = receiveReward();
    double gamma = 0.5;
    int sum =0;
    for(int state_next = 0; state_next < LegStates; state_next++){
        bestAction(state_next);
        sum += P[state_next][A][GK]*Q[state_next][maxAction][GK];
    }
    Q[LS][A][GK] = reward + gamma*sum;
    return Q;
}




// Exploitation and Exploration and get action for current state
void RL::performAction(int GKP, int LS){
    GK=GKP;
    bestAction(LS);
  int Max_Reward = 20;
    if (Q[LS][maxAction][GK]>0.4*Max_Reward){  //exploitation
      cout << "Perform exploitation: " << endl;
      Action = 0;
      int compare = Q[LS][0][GK];
      for(int i=0;i<3;i++){
          if(Q[LS][i][GK] > Q[LS][0][GK]){
              compare = Q[LS][i][GK] ;
              Action =i;
          }
      }
    }else{  //exploration
      cout << "Perform Exploration: " << endl;
        Action =3.0*rand()/(RAND_MAX);
    }

    // Define action 0: leg in, 1: leg out, 2: Kick
    // Change State after either action 0 or 1 is performed,

    //Also, limit the possible actions for the edges> 0 max and -0.4 min are the boundaries. --0.2 is the middle
    //Improve edge behaviour

    if( LS == 4 && Action == 1) {  //-1
        LegStateNew =LegState ;
    }else if(LS == 0 && Action == 0){
      LegStateNew =LegState ;  //+1
    }else if( Action == 1){
        LegStateNew=LegState+1;
    }else if( Action == 0){
        LegStateNew=LegState-1;
    }else{ // Action ==2, it means kicking
        LegStateNew=LegState;
    }

}

int RL::getAction(){

   return Action;
}
/*
int RL::getLegstate(){

   return LegState;
}
*/


// Save the Q Matrix
ofstream myfile;

void RL::saveQ(){

    string new_name = "Q";
    string string_step = std::to_string(action_step);

    new_name = data_file_location + new_name + goal_Q + string_step + filetype;
    myfile.open ( new_name.c_str() );

    //myfile.open ("Q.txt");
    for(int k = 0;k<5;k++){
        for(int i = 0;i<5;i++){
          for(int j=0; j<3;j++){
            myfile << Q[i][j][k] << " ";
          }
        }
        myfile << endl;
    }
    myfile.close();

}


ofstream myfile2;
ofstream myfile4;

// Save the N and P matrix
void RL::saveN(){

    myfile2.open ("N.txt");

    for(int k = 0;k<5;k++){
        for(int i = 0;i<5;i++){
            for(int j = 0;j<3;j++){
                myfile2 << N[i][j][k] << " ";
            }
        }
        myfile2 << endl;
    }
    myfile2.close();

    myfile4.open("P.txt");
    for(int k = 0;k<5;k++){
        for(int i = 0;i<5;i++){
            for(int j = 0;j<3;j++){
                myfile4 << P[i][j][k] << " ";
            }
        }
        myfile4 << endl;

    }
    myfile4.close();
}



ifstream myfile3;
// Read the Q Matrix
void RL::readQ(){

    myfile3.open ("Q.txt");
    for(int k = 0;k<5;k++){
        for(int i = 0;i<5;i++){
          for(int j=0; j<3;j++){
            myfile3 >> Q[i][j][k];
          }
        }
    }
    myfile3.close();
}

// Read the P Matrix
void RL::readN(){
    myfile3.open ("P.txt");

    for(int k = 0;k<5;k++){
        for(int i = 0;i<5;i++){
            for(int j = 0;j<3;j++){
                myfile3 >> P[i][j][k] ;
            }
        }

    }
    myfile3.close();
}

