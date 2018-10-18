/*
 *   Author:      Erhard Wieser
 *   Students:
    Description:    Neuro Netwrok
*/
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

using namespace std;

// the number of input data's characteriscs
#define resolution 50
#define N	2	// the number of input neuros
#define Q	2	// the number of output neuros

#define MSE_limit  0.00006 //0.00003  0.000054   ---- > 16 -- >20  0.00005   0.00005
#define Save_epoch 1

// Normalize the input 0~1
#define img_row 640.0
#define img_col 480.0
// Normalize the output 0~1
#define L_shoulder_pitch -2.0857
#define H_shoulder_pitch 2.0857
#define L_shoulder_roll -0.3142
#define H_shoulder_roll 1.3265
// http://doc.aldebaran.com/2-1/family/robots/joints_robot.html

// gedit .bashr.
// press the button to hear IP
#define Field_size 5 // 5
#define alpha 0.5             // Learning rate

class CMAC{

public:
  CMAC();
  void load_data(); 
  void InitNeuro();
  void Feed_Forward();
  double string_to_double( const std::string& s );
  void Calculate_pos(double x, double y, double output_joint[]);
  void Test_network();
  void Save_weight_bias();
  void Load_weight_bias(); 

private:

  //int Compute_active_neurons (double a[][],);

  string data_file_location;
  string load_name;
  string load_path_name;
  string save_file_address;
  string save_path_name;


  vector< vector<int> > a_vec;
  vector< vector<double> > c_vec;
  
  
  double a_n[N];  // input x y   L1
  double x_L2[resolution][resolution];  // Weight 1,2,3... depend on the input, ex: image_x image_y
  double y_L2[resolution][resolution];  
  double x_L2_init[resolution][resolution]; 
  double y_L2_init[resolution][resolution]; 
  
  double c_n[Q]; // output x y L3
  
  int train_data_num;
  int RFpos[Field_size][N];
  //int RFpos[Field_size][N] = { {0, 3}, {1, 0}, {2, 2}, {3, 4}, {4, 1} }; // If I want to give the value in CMAC::CMAC() ? how
  int Position[Field_size][N];
  
};

CMAC::CMAC(){
	
  data_file_location = "Hands/";
  load_name ="TrainingData_150_v3";
  load_path_name = data_file_location + load_name;

  //int RFpos[Field_size][N] = { {0, 3}, {1, 0}, {2, 2}};


  int RFpos[Field_size][N] = { {0, 3}, {1, 0}, {2, 2}, {3, 4}, {4, 1} };
  //alpha = 0.5;
  //beta = 0.5;
}

double CMAC::string_to_double( const std::string& s ){
  std::istringstream i(s);
  double x;
  if (!(i >> x))
    return 0;
  return x;
}

void CMAC::load_data(){   // If I can not know the trainig data number, how can I create another function for with specifc arrary a[][len]

  InitNeuro();
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  int iter = 0;

  double w,k,q,z;


  string delimite_space =" ";
  std::string in_out_arr[N+Q]; // 2+2
  string s;
  int npos;
  vector<vector<int> > img_vec;
  vector<vector<double> > out_vec;

  // Write file
  ofstream file_epoch;
  string Train_epoch_mse = data_file_location + "Train_epoch_mse";
  file_epoch.open (Train_epoch_mse.c_str(), ios::out);

  if (myfile.is_open())
  {}else cout << "Unable to open file";
  
  //------------------------------------------------------Reminder
  while ( getline(myfile,line) )
  {
    line_num++;
  }
  cout << "Sample point:" << line_num << endl;
  train_data_num = line_num;


  double a_o[line_num][N];    // -----------------------change the size
  double c_o[line_num][Q];

  // Set the index to zero
  myfile.clear();
  myfile.seekg(0, ios::beg);

  while (getline(myfile,line))
  {
    for(int i=0; i<N+Q-1; i++ ){ // 4-1 = 3 space
      if(i==0){
        s = line;
      }else{
        s = in_out_arr[i];
      }
      npos = 0;
      in_out_arr[i]= s.substr (npos, s.find(delimite_space));
      npos+= in_out_arr[i].size() + delimite_space.size();
      in_out_arr[i+1] = s.substr (npos);
    }
    // Train Input  0 1
    for(int i=0; i<N; i++ ){
      a_o[iter][i] = string_to_double(in_out_arr[i]);
    }
    // Train Out  2 3
    for(int i=0; i<Q; i++ ){
      c_o[iter][i] = string_to_double(in_out_arr[N+i]);
      //c[iter][i] =stod(in_out_arr[characteristic_num+i]);   // This function only work at C++10
    }
    iter++;
  }
  myfile.close();


  // Load data
  for (int i= 0; i< train_data_num; i++){
    cout << i <<" ";
    for(int k=0; k<N; k++ ){
      cout << a_o[i][k] <<" ";
    }
    for(int k=0; k<Q; k++ ){
      cout << c_o[i][k] <<" ";
    }
    cout << endl;
  }
  //=========================================================Make sure the whole table get training
  int increase_num = 0;
  int x_640 = 0;
  int y_480 = 0;
  int ave_num = 5; // choose five for average

  double ave_distance[ave_num];
  int ave_index[ave_num];
  double temp_dis = 0;
  double x_dis, y_dis;

  double a_ave[increase_num][N]; // over flow
  double c_ave[increase_num][Q];
  double a[line_num + increase_num][N];
  double c[line_num + increase_num][Q];

  //-----------------    151        +  307059   =307210
  for (int i = 0; i < train_data_num + increase_num; i++)   // more 10      307210
  {
    if( i < train_data_num ){         // original point 150   0-149
      //cout << i << " ";
      for(int k=0; k<N; k++ ){
        a[i][k] = a_o[i][k];
        //cout << a[i][k] <<" ";
      }
      //cout << " || ";
      for(int k=0; k<Q; k++ ){
        c[i][k] = c_o[i][k];  //
        //cout << c[i][k] <<" ";
      }
      //cout << endl;
    }else{
      //cout <<"Now index:"  << i << endl;
      for(int k = 0; k<N; k++ ){        // increase point  automatically circle first one 150 +  307050  -150
        a[i][k] = a_ave[i-train_data_num][k];  // 150 - 150 = 0
        //cout << a[i][k] <<" ";
      }
      //cout << " || ";
      for(int k = 0; k<Q; k++ ){
        c[i][k] = c_ave[i-train_data_num][k];
        //cout << c[i][k] <<" ";
      }
      //cout << endl;
//              if(i >= 4400){
//                sleep(100);
//              }
    }
  }
  cout << "===========================before Normailz" << endl;

  cout << "Total training data: " << train_data_num + increase_num << " Increasing Data: " << increase_num <<endl;
  // 307210                                           // 307059     // 0-150 = 151 data


  //==============================Save  data into global vector
      for (int i = 0; i < train_data_num; i++)
      {
        vector<int> a_two;
        for(int k = 0; k < N; k++)
        {
          a_two.push_back(a[i][k]);
        }
        a_vec.push_back(a_two);
      }
  //==============================Save global vector End ===========



  //=================================================================Normalize start !!!!!!!!!!!!!!!!!!!!!!!!!

  train_data_num = train_data_num + increase_num;  // a lot of data   150    19188

  if (data_file_location != "XOR/"){
    cout << "=================== Normalize ==============" << endl;
    for (int i= 0; i< train_data_num; i++){

      //cout << i <<" ";

      for(int k = 0; k<N; k++ ){
        //cout << a[i][k] <<"---ori--- ";
        if(k==0){
          a[i][k] = a[i][k] / img_row;
        }else if(k==1){
          a[i][k] = a[i][k] / img_col;
        }
        //cout << a[i][k] <<" ";
      }
      for(int k=0; k<Q; k++ ){
        if(k==0){
          c[i][k] = ( c[i][k] - L_shoulder_pitch ) / ( H_shoulder_pitch - L_shoulder_pitch);
        }else if(k==1){
          c[i][k] = ( c[i][k] - L_shoulder_roll ) / ( H_shoulder_roll - L_shoulder_roll);
        }
        //cout << c[i][k] <<" ";
      }
      //cout << endl;

      if(i % 1000 == 0){
        cout << i <<" data normalizing" << endl;
      }

    }
    //=================================================================
    //  Write file for nornalize data
    //        ofstream file_norma;
    //        string norma = data_file_location + "Normalize_Data_150";
    //        file_norma.open (norma.c_str(), ios::out);
    //        for (int i= 0; i< train_data_num; i++){
    //          for(int k=0; k<N; k++ ){
    //            file_norma << a[i][k] <<" ";
    //          }
    //          for(int k=0; k<Q; k++ ){
    //            file_norma << c[i][k] <<" ";
    //          }
    //          file_norma << endl;
    //        }
  }

  //================================================================= Train

  int iter_index = 0;
  double mse = 0;
  double error= 0;
  int x_temp,y_temp;



  do{
    mse = 0;
    for(int i = 0; i < train_data_num; i++)
    {

//      if(i % 1000 == 0){
//        cout << i <<" training" << endl;
//      }

      // one time train a sample with two specific data
      for(int j = 0; j < N; j++)   // input data  image x y    N = 2
      {
        a_n[j] = a[i][j];   // a[num_data][input_specific]
      }
      //

      Feed_Forward();
      //cout << "iter_index:"<< iter_index << " Training Num: " << i << "  a_n[0]:" <<  a_n[0] << "  a_n[1]:" << a_n[1] << "  c[0]:" << c[i][0] <<"  c[1]:" << c[i][1] << endl;

      // Smooth Update
      int smooth_range = 5;
      int x_smo = 0;
      int y_smo = 0;

      // UPdate
      for(int r = 0; r < Field_size;  r++)  // 5
      {
        x_temp = Position[r][0];  // 0
        y_temp = Position[r][1];  // 1  -------------------for
        x_L2[x_temp][y_temp] += alpha * ( c[i][0] - c_n[0] ) / Field_size; // x table
        y_L2[x_temp][y_temp] += alpha * ( c[i][1] - c_n[1] ) / Field_size; // y table

      }
      error = 0;  // for each epoch
      //cout << "Ans " <<c[i][0] << " " << c_n[0] << " || " << c[i][1] << " " << c_n[1] <<endl;
      for(int k = 0; k < Q; k++)  // 0 1  = 2
      {
        error += pow ( (c[i][k]- c_n[k]) ,2);
      }
      mse += 0.5*error;
    }
    mse = mse / double(train_data_num*Q);

    // Save file
    if (iter_index%Save_epoch == 0){
      cout << "========	" << iter_index << "	iterations ===== MSE: "<< mse << endl;
      file_epoch <<  iter_index << " " << mse << endl;
    }
    iter_index++;

  }while (mse > MSE_limit);
  file_epoch.close();
  cout << "================= Success CMAC Train ================== " << endl;
  Save_weight_bias();
  

  
}


void CMAC::InitNeuro(){
	for(int i = 0; i < resolution; i++){
		for(int j = 0; j < resolution; j++){
			x_L2[i][j] = ((double) rand() / (RAND_MAX))*2-1;  // -1~1
			y_L2[i][j] = ((double) rand() / (RAND_MAX))*2-1;		
			//cout << "ij: "<< i << " "<< j << " " <<"x: " << x_L2[i][j]<< "y: " << y_L2[i][j] << endl;			
			x_L2_init[i][j] = x_L2[i][j];
			y_L2_init[i][j] = y_L2[i][j];
			
		}
	}
}

void CMAC::Feed_Forward(){
	
	// only for one xy sample 

	int input_index = 0; 
	int shift_amount = 0;
	int local_coord = 0;
	int coord = 0;
	int x_temp =0;
	int y_temp =0;
	
	for(int r = 0; r < Field_size; r++)
	{
		//int Coord[N];
		// ch = channel
		for(int ch =0;  ch < N; ch++ ) // N = 2
		{                 	
			input_index = round(  a_n[ch] * resolution );  // To get the int 
			// Protection ???			
			if(input_index > (resolution-Field_size)){
				input_index = resolution-Field_size ; 
			}			
			shift_amount = Field_size - input_index % Field_size;
			local_coord = ( shift_amount + RFpos[r][ch] ) % Field_size;
			coord = input_index + local_coord;
			//Coord[ch] = coord;
			Position[r][ch] = coord;	
		} 
	}	
	// Output to sum up all the weight value 
	c_n[0] = 0; 
	c_n[1] = 0;
	for(int r = 0; r < Field_size; r++)   // ~~  5 
	{	
		x_temp = Position[r][0];  // 0
		y_temp = Position[r][1];  // 1		
		//cout << " xy: " << x_temp << " " << y_temp;
		//cout << " va: " << x_L2[x_temp][y_temp]<< " " << y_L2[x_temp][y_temp];		
		c_n[0] += x_L2[x_temp][y_temp];      // global weight position            
		c_n[1] += y_L2[x_temp][y_temp];      // add up five numbers
	}
	//cout << " || " << a_n[0] << " " << a_n[1] << " " << c_n[0] << " " << c_n[1] << endl;
}




void CMAC::Calculate_pos(double x, double y, double output_joint[]){
	
	double a_xy[2];
	a_xy[0] = x ;
	a_xy[1] = y ;
	
 	double c_xy[Q];
    int input_index = 0; 
	int shift_amount = 0;
	int local_coord = 0;
	int coord = 0;
	int x_temp =0;
	int y_temp =0;
	
	int x_distance = 0;
	int y_distance = 0;
	int xy_distance = 0;
	int x_dis_cri = 100;
	int y_dis_cri = 100;
	
	int x_index = 0;
	int y_index = 0;
	int sample_index = 0;


  a_xy[0] = a_xy[0]/img_row;
  a_xy[1] = a_xy[1]/img_col;
 
 	
 	for(int r = 0; r < Field_size; r++)
	{
		for(int ch =0;  ch < N; ch++ ) // N = 2
		{                 
			input_index = round(  a_xy[ch] * resolution );  // To get the int 
			// Protection ???	over the range to calculate the bounding 	
			if(input_index > (resolution-Field_size)){
				input_index = resolution-Field_size ; 
			}			
			shift_amount = Field_size - input_index % Field_size;
			local_coord = ( shift_amount + RFpos[r][ch] ) % Field_size;
			coord = input_index + local_coord;
			Position[r][ch] = coord;
		} 
	}
	
	// Output to sum up all the weight value 
	c_xy[0] = 0; 
	c_xy[1] = 0;
	
	for(int r = 0; r < Field_size; r++)   // ~~  5 
	{	
		x_temp = Position[r][0];  // 0
		y_temp = Position[r][1];  // 1				
		c_xy[0] += x_L2[x_temp][y_temp];      // global weight position            
		c_xy[1] += y_L2[x_temp][y_temp];      // add up five numbers
	}	
	
    output_joint[0] = c_xy[0] * ( H_shoulder_pitch - L_shoulder_pitch) + L_shoulder_pitch;
  	output_joint[1] = c_xy[1] * ( H_shoulder_roll - L_shoulder_roll) + L_shoulder_roll;

  
}


void CMAC::Save_weight_bias(){
  // Write file
  ofstream Write_File;

  string loc_v = data_file_location + "x_L2.txt";
  Write_File.open (loc_v.c_str(), ios::out);
  
  //cout << "x_L2 ===========" << endl; 
	for(int k = 0; k < resolution; k++){                  // N P
		for(int j = 0; j < resolution; j++){  		
			Write_File << x_L2[k][j] << " ";
			//cout <<  x_L2[k][j] << " ";
		}
	}
  Write_File.close();
  
  string loc_v_bias = data_file_location + "y_L2.txt";
  Write_File.open (loc_v_bias.c_str(), ios::out);
  
  //cout << "y_L2 ===========" << endl; 
	for(int k = 0; k < resolution; k++){                  // N P
		for(int j = 0; j < resolution; j++){  		
			Write_File << y_L2[k][j] << " ";
			//cout <<  y_L2[k][j] << " ";
		}
	}
  Write_File.close();
  
}

void CMAC::Load_weight_bias(){
	// Read file
  ifstream Read_File;
  string loc_v = data_file_location + "x_L2.txt";
  Read_File.open(loc_v.c_str());
  for(int k = 0; k < resolution; k++){              
    for(int j = 0; j < resolution; j++){
      Read_File >> x_L2[k][j];
    }
  }
  Read_File.close();

  string loc_w = data_file_location + "y_L2.txt";
  Read_File.open(loc_w.c_str());
  for(int j = 0; j < resolution; j++){       
    for(int h = 0; h < resolution; h++){
      Read_File >> y_L2[j][h];
    }
  }
  Read_File.close();
  cout << "Load data successfully !!!" << endl;
}



void CMAC::Test_network(){
	
	//check which weight is not 
	
	cout << "======================= x_L2 table ================= " <<endl;
	
	// try to find two table value 
 	for(int k = 0; k < resolution; k++){               
	    for(int j = 0; j < resolution; j++){	
	    	if(x_L2[k][j] == x_L2_init[k][j]){
	    		cout<< "1 "; // the same 
			}else{
				cout<< "x "; 
			}
	    }
	    cout << endl;
  	}
	
	cout << "======================= y_L2 table ================= " <<endl;
	
	for(int j = 0; j < resolution; j++){      
	    for(int h = 0; h < resolution; h++){
	    	if(y_L2[j][h] == y_L2_init[j][h]){
          cout<< "0 "; // the same
			}else{
				cout<< "x "; 
			}
	    }
	    cout << endl;
  	}

  //=========================================================== Orignial Data
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  double w,k,q,z;
  int iter = 0;

  string delimite_space =" ";
  std::string in_out_arr[N+Q]; // 2+2
  string s;
  int npos;
  double c_test[2];


  if ( myfile.is_open() ){
    while ( getline(myfile,line) )
    {
      line_num++;
    }
    cout << "=================Start Test Network sample points:" << line_num << endl;
    train_data_num = line_num;

    double a[line_num][N];
    double c[line_num][Q];

    // Set the index to zero
    myfile.clear();
    myfile.seekg(0, ios::beg);

    while (getline(myfile,line))
    {
      for(int i=0; i<N+Q-1; i++ ){ // 4-1 = 3 space
        if(i==0){
          s = line;
        }else{
          s = in_out_arr[i];
        }
        npos = 0;
        in_out_arr[i]= s.substr (npos, s.find(delimite_space));
        npos+= in_out_arr[i].size() + delimite_space.size();
        in_out_arr[i+1] = s.substr (npos);
      }
      // Train Input  0 1
      for(int i=0; i<N; i++ ){
        a[iter][i] = string_to_double(in_out_arr[i]);
      }
      // Train Out  2 3
      for(int i=0; i<Q; i++ ){
        c[iter][i] = string_to_double(in_out_arr[N+i]);
      }
      iter++;
    }
    // Test data
    for (int i= 0; i< train_data_num; i++){
      cout << i <<" ";
      for(int k=0; k<N; k++ ){
        cout << a[i][k] <<" ";
      }
      Calculate_pos(a[i][0], a[i][1], c_test);
      for(int k=0; k<Q; k++ ){
        cout << c[i][k] <<" ";
      }
      cout <<"||";
      for(int k=0; k<Q; k++ ){
        cout << c_test[k] <<" ";
      }
      cout <<"Err:";
      for(int k=0; k<Q; k++ ){
        cout <<  c[i][k]-c_test[k] <<" ";
      }
      cout << endl;
    }
  }
  else{
    cout << "Can not find the test file !" << endl;
  }
  //=========================================================== whole Data


}







