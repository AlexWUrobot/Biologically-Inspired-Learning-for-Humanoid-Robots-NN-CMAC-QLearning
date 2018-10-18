/*
 *   Author:      Erhard Wieser
 *   Students:    Wu Li-Fan, Jakovleski Philipp, Fehr Axel
    Description:    Neuro Netwrok
*/
#include <iostream>
#include <fstream>
#include <string>

#include <sstream>
using namespace std;

#include <cmath>

// the number of input data's characteriscs
#define characteristic_num 2
#define output_num 2
// Normalize the input 0~1
#define img_row 640.0
#define img_col 480.0
// Normalize the output 0~1
#define L_shoulder_pitch -2.0857
#define H_shoulder_pitch 2.0857
#define L_shoulder_roll -0.3142
#define H_shoulder_roll 1.3265
// http://doc.aldebaran.com/2-1/family/robots/joints_robot.html

#define N	2	// the number of input neuros 
#define P	8	// the number of hidden neuros
#define Q	2	// the number of output neuros
#define MSE_limit 0.00003
#define Save_epoch 100

class NN{

public:
  NN();
  void load_data();
  void InitNeuro();
  
  double Sigmoid (double x); 
  
  void Feed_Forward();
  void load_create_data();
  void Calculate_pos(double x, double y, double output_joint[]);

  void Save_weight_bias();
  void Load_weight_bias(); 
  double string_to_double( const std::string& s );
  
  void Test_network();

private:
  string data_file_location;
  string load_name;
  string load_path_name;
  string save_file_address;
  string save_path_name;

  // Train Input
  int train_data_num;

  // Learning rate 
  double alpha;
  double beta;
  
  double a_n[N];
  double b_n[P];
  double c_n[Q];
  
  double d_n[Q];
  double e_n[P];
  
  double V[N][P];
  double W[P][Q];
  double V_bias[P]; 
  double W_bias[Q];

  double dV[N][P];
  double dW[P][Q];
  double dV_bias[P];
  double dW_bias[Q];
};


NN::NN(){
  //data_file_location = "/home/bilhr_c/catkin_ws/src/bilhr/src/";
  data_file_location = "Hands/";    //   XOR/  and Hands/                XOR/ change the Node 2 2 1 Q,  comment the normalize
  load_name ="TrainingData_raw";

  load_path_name = data_file_location + load_name;
  alpha = 0.5;
  beta = 0.5;
}

double NN::string_to_double( const std::string& s ){
  std::istringstream i(s);
  double x;
  if (!(i >> x))
    return 0;
  return x;
}

void NN::load_create_data(){
  // Loading data
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  string in_out_arr[characteristic_num+output_num]; // 2+2
  // Write file
  ofstream myfile2;
  myfile2.open ("TrainingData_raw", ios::out);

  if (myfile.is_open())
  {
    while ( getline (myfile,line))
    {
      //cout << line << endl;
      line_num++;
    }
    cout << "Sample point:" << line_num << endl;
    train_data_num = line_num;

    // Set the index to zero
    myfile.clear();
    myfile.seekg(0, ios::beg);

    // Declare the the number of line
    string str_line[line_num];

    string delimiter[3];

    delimiter[0] = "Shoulder Position: ";
    delimiter[1] = " Pixel Position";
    delimiter[2] = " ";

    int delimiter_element_num = sizeof(delimiter)/sizeof(delimiter[0]);

    string token;
    string s;
    size_t pos = 0;

    for (int i = 0; i < line_num ; i ++)
    {
      getline( myfile, str_line[i]);
      s = str_line[i];
      //cout << str_line[i] << endl;
      int npos = 0;  // a
      int k = 0;
      // Remove 0
      pos = s.find(delimiter[0]);
      pos+= delimiter[0].size();
      s = s.substr(pos);
      //http://www.cplusplus.com/reference/string/string/substr/

      // Remove 1
      in_out_arr[0]= s.substr (npos, s.find(delimiter[1]));
      npos+= in_out_arr[0].size() + delimiter[1].size();
      in_out_arr[2] = s.substr (npos);

      // Remove 2
      s = in_out_arr[0];
      npos = 0;
      in_out_arr[0]= s.substr (npos, s.find(delimiter[2]));
      npos+= in_out_arr[0].size() + delimiter[2].size();
      in_out_arr[1] = s.substr (npos);

      cout << in_out_arr[0]  <<" "<< in_out_arr[1] <<" "<< in_out_arr[2] <<" " << in_out_arr[3]<< endl;
      if (myfile2.is_open()) {
        //myfile2 << in_out_arr[0] <<" "<< in_out_arr[1] <<" "<< in_out_arr[2] <<" "<< in_out_arr[3]<< endl;
        myfile2 << in_out_arr[2] <<" "<< in_out_arr[3] <<""<< in_out_arr[0] <<" "<< in_out_arr[1]<< endl;
      }else{
        cout<<"====================ERROR==========================";
      }
    }
    myfile.close();
    myfile2.close();
  }
  else cout << "Unable to open file";
}

void NN::load_data(){
  
  InitNeuro();
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  double w,k,q,z;
  int iter = 0;

  string delimite_space =" ";
  std::string in_out_arr[characteristic_num+output_num]; // 2+2
  string s;
  int npos;
  
  // Write file
  ofstream file_epoch;
  string Train_epoch_mse = data_file_location + "Train_epoch_mse";
  file_epoch.open (Train_epoch_mse.c_str(), ios::out);


  if (myfile.is_open())
  {
    while ( getline(myfile,line) )
    {
      line_num++;
    }
    cout << "Sample point:" << line_num << endl;
    train_data_num = line_num;
    
    double a[line_num][characteristic_num];
    double c[line_num][output_num];
    
    // Set the index to zero
    myfile.clear();
    myfile.seekg(0, ios::beg);

    while (getline(myfile,line))
    {
      for(int i=0; i<characteristic_num+output_num-1; i++ ){ // 4-1 = 3 space
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
      for(int i=0; i<characteristic_num; i++ ){
        a[iter][i] = string_to_double(in_out_arr[i]);
      }
      // Train Out  2 3
      for(int i=0; i<output_num; i++ ){
        c[iter][i] = string_to_double(in_out_arr[characteristic_num+i]);
        //c[iter][i] =stod(in_out_arr[characteristic_num+i]);   // This function only work at C++10
      }
      iter++;
    }
    myfile.close();


    // Load data
    for (int i= 0; i< train_data_num; i++){

      cout << i <<" ";
      for(int k=0; k<characteristic_num; k++ ){
        cout << a[i][k] <<" ";
      }
      for(int k=0; k<output_num; k++ ){
        cout << c[i][k] <<" ";
      }
      cout << endl;
    }
    if (data_file_location != "XOR/"){
      cout << "=================== Normalize ==============" << endl;
      for (int i= 0; i< train_data_num; i++){
        cout << i <<" ";
        for(int k=0; k<characteristic_num; k++ ){

          if(k==0){
            a[i][k] = a[i][k] / img_row;
          }else if(k==1){
            a[i][k] = a[i][k] / img_col;
          }

          cout << a[i][k] <<" ";
        }
        for(int k=0; k<output_num; k++ ){

          if(k==0){
            c[i][k] = ( c[i][k] - L_shoulder_pitch ) / ( H_shoulder_pitch - L_shoulder_pitch);
          }else if(k==1){
            c[i][k] = ( c[i][k] - L_shoulder_roll ) / ( H_shoulder_roll - L_shoulder_roll);
          }
          cout << c[i][k] <<" ";
        }
        cout << endl;
      }
      //  Write file for nornalize data
      //  ofstream file_norma;
      //  string norma = data_file_location + "Normalize_Data";

      //  file_norma.open (norma.c_str(), ios::out);

      //  for (int i= 0; i< train_data_num; i++){

      //    for(int k=0; k<characteristic_num; k++ ){
      //      file_norma << a[i][k] <<" ";
      //    }
      //    for(int k=0; k<output_num; k++ ){
      //      file_norma << c[i][k] <<" ";
      //    }
      //    file_norma << endl;
      //  }
    }

    cout << "=================== Train ================== " << endl;

    int iter_index = 0;
    double mse = 0;
    double error = 0;

    do{
      mse = 0;

      // Set all dW and dV zero every epoch
      for(int j = 0; j < P; j++){
        for(int h = 0; h < Q; h++){
          dW[j][h] = 0;
          dW_bias[h] = 0;
        }
      }
      for(int k = 0; k < N; k++){
        for(int j = 0; j < P; j++){
          dV[k][j] = 0;
          dV_bias[j] = 0;
        }
      }

      for(int i = 0; i < train_data_num; i++){
        for(int k = 0; k < N; k++){
          a_n[k] = a[i][k];
        }
        // Forward   Set: b_n c_n
        Feed_Forward();
        // Renew Det Set: d_n e_n
        for(int h = 0; h < Q ; h++){
          d_n[h] = c_n[h] * ( 1 - c_n[h] ) * ( c[i][h]- c_n[h] );
        }

        for(int j = 0; j < P; j++){
          double Wd_temp = 0;
          for(int h = 0; h < Q; h++){
            Wd_temp = Wd_temp + W[j][h]*d_n[h];
          }
          e_n[j] = b_n[j] * ( 1 - b_n[j] ) * Wd_temp;
        }
        // Change Weight
        for(int j = 0; j < P; j++){
          for(int h = 0; h < Q; h++){
            dW[j][h] += alpha * b_n[j]* d_n[h];
            dW_bias[h] += alpha * d_n[h];
          }
        }

        for(int k = 0; k < N; k++){
          for(int j = 0; j < P; j++){
            dV[k][j] += beta * a_n[k] * e_n[j];
            dV_bias[j] += beta * e_n[j];
          }
        }
        // Calculate MSE
        error = 0;   // Sum up error of every output
        for(int h = 0; h < Q; h++){
          error += pow ( (c[i][h]- c_n[h]) ,2);
        }
        mse += 0.5*error;
      }
      mse = mse / double(train_data_num*Q);

      // Renew Weight and Bias
      for(int j = 0; j < P; j++){
        for(int h = 0; h < Q; h++){
          W[j][h] += dW[j][h] / double(train_data_num);
          W_bias[h] += dW_bias[h] / double(train_data_num);
        }
      }
      for(int k = 0; k < N; k++){
        for(int j = 0; j < P; j++){
          V[k][j] += dV[k][j] / double(train_data_num);
          V_bias[j] += dV_bias[j] / double(train_data_num);
        }
      }
      // Save file
      if (iter_index%Save_epoch == 0){
        cout << "========	" << iter_index << "	iterations ===== MSE: "<< mse << endl;
        file_epoch <<  iter_index << " " << mse << endl;
      }
      iter_index++;
    }while (mse > MSE_limit);
    file_epoch.close();
    cout << "================= Success Train ================== " << endl;
  }else cout << "Unable to open file";
  Save_weight_bias();
}

void NN::InitNeuro(){
  // Initial the weight and bias of the neuro network
  for(int i=0; i<N ; i++){
    for(int k=0; k<P ; k++){
      V[i][k] = ((double) rand() / (RAND_MAX))*2-1;  // -1~1
      V_bias[k] = ((double) rand() / (RAND_MAX))*2-1;
    }
  }
  for(int i=0; i<P ; i++){
    for(int k=0; k<Q ; k++){
      W[i][k] = ((double) rand() / (RAND_MAX))*2-1;
      W_bias[k] = ((double) rand() / (RAND_MAX))*2-1;
    }
  }
}

double NN::Sigmoid(double x){
	return 1.0/(1.0+exp(-x)); 
}

void NN::Feed_Forward(){
  // the feed forward equation for the neuro network
	for (int i = 0; i < P; i++){
		double b_temp = 0;
		for (int k = 0; k < N; k++){   
			b_temp = b_temp + a_n[k]*V[k][i];
		}
		b_n[i] = Sigmoid ( b_temp + V_bias[i] );
	}
	
	for (int i = 0; i < Q; i++){
		double c_temp = 0;
		for (int k = 0; k < P; k++){
			c_temp = c_temp + b_n[k]*W[k][i];
		}
		c_n[i] = Sigmoid ( c_temp + W_bias[i]);	
	}	
}

void NN::Calculate_pos(double x, double y, double output_joint[]){
  // Input the image pixel point and publish the pitch and roll angle's of the shoulder
	double a_xy[2];

  if (data_file_location != "XOR/"){
    a_xy[0] = x / img_row;
    a_xy[1] = y / img_col;
  }else{
    a_xy[0] = x;
    a_xy[1] = y;
  }

  double b_xy[P];
  double c_xy[Q];

  for (int i = 0; i < P; i++){
    double b_temp = 0;
    for (int k = 0; k < N; k++){
      b_temp = b_temp + a_xy[k]*V[k][i];
    }
    b_xy[i] = Sigmoid ( b_temp + V_bias[i] );
  }

  for (int i = 0; i < Q; i++){
    double c_temp = 0;
    for (int k = 0; k < P; k++){
      c_temp = c_temp + b_xy[k]*W[k][i];
    }
    c_xy[i] = Sigmoid ( c_temp + W_bias[i]);
  }

  if (data_file_location != "XOR/"){
    output_joint[0] = c_xy[0] * ( H_shoulder_pitch - L_shoulder_pitch) + L_shoulder_pitch;
    output_joint[1] = c_xy[1] * ( H_shoulder_roll - L_shoulder_roll) + L_shoulder_roll;
  }else{
    output_joint[0] = c_xy[0];
    output_joint[1] = c_xy[1];
  }
}



void NN::Test_network(){
  // For test whether the networks work is right
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  double w,k,q,z;
  int iter = 0;

  string delimite_space =" ";
  std::string in_out_arr[characteristic_num+output_num];
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

    double a[line_num][characteristic_num];
    double c[line_num][output_num];

    // Set the index to zero
    myfile.clear();
    myfile.seekg(0, ios::beg);

    while (getline(myfile,line))
    {
      for(int i=0; i<characteristic_num+output_num-1; i++ ){
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
      // Train Input
      for(int i=0; i<characteristic_num; i++ ){
        a[iter][i] = string_to_double(in_out_arr[i]);
      }
      // Train Out
      for(int i=0; i<output_num; i++ ){
        c[iter][i] = string_to_double(in_out_arr[characteristic_num+i]);
      }
      iter++;
    }
    // Test data
    for (int i= 0; i< train_data_num; i++){
      cout << i <<" ";
      for(int k=0; k<characteristic_num; k++ ){
        cout << a[i][k] <<" ";
      }
      Calculate_pos(a[i][0], a[i][1], c_test);
      for(int k=0; k<output_num; k++ ){
        cout << c[i][k] <<" ";
      }
      cout <<"||";
      for(int k=0; k<output_num; k++ ){
        cout << c_test[k] <<" ";
      }
      cout <<"Err:";
      for(int k=0; k<output_num; k++ ){
        cout <<  c[i][k]-c_test[k] <<" ";
      }
      cout << endl;
    }
  }
  else{
    cout << "Can not find the test file !" << endl;
  }
}


void NN::Save_weight_bias(){
  // Write file
  ofstream Write_File;

  string loc_v = data_file_location + "V.txt";
  Write_File.open (loc_v.c_str(), ios::out);
  cout << "V ===========" << endl;
  for(int k = 0; k < N; k++){
    for(int j = 0; j < P; j++){
      Write_File << V[k][j] << " ";
      cout <<  V[k][j] << " ";
    }
  }
  Write_File.close();

  cout<<endl;
  string loc_v_bias = data_file_location + "V_bias.txt";
  Write_File.open (loc_v_bias.c_str(), ios::out);
  cout << "V_bias ===========" << endl;
  for(int j = 0; j < P; j++){
    Write_File << V_bias[j] << " ";
    cout <<  V_bias[j] << " ";
  }
  Write_File.close();

  cout<<endl;
  string loc_w = data_file_location + "W.txt";
  Write_File.open (loc_w.c_str(), ios::out);
  cout << "W ===========" << endl;
  for(int j = 0; j < P; j++){
    for(int h = 0; h < Q; h++){
      Write_File << W[j][h] << " ";
      cout <<  W[j][h] << " ";
    }
  }
  Write_File.close();

  cout<<endl;
  string loc_w_bias = data_file_location + "W_bias.txt";
  Write_File.open (loc_w_bias.c_str(), ios::out);
  cout << "W_bias ===========" << endl;
  for(int h = 0; h < Q; h++){
    Write_File << W_bias[h] << " ";
    cout <<  W_bias[h] << " ";
  }
  Write_File.close();
}

void NN::Load_weight_bias(){
  // Read file
  ifstream Read_File;

  string loc_v = data_file_location + "V.txt";
  Read_File.open(loc_v.c_str());
  for(int k = 0; k < N; k++){
    for(int j = 0; j < P; j++){
      Read_File >> V[k][j];
    }
  }
  Read_File.close();

  string loc_v_bias = data_file_location + "V_bias.txt";
  Read_File.open(loc_v_bias.c_str());
  for(int j = 0; j < P; j++){
    Read_File >> V_bias[j];
  }
  Read_File.close();

  string loc_w = data_file_location + "W.txt";
  Read_File.open(loc_w.c_str());
  for(int j = 0; j < P; j++){
    for(int h = 0; h < Q; h++){
      Read_File >> W[j][h];
    }
  }
  Read_File.close();

  string loc_w_bias = data_file_location + "W_bias.txt";
  Read_File.open(loc_w_bias.c_str());
  for(int h = 0; h < Q; h++){
    Read_File >> W_bias[h];
  }
  Read_File.close();

  cout << "Load data successfully !!!" << endl;
}

