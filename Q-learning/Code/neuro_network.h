/*
 *  Author:
 *
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
//#define img_row 560.0
//#define img_col 420.0
// Normalize the output 0~1
#define L_shoulder_pitch -2.0857
#define H_shoulder_pitch 2.0857
#define L_shoulder_roll -0.3142
#define H_shoulder_roll 1.3265
// http://doc.aldebaran.com/2-1/family/robots/joints_robot.html

#define N	2	// the number of input neuros 
#define P	8	// the number of hidden neuros
#define Q	2	// the number of output neuros
#define MSE_limit 0.00002
#define Save_epoch 10000

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
  //void normalize();
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
//  double a[][characteristic_num];   // Cannot transfer if the matrix is not sure for size
//  double c[][output_num];
  
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

  data_file_location = "Hands/";    //   XOR/                         XOR/ change the Node 2 2 1 Q,  comment the normalize
  load_name ="TrainingData_raw";


  // save_file_address = "Hands";
  load_path_name = data_file_location + load_name;

  alpha = 0.5;
  beta = 0.5;


  //load_name ="XOR";
  //load_data();
  //normalize();

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
    //cout << "Delete Num:" << delimiter_element_num << endl;


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
      //myfile >> a[i][0] >> a[i][1] >> c[i][0] >> c[i][1];
      // will count one more point; start from 2
      //myfile >> w >> k >> j >> z;
      //cout << i <<" "<< w <<"  "<< k <<"  "<< j <<"  "<< z <<"  "<<endl;
      //cout << i <<" "<< line << endl;
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
        //a[iter][i] = in_out_arr[i];
        a[iter][i] = string_to_double(in_out_arr[i]);
//		std::string orbits ("365.24 29.53");
//		std::string::size_type sz; 
//		double earth = std::stod (orbits);
//		cout<<earth<< endl;

      }
      // Train Out  2 3
      for(int i=0; i<output_num; i++ ){
      	
      	//cout << in_out_arr[2] << "	" << in_out_arr[3] << endl; 
        c[iter][i] = string_to_double(in_out_arr[characteristic_num+i]);
        //cout<< characteristic_num+i << endl;
        //c[iter][i] =stod(in_out_arr[characteristic_num+i]);
      }
      iter++;
	}
//    // Load data
    for (int i= 0; i< train_data_num; i++){
    	
    cout << i <<" ";
      for(int k=0; k<characteristic_num; k++ ){
        cout << a[i][k] <<" ";
      }
      for(int k=0; k<output_num; k++ ){
        cout << c[i][k] <<" ";
      }
      cout << endl;
    	
    myfile.close();
     
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
  // Write file
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

cout << "=================== Train ================== " << endl; 
	 
	int iter_index = 0;
	double mse = 0;
	double error = 0;
	
	do{
		mse = 0;
		
		for(int j = 0; j < P; j++){       //  P Q
	 		for(int h = 0; h < Q; h++){	
				dW[j][h] = 0;
				dW_bias[h] = 0;
			}
		}
		for(int k = 0; k < N; k++){        // N P
			for(int j = 0; j < P; j++){  
				dV[k][j] = 0;
				dV_bias[j] = 0;
			}
		}	
		
		for(int i = 0; i < train_data_num; i++){
			
			//==================================== upper do not need

			for(int k = 0; k < N; k++){
				a_n[k] = a[i][k]; // a[Num][Dimen]
			}

			// Forward   Set: b_n c_n
			Feed_Forward();
			
			// Renew Det Set: d_n e_n
			for(int h = 0; h < Q ; h++){
				d_n[h] = c_n[h] * ( 1 - c_n[h] ) * ( c[i][h]- c_n[h] ); // c[Num][Dimen]
			}
			
			for(int j = 0; j < P; j++){
				double Wd_temp = 0;
				for(int h = 0; h < Q; h++){
					Wd_temp = Wd_temp + W[j][h]*d_n[h];   // w [P][Q]
				}
				e_n[j] = b_n[j] * ( 1 - b_n[j] ) * Wd_temp;
			}
			// Change Weight
			
			for(int j = 0; j < P; j++){                  // Only look the P of W[P][Q]  
				for(int h = 0; h < Q; h++){					
					// dW[j][h] += dW[j][h]             // For each dW,  thus not to sum up !!!
					dW[j][h] += alpha * b_n[j]* d_n[h];
					dW_bias[h] += alpha * d_n[h];
				}
			}
			
			for(int k = 0; k < N; k++){                  // N P
				for(int j = 0; j < P; j++){  
					dV[k][j] += beta * a_n[k] * e_n[j];
					dV_bias[j] += beta * e_n[j];
				}
			}
			
			// Calculate MSE
			error = 0;   // 1 for one 
			
			for(int h = 0; h < Q; h++){
				error += pow ( (c[i][h]- c_n[h]) ,2); 
			}
			mse += 0.5*error;	
		}
		mse = mse / double(train_data_num*Q);
		
		
		// Renew Weight and Bias
		for(int j = 0; j < P; j++){       //  P Q
 			for(int h = 0; h < Q; h++){	
			 	W[j][h] += dW[j][h] / double(train_data_num);
			 	W_bias[h] += dW_bias[h] / double(train_data_num);
			 }
		}
		for(int k = 0; k < N; k++){                  // N P
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


  }
  else cout << "Unable to open file";
//  for (int i = 0; i < train_data_num; i ++){
//    for (int j = 0; j < characteristic_num; j++){
//      cout << a[i][j] << "  ";
//    }
//    for (int k = 0; k < output_num; k ++){
//      cout << c[i][k] << "  ";
//    }
//    cout << endl;
//  }
  //Save the file
  Save_weight_bias();
}

void NN::InitNeuro(){
	
	for(int i=0; i<N ; i++){
		for(int k=0; k<P ; k++){
			// ((double) rand() / (RAND_MAX))	   0~1 	
			//cout << ((double) rand() / (RAND_MAX))*2-1 << endl;   -1~1
			V[i][k] = ((double) rand() / (RAND_MAX))*2-1;
			V_bias[k] = ((double) rand() / (RAND_MAX))*2-1;
//			dV[i][k] = 0;  // dV[N][P] 
//			dV_bias[k] = 0;
		}	
	}
	for(int i=0; i<P ; i++){
		for(int k=0; k<Q ; k++){
			W[i][k] = ((double) rand() / (RAND_MAX))*2-1;
			W_bias[k] = ((double) rand() / (RAND_MAX))*2-1;
//			dW[i][k] = 0;   
//			dW_bias[k] = 0;			
		}	
	}	
}

double NN::Sigmoid(double x){
	return 1.0/(1.0+exp(-x)); 
}

void NN::Feed_Forward(){
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
	
	double a_xy[2];
	a_xy[0] = x / img_row;
	a_xy[1] = y / img_col;
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
	
  output_joint[0] = c_xy[0] * ( H_shoulder_pitch - L_shoulder_pitch) + L_shoulder_pitch;
  output_joint[1] = c_xy[1] * ( H_shoulder_roll - L_shoulder_roll) + L_shoulder_roll;
}



void NN::Test_network(){


  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  double w,k,q,z;
  int iter = 0;

  string delimite_space =" ";
  std::string in_out_arr[characteristic_num+output_num]; // 2+2
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
	for(int k = 0; k < N; k++){                  // N P
		for(int j = 0; j < P; j++){  		
			Write_File << V[k][j] << " ";
			cout <<  V[k][j] << " ";
		}
	}
  Write_File.close();
  
  
  
  string loc_v_bias = data_file_location + "V_bias.txt";
  Write_File.open (loc_v_bias.c_str(), ios::out);
  cout << "V_bias ===========" << endl; 
	for(int j = 0; j < P; j++){  		    //  P
		Write_File << V_bias[j] << " ";
		cout <<  V_bias[j] << " ";
	}
  Write_File.close();
  
  
  
  string loc_w = data_file_location + "W.txt";
  Write_File.open (loc_w.c_str(), ios::out);
  cout << "W ===========" << endl;
  	for(int j = 0; j < P; j++){       //  P Q
 		for(int h = 0; h < Q; h++){	
			Write_File << W[j][h] << " ";
			cout <<  W[j][h] << " ";
		}
	}
  Write_File.close();
  
  string loc_w_bias = data_file_location + "W_bias.txt";
  Write_File.open (loc_w_bias.c_str(), ios::out);
  cout << "W_bias ===========" << endl;
 	for(int h = 0; h < Q; h++){	    //   Q
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
  for(int k = 0; k < N; k++){                  // N P
    for(int j = 0; j < P; j++){
      Read_File >> V[k][j];
    }
  }
  Read_File.close();


  string loc_v_bias = data_file_location + "V_bias.txt";
  Read_File.open(loc_v_bias.c_str());
  for(int j = 0; j < P; j++){  		    //  P
    Read_File >> V_bias[j];
  }
  Read_File.close();

  string loc_w = data_file_location + "W.txt";
  Read_File.open(loc_w.c_str());
  for(int j = 0; j < P; j++){       //  P Q
    for(int h = 0; h < Q; h++){
      Read_File >> W[j][h];
    }
  }
  Read_File.close();

  string loc_w_bias = data_file_location + "W_bias.txt";
  Read_File.open(loc_w_bias.c_str());
  for(int h = 0; h < Q; h++){	    //   Q
    Read_File >> W_bias[h];
  }
  Read_File.close();
  
  
  
    //===================================================== Verify 
    
//    cout << "====================== Verify " << endl; 
//    cout << "V ===========" << endl; 
//	for(int k = 0; k < N; k++){                  // N P
//		for(int j = 0; j < P; j++){  		
//			cout <<  V[k][j] << " ";
//		}
//	}
//  	cout << "V_bias ===========" << endl; 
//	for(int j = 0; j < P; j++){  		    //  P
//		cout <<  V_bias[j] << " ";
//	}
//	
//	cout << "W ===========" << endl;
//  	for(int j = 0; j < P; j++){       //  P Q
// 		for(int h = 0; h < Q; h++){	
//			cout <<  W[j][h] << " ";
//		}
//	}
//	
//	cout << "W_bias ===========" << endl;
// 	for(int h = 0; h < Q; h++){	    //   Q
//		cout <<  W_bias[h] << " ";
//	}
	
  cout << "Load data successfully !!!" << endl;
}

//void NN::normalize(){
//	
//	cout << "==================== Normalize==============" << endl; 
//	for (int i= 0; i< train_data_num; i++){
//      cout << i <<" ";
//      for(int k=0; k<characteristic_num; k++ ){ 
//	  	
//	  	if(k==0){
//	  		a[i][k] = a[i][k] / img_row;
//		}else if(k==1){
//			a[i][k] = a[i][k] / img_col;
//		}
//				
//        cout << a[i][k] <<" ";
//      }
//      for(int k=0; k<output_num; k++ ){
//      	
//	  	if(k==0){
//	  		c[i][k] = ( c[i][k] - L_shoulder_pitch ) / ( H_shoulder_pitch - L_shoulder_pitch);
//		}else if(k==1){
//			c[i][k] = ( c[i][k] - L_shoulder_roll ) / ( H_shoulder_pitch - L_shoulder_roll);
//		}
//      	
//        cout << c[i][k] <<" ";
//      }
//      cout << endl;
//    }
//}






















// Forward Trainning






// Output
