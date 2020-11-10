#include <iostream> //cout cin
#include <fstream> //handling csv file
#include <ctime> //computing execution time
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

#define print_error
#define print
#include "/home/ravipr/Desktop/skill_learning_stuff/Simulator_C++/DMPS_TYPES/MultipleDmps_v1/gnuplot_i.hpp"
using namespace std;
ofstream out("out.csv");
// Class for dynamic motion primitives
class DMP 
{
	public:
		unsigned int Dmps; // number of DMP , individual trajectory we will follow
		unsigned int Bfs; // Number of Radial basis function for non linear function
		float *Y0; // Initial state
		float *Y; // Calculated state  
		float *Dy; // Calculated velocity
		float *DDy; // Calculated acceleration
        float *G; // Initial Goal state
        float *Goal; // Goal state

		long double **Weights; // Weights for function
		//float *ay; // PD controller constants
		//float *by; // PD controller constants
		float ay;
		float by;
		float *Centers;  // Radial basis function centers
		float *Variance; // Radial basis function variance
		float *Psi; // Calculated Radial basis function value
		float force; // Non linear force to modify trajectory
		float CSx; // Canonical system value
		float CSax; // Canonical system time constant
		double dt; // Sampling Time for DMP
        double t; // Clock for the DMP system
		bool IsGoal; //Trajectory completion flag

		void MakeMatrix(float ** &temp,unsigned rows, unsigned cols)
		{
			temp = new float*[rows]; 

			for(unsigned int i = 0; i < rows; ++i)
    				temp[i] = new float[cols];
		}
		
		void MakeMatrixd(long double ** &temp,unsigned rows, unsigned cols)
		{
			temp = new long double*[rows]; 

			for(unsigned int i = 0; i < rows; ++i)
    				temp[i] = new long double[cols];
		}

		// Load weights from w.csv file
		void LoadWeights() 
		{

			string line;
            ifstream file ( "/home/ravipr/Desktop/ur_ws/src/ur5_ROS-Gazebo/w.csv" );
			char temp;
			if(file.good())
			{
				for(unsigned int ii = 0;ii<Dmps;ii++)
				{
					for(unsigned int jj = 0;jj<Bfs;jj++)
					{
						getline ( file, line, ',' );
						Weights[ii][jj] = atof(line.c_str());
						//cout<<Weights[ii][jj]<<"  "<< jj+1<<" ";
						//cin >> temp;
					}
					//cout<<endl<<endl;
				}
			}
			#ifdef print_error
			else
			{
				cout<<"Cannot find 'w.csv'"<<endl;
				exit(0);
			}
			#endif
			
			//for (unsigned int i=0; i<Dmps; i++)
			//	for(unsigned int j=0;j<Bfs;j++)
				//	Weights[i][j] = data[i][j]; 
		}

		// Initialize DMP parameters
		void InitDmpSys(const unsigned int dmps,const unsigned int bfs, float a, float b, float runtime, float tolerance, float sampling_time) 
		{
			dt = sampling_time; // Sampling time
			IsGoal = false; // Trajectory completion flag
			Dmps = dmps; // Set number of DMPs
    			Bfs = bfs; // Number of basis functions 
			Y0 = new float [Dmps]; // Set initial value
			Y = new float [Dmps]; // Set initial value
			Dy = new float [Dmps]; // Set initial value
			DDy = new float [Dmps]; // Set initial value
			Goal = new float [Dmps]; // Set goal value
            G = new float [Dmps]; // Set goal value

			Centers = new float [Bfs]; // Allocate memory for Radial Basis Function centers
			Variance = new float [Bfs]; // Allocate memory for Radial Basis Function variance
			Psi = new float [Bfs];  // Allocate memory for Radial Basis Function output
			MakeMatrixd(Weights,Dmps,Bfs); // Allocate memory for weights
			//ay = new float [Dmps]; // Allocate memory for PD controller parameters
			//by = new float [Dmps]; // Allocate memory for PD controller parameters
			ay = a;
			by = b;

//    			CSax = -std::log(tolerance)/runtime; // Time constant such that trajectory goes to 95% or (1-tolerance) in runtime
                CSax = -std::log(0.05); // Sachaal 2012 paper values to verify code
			CSx = 1.0; // Canonicalsystem initialize
			force = 0.0;

			unsigned int i, j;
			for (i=0; i<Dmps; i++)
			{
				//ay[i] = a[i]; // Set value
				//by[i] = b[i]; // Set value
				Y[i] = 0.0; // Empty for safety 
				Dy[i] = 0.0; // Empty for safety
				DDy[i] = 0.0; // Empty for safety

				for(j=0; j<Bfs; j++)
					Weights[i][j] = 0.0; // Empty weight vector for safety
			}

			GenDMPGaussCenters(runtime); // Set Radial Basis Function Centers and Varience
		}

		// Set goal and initial value
        void SetDMPConditions(float* y0, float* goal0)
		{
			for(unsigned i=0; i<Dmps; i++)
			{
				Y0[i] = y0[i];
				Y[i] = y0[i];
                G[i] = goal0[i];
                Goal[i] = goal0[i];

			}
		}

		// Set Radial Basis Function Centers and Varience
		void GenDMPGaussCenters(float runtime)
		{
			
			unsigned int i;
			float *des_c = new float [Bfs];
			des_c[0] = exp(-CSax*runtime);
			des_c[Bfs-1] = 1.05 - des_c[0];
			float tempdiff = (des_c[Bfs-1] - des_c[0]) / (Bfs-1);
						
			for(i = 1; i<Bfs; i++)
				des_c[i] = des_c[i-1] + tempdiff;
			
			for (i = 0; i<Bfs; i++)
			{
				// x = exp(-c), solving for c
				// Centers are distributed evenly in time axis
				// Variance accordint to Sachaal 2012 paper
				Centers[i] = -std::log(des_c[i]); // Set centeres
				Variance[i] = (pow(Bfs,1.5))/(Centers[i]+0.001); // Set variance
				
			}
			
		}

		// Calculate Radial Basis Function value
		float GenDMPActFunc()
		{
			float sum = 0.0;
			for(unsigned int i = 0; i<Bfs; i++)
			{
				Psi[i] = exp(-Variance[i]*pow((CSx - Centers[i]),2)); // Calculate Radial Basis Function value
				sum = sum + Psi[i];
			}
			return sum;
		}

		// Run DMP system one time step 
		bool StepDMP()
		{
			CSx = CSx - CSax*CSx*dt; // Run Canonical system one time step
			float sum = 0.0;
			sum = GenDMPActFunc(); // Calculate Radial Basis Function value
			
			float w_phi = 0.0;
			unsigned int i,j;
			for(i = 0;i<Dmps;i++)
			{
				w_phi = 0.0;
				// force = sumation(weight*RBF value)/sumation(RBF value)
				for( j=0;j<Bfs;j++)
				{
					w_phi = w_phi + Weights[i][j]*Psi[j];
					//sum = sum + Psi[j];
				}
				
				force = (w_phi*CSx*(Goal[i] - Y0[i]))/sum;
				
				// acceleration = ay*( by*(error) - deriavtive_error) + force
				// stable dynamics as force is zero when canonical system goes to zero
				DDy[i] = (ay*(by*(Goal[i] - Y[i]) - Dy[i]) + force);
				Dy[i] = Dy[i] + DDy[i]*dt;
				Y[i] = Y[i] + Dy[i]*dt;
				//Y[i] = Y0[i] + 0.5*DDy[i]*DDy[i]*dt*dt;
			}  
			//out<<endl;
			if(CSx == 0.05)
				return true;
			else
				return false;
		}

        bool StepDMP_visual_track()
        {
            CSx = CSx - CSax*CSx*dt; // Run Canonical system one time step
            t = -std::log(CSx)/CSax;

            unsigned int i,j;
            for(i = 0;i<Dmps;i++)
            {
                G[i]=Goal[i]-(Goal[i]-Y0[i])*std::exp(-t);
                // acceleration = ay*( by*(error) - deriavtive_error) + force
                // stable dynamics as force is zero when canonical system goes to zero
                DDy[i] = (ay*(by*(G[i] - Y[i]) - Dy[i]) );
                Dy[i] = Dy[i] + DDy[i]*dt;
                Y[i] = Y[i] + Dy[i]*dt;
                //Y[i] = Y0[i] + 0.5*DDy[i]*DDy[i]*dt*dt;
            }
            return true;
        }

		// Check if goal is too close to start position
		void CheckDMPGaolOffset()
		{
		 	// Check to see if initial position and goal are the same
		 	// If they are, offset slightly so that the forcing term is not 0
			for (unsigned int i = 0; i<Dmps; i++)
				if (Y0[i] == Goal[i])
					Goal[i] = Goal[i] + 0.001;
		}

		// Check if goal is too close to start position
		void CheckDMPGaol() 
		{
		 	// Check to see if initial position and goal are the same
		 	// If they are, offset slightly so that the forcing term is not 0
			IsGoal = true;
			for (unsigned int i = 0; i<Dmps; i++)
			{
				if (abs(Y[i] - Goal[i]) < 0.01)
				{
					#ifdef print
					//printf(" 1 ");
					#endif
					IsGoal &= true;
				}
				else
				{
					#ifdef print
					//printf(" 0 ");
					#endif
					IsGoal &= false;
				}
				
			}
			#ifdef print
			printf(" = %d \n", IsGoal);
			#endif
		}
			
		// Reset DMP
		void ResetDMPState()
		{
			CSx = 1.0;
			//CSax = 0.0;
			for(unsigned int i=0; i<Dmps; i++)
			{
				Y[i] = Y0[i];
				Dy[i] = 0;
				DDy[i] = 0;
			} 
		}



};

int main()
{
	DMP Nexus;

        vector<float> vy[3],vx;
        Gnuplot g1("lines");
        unsigned int time = 0;

	unsigned int number_steps = 0;
	float runtime = 9.534;
	float** y;
	float** dy;
	float** ddy;
	unsigned short int i;

	float a = 25.0; // PD values
	float b = 25.0/4; // PD values
    float y0[3]  = { -100,-250,200}; // Initial state [x0,y0,z0]
    float goal0[3] = {-100,5,5};
    float goal_lift[3]={400,5,300};

    Nexus.InitDmpSys(3, 200, a, b, runtime, 0.05, 0.002);
	Nexus.ResetDMPState();
    Nexus.SetDMPConditions(y0, goal0); // Initial conditions
	Nexus.CheckDMPGaolOffset();
//	Nexus.LoadWeights();


    while(Nexus.Goal[1]<200)
	{
        Nexus.Goal[1]= Nexus.Goal[1]+100*Nexus.dt;
        Nexus.StepDMP_visual_track();
		//out<<Nexus.CSx;
		for(i = 0; i<Nexus.Dmps; i++)
		{
			out<<Nexus.Y[i]<<",";
                        vy[i].push_back(Nexus.Y[i]);
                        vx.push_back(time);
                        time++;
			//y[i] = Nexus.Y[i];
			//cout<<Nexus.Y[i]<<" ";
			//dy[i][] = Nexus.Dy[i];
			//ddy[i] = Nexus.DDy[i];
                         out<<endl;
		}

	}
    Nexus.CSx = 1.0;
    for(i = 0; i<Nexus.Dmps; i++)
    {
      Nexus.Y[i]= Nexus.G[i];
      Nexus.Y0[i]= Nexus.G[i];
      Nexus.Goal[i]=goal_lift[i];
    }
    while(!Nexus.IsGoal)
    {
        Nexus.StepDMP_visual_track();
        //out<<Nexus.CSx;
        for(i = 0; i<Nexus.Dmps; i++)
        {
            out<<Nexus.Y[i]<<",";
                        vy[i].push_back(Nexus.Y[i]);
                        vx.push_back(time);
                        time++;
                        out<<endl;

        }

        Nexus.CheckDMPGaol();

     }

	out.close();
        for(i = 0; i<3; i++)
        {
                g1.reset_all();
                cout << endl << endl << "*** user-defined lists of doubles" << endl;
                g1.set_style("lines").plot_x(vy[i],"user-defined doubles");
                char temp;
                cin>>temp;
        }
	cout<<"DONE"<<endl;
	return 0;
}

