#include<iostream>
#include<vector>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include "ros/ros.h"
 #include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
// #include <typeinfo>
using namespace std;

bool checkMap = false;
float stepsize= 3;
float radius= 10;
float goal_radius= 4;
int RoadMapW, RoadMapH;

float arr[2][4]= {{20, 60, 20, 50}, {50, 80, 50, 80}};
int s= sizeof(arr)/sizeof(arr[0]);


struct Point
{
	double x;
	double y;
};

struct Node
{

	float cost;
	Point P; 
	Node *parent;

};

int goalx = 35;
int goaly = 35;
int begx = 20;
int begy = 0;


Point gridSize = {RoadMapW, RoadMapH};

Point goal= {goalx, goaly};
Point beg= {begx, begy};

//int arr_1[gridx][gridy];
vector<vector<int> > obstacle_map;


//ros::init("rrt");
ros::Publisher chatter_pub;
nav_msgs::Path poses;


/*for(int i=0; i<abs(goal.x-beg.x+1); i++)
	for(int j=0; j<abs(goal.y-beg.y+1))
		arr_1[i][j]= 0;
for(int i=0; i<s; i++)
{
	for(int j=arr[i][0]; j<=arr[i][1]; j++)
		for(int k=arr[i][2]; k<=arr[i][3]; j++)
			arr_1[i][j]= 1;
}*/



// Point x_centre= {(goal.x+beg.x)/2, (goal.y+beg.y)/2};


// float c_min= sqrt((goal.x-beg.x)*(goal.x-beg.x) + (goal.y-beg.y)*(goal.y-beg.y));
// float C[2][2]= {{(goal.x-beg.x)/c_min, (goal.x-beg.x)/c_min}, {(goal.y-beg.y)/c_min, (goal.y-beg.y)/c_min}} ;


class RRT
{
public:
	Node* start = new Node;
	// start = &dd;
	vector<Node*> all_nodes;
	Node* final = new Node;
	Node sstart;
	RRT()
	{
		
		
		//cout<<"aa "<<(*start).P.x<<endl;
		//cout<<start->P.x<<endl;
		
		//cout<<sstart.P.x<<endl;
		(*start).P.x = beg.x;
		
		(*start).P.y = beg.y;
		
		start->parent= NULL;
		
		start->cost= 0;
		
		all_nodes.push_back(start);
		
		final->parent= NULL;
		
	}
	Node* generate_randnode()
	{

		// rand();
		// int asdfghjkhgfdsa = 1;
		Point gridSize = {RoadMapW, RoadMapH};
		//cout<<4444444<<endl;
		//cout<<gridSize.x<<","<<gridSize.y<<endl;
		Point p= {(rand() % int(gridSize.x)) +0, ((rand() % int(gridSize.y+0)))+0};
		bool a= true;
		//cout<<5555555<<endl;
		/*cout<<"sssssss"<<endl;
		for(int i= 0; i< all_nodes.size(); i++)
		{
			//cout<<"sssssss"<<endl;
			if(((p.x= (all_nodes[i]->P).x) && (p.y= (all_nodes[i]->P).y)))
				a= false;
				
		}*/
		if(a)
		{
			Node* rand_node= new Node;
			rand_node-> P= p;
			rand_node-> parent= NULL;
			rand_node->cost= 0;
			return rand_node;
		}
		else
			generate_randnode();
	}
		


	/*float** multiply_matrices(float a[2][2], float b[2][2])
	{
		// float** c;
		float** c= new float*[2];
		c[0]= new float[2];
		c[1]= new float[2];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}
	float* multiply_matr_vec(float a[2][2], float b[2])
	{
		float c_1[2];
		c_1[0]= a[0][0]*b[0] + a[0][1]*b[1];
		c_1[1]= a[1][0]*b[0] + a[1][1]*b[1];
		return c_1;
	}
	Node* sample(float c_best)
	{
		float r= (float) rand()/RAND_MAX;
		float theta= (float) rand()/RAND_MAX* 2*22/7;
		float s_1[2];
		s_1[0]= r*cos(theta);
		s_1[1]= r*sin(theta);
		float L[2][2]= {{0,0}, {0,0}};
		L[0][0]= c_best/2;
		L[1][1]= 0.5*sqrt(c_best*c_best - c_min*c_min);
		float** k=multiply_matrices(C, L);
		
		float* p= multiply_matr_vec(&&k, s_1);
		Point n;
		n.x= p[0]+x_centre.x;
		n.y= p[1]+x_centre.y;
		Node* rand_1;
		rand_1->P= n;
		if((obstacle_check(rand_1)) && n.x<= gridSize.x && n.y<= gridSize.y)
		{
			rand_1->parent= NULL;
			return rand_1;
		}
		else
			sample(c_best);
	}*/

	vector<Node*> near_nodes(Node* N)
	{
		//cout<<12345<<endl;
		vector<Node*> nearby_nodes;
		int e=1;
		for(int i=0; i< all_nodes.size(); i++)
		{
			if(distance(all_nodes[i]->P, N->P)< radius)
			{
				// cout<<"boom "<<all_nodes[i]->P.x<<" "<<all_nodes[i]->P.y<<endl;
				nearby_nodes.push_back(all_nodes[i]);
			}
			if(all_nodes[i]==N)
			{
				//cout<<i;
				e=i;
			}
		}
		//cout<<endl<<3<<endl;
		//nearby_nodes.erase(nearby_nodes.begin()+ e);
		//cout<<nearby_nodes.size()<<endl<<"11236432875260746597284343798247897432894890952"<<endl;
		return nearby_nodes;
	}



	// bool obstacle_check_1(Node* N)
	// {
	// 	Point p= N->P;
	// 	int a, b;
	// 	a= p.x;
	// 	b= p.y;
	// 	int bb = 1, aa = 1;
	// 	if(b>99)
	// 	{
	// 		b = 99;
	// 	}
	// 	for(int i=0; i<s; i++)
	// 	{
	// 		if((p.x>=(arr[i][0]-0)&&p.x<=(arr[i][1]+0)) && (p.y>=(arr[i][2]-0)&&p.y<=(arr[i][3]+0)))
	// 		{
	// 			bb = 0;
	// 			// return false;
	// 		}
	// 	}
	// 	if(arr_1[a][b]==1)
	// 	{
	// 		aa = 0;
	// 	}
	// 	if(aa!=bb)
	// 	{

	// 		cout<<"booooooom "<<aa<<bb<<endl;
	// 	}
	// 	if(!(a<100 and a>=0 && b>=0 && b<100))
	// 	{
	// 		cout<<"asdfghj "<<a<<" "<<b<<"\n";
	// 	}
	// 	return bb;
	// }

	// bool obstacle_check_2(Node* N)
	// {
	// 	Point p= N->P;
	// 	for(int i=0; i<s; i++)
	// 	{
	// 		if((p.x>=(arr[i][0]-1)&&p.x<=(arr[i][1]+1)) && (p.y>=(arr[i][2]-1)&&p.y<=(arr[i][3]+1)))
	// 			return false;
	// 	}
	// 	return true;
	// }

	bool obstacle_check(Node* N)
	{
		Point p= N->P;
		int a, b;
		a= p.x;
		b= p.y;
		if(obstacle_map[a][b]==1)
			return false;
		return true;
	}


	bool linejoin_check(Node* a, Node*b)
	{
		Point A= a->P;
		Point B= b->P;
		Node* N_1= new Node;
		Node* N_2= new Node;
		Node* N_3= new Node;
		Point n_1= {0.5*(A.x+B.x), 0.5*(A.y+B.y)};
		Point n_2= {0.5*(A.x+n_1.x), 0.5*(A.y+n_1.y)};
		Point n_3= {0.5*(B.x+n_1.x), 0.5*(B.y+n_1.y)};
		N_1->P= n_1;
		N_2->P= n_2;
		N_3->P= n_3;
		if(!(obstacle_check(N_1)))
			return 0;
		else if(!(obstacle_check(N_2)))
			return 0;
		else if(!(obstacle_check(N_3)))
			return 0;
		else
			return 1;
	}


	Node* nearest_node(Node* N)
	{
		Node* nearest= new Node;
		float d= 1000000;
		float m;
		//vector<Node*> all_= near_nodes(N);

		for(int i=0; i< all_nodes.size(); i++)
		{
			m= distance(all_nodes[i]->P, N->P);
			if(m<d)
			{
				d= m;
				nearest= all_nodes[i];
			}	
		}
		return nearest;
	}

	float** transpose1(float** mx)   // mx is [2][2]
	{
		float** tr=new float*[2];
 
		for(int i=0;i<2;i++)
		{
			tr[i]=new float[2];
			for(int j=0;j<2;j++)
			{
				tr[j][i]=mx[i][j];
			}	
		}

		return tr;
	}

	float** transpose2(float** mx)   // mx is [2][1]
	{
		float** tr=new float*[1];
		tr[0]=new float[2];
 
		for(int j=0;j<2;j++)
		{
			tr[0][j]=mx[j][0];	
		}

		return tr;
	}

	float** multiply_matrices1(float** a, float** b)     // for [2][2] x [2][2]
	{
		// float** c;
		float** c= new float*[2];
		c[0]= new float[2];
		c[1]= new float[2];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices2(float** a, float** b)       // for [2][2] x [2][1]
	{
		float** c=new float*[2];
		// float c[2][1];
		c[0]=new float[1];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		// c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		c[1]=new float[1];
		c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices3(float** a, float** b)    // for [1][2] x [2][1]
	{
		float** c=new float*[1];
		// float c[2][1];
		c[0]=new float[1];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		// c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		// c[1]=new float[1];
		// c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices4(float** a, float** b)    // for [1][1] x [1][1]
	{
		float** c=new float*[1];
		// float c[2][1];
		c[0]=new float[1];
		c[0][0]= a[0][0]*b[0][0];
		// c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		// c[1]=new float[1];
		// c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices2111(float** a, float** b)    // for [2][1] x [1][1]
	{
		float** c=new float*[2];
		// float c[2][1];
		c[0]=new float[1];
		c[0][0]= a[0][0]*b[0][0];
		// c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		c[1]=new float[1];
		c[1][0]= a[1][0]*b[0][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices1222(float** a, float** b)    // for [1][2] x [2][2]
	{
		float** c=new float*[1];
		// float c[2][1];
		c[0]=new float[2];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		// c[1]=new float[1];
		// c[1][0]= a[1][0]*b[0][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices2112(float** a, float** b)    // for [2][1] x [1][2]
	{
		float** c=new float*[2];
		// float c[2][1];
		c[0]=new float[2];
		c[0][0]= a[0][0]*b[0][0]; //+ a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1]; //+ a[0][1]*b[1][1];
		c[1]=new float[1];
		c[1][0]= a[1][0]*b[0][0];
		c[1][1]= a[1][0]*b[0][1]; //+ a[1][1]*b[1][1];
		return c;
	}

	float** multiply_matrices1112(float** a, float** b)    // for [1][1] x [1][2]
	{
		float** c=new float*[1];
		// float c[2][1];
		c[0]=new float[2];
		c[0][0]= a[0][0]*b[0][0]; //+ a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1]; //+ a[0][1]*b[1][1];
		// c[1]=new float[1];
		// c[1][0]= a[1][0]*b[0][0];
		// c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}





	float** inverse(float** mx)   /// we would need inverse of a 1x1 matrix
	{
		float** inv=new float*;
		inv[0]=new float[1];
		inv[0][0]=1/mx[0][0];
		return inv;
	}

	float** add_matrices(float** a,float** b)
	{
		float** res=new float*[2];

		for(int i=0;i<2;i++)
		{
			res[i]=new float[2];

			for(int j=0;j<2;j++)
				res[i][j]=a[i][j]+b[i][j];
		}	

		return res;
	}

	float** subtract_matrices(float** a,float** b)    // a-b
	{
		for(int i=0;i<2;i++)
			for(int j=0;j<2;j++)
				b[i][j]=-b[i][j];

		return add_matrices(a,b);
	}

	float max_element(float** mx)
	{
		float max=-999;
		for(int i=0;i<2;i++)
			for (int j = 0; j<2; j++)
			{
				if(mx[i][j]>max)
					max=mx[i][j];
			}

		return max;
	}

	float** abs_element(float** mx)
	{
		float** abs_mx=new float*[2];
		for(int i=0;i<2;i++)
		{
			abs_mx[i]=new float[2];

			for(int j=0;j<2;j++)
				if(mx[i][j]<0)
					abs_mx[i][j]= -1*mx[i][j];
				else
					abs_mx[i][j]=mx[i][j];
		}		

	}



	float** LQR_control(float** A,float** B,float** x)        // A is [2][2], B is [2][1] , x is [2][1] 
	{
		// float** Q={{1,0},{0,1}}, R[1][1]={{1}}, X[2][2], Xn[2][2];
		float** Q=new float*[2];
		Q[0]=new float[2];
		Q[1]=new float[2];
		Q[0][0]=1.0;Q[0][1]=0.0;
		Q[1][0]=0.0;Q[1][1]=1.0;

		float** R=new float*[1];
		R[0]=new float[1];
		R[0][0]=1.0;

		float** X=new float*[2];
		X[0]=new float[2];
		X[1]=new float[2];

		float** Xn=new float*[2];
		Xn[0]=new float[2];
		Xn[1]=new float[2];

		float** K;


		float EPS=0.01;
		
		for(int i=0;i<2;i++)               // X=Q, Xn=Q
			for(int j=0;j<2;j++)
			{	
				X[i][j]=Q[i][j];
				Xn[i][j]=Q[i][j];
			}	

		int MAX_ITER=150;
		for(int i=0;i<MAX_ITER;i++)
		{
			Xn =subtract_matrices( add_matrices( multiply_matrices1( multiply_matrices1(transpose1(A), X), A ), Q) ,
			                                     multiply_matrices2112(
			                       	                                 multiply_matrices2111( multiply_matrices2( multiply_matrices2( transpose1(A), X), B ) , inverse( add_matrices( multiply_matrices4( multiply_matrices3( transpose2(B), X), B ), R) )),
			                       	                                 multiply_matrices1222( multiply_matrices1222( transpose2(B), X ), A )	
			                        	                          )   
			                     );	                                                                                   
			                       
			                     
			if( max_element( abs_element(subtract_matrices(Xn,X))) < EPS )
				break;

			for(int i=0;i<2;i++)
				for(int j=0;j<2;j++)
					X[i][j]=Xn[i][j];          // X=Xn
		}

		K = multiply_matrices1112( inverse( add_matrices( multiply_matrices3( transpose2(B),multiply_matrices2( Xn, B)), R )), multiply_matrices1222( transpose2(B), multiply_matrices1( Xn, A ) ) );

		float** minus_K;
		float** u;
		for(int j=0;j<2;j++)
			minus_K[0][j]=-1*K[0][j];

		u = multiply_matrices3(minus_K,x);

		
		return u;
	}


	Node* new_node(Node* nearest,Node* N)
	{
		Node* new_node = new Node;
		// float s;
		// s=stepsize;
		// Point m;

		float MAX_TIME = 100.0;       // Maximum simulation time
        float DT = 0.1;                // Time tick

        float sx,sy,gx,gy;
        
		sx=(nearest->P).x;
		sy=(nearest->P).y;
		gx=(N->P).x;
		gy=(N->P).y;

		vector<float> rx;
		vector<float> ry;
		rx.push_back(sx);	
		ry.push_back(sy);	
 
		float** x=new float*[2];   // state vector
		x[0]=new float[1];
		x[1]=new float[1];
		x[0][0]=sx-gx;
		x[1][0]=sy-gy;		

		float** A=new float*[2];
		A[0]=new float[2];
		A[1]=new float[2];
		A[0][0]=DT; A[0][1]=1.0;
		A[1][0]=0.0;A[1][1]=DT;

		float** B=new float*[2];
		B[0]=new float[1];
		B[1]=new float[1];
		B[0][0]=0.0;
		B[1][0]=1.0;


		bool found_path=false;
		float time=0.0;
		float** u;

		while(time<=MAX_TIME)
		{
			time+=DT;
			u=LQR_control(A,B,x);

			x = add_matrices(multiply_matrices1(A,x),multiply_matrices2111(B,u));

			rx.push_back(x[0][0]+gx);
			ry.push_back(x[1][0]+gy);

			float d;
			d = sqrt( (gx - rx[-1])*(gx - rx[-1]) + (gx - rx[-1])*(gx - rx[-1]) );

			if(d <= goal_radius )
			{
				found_path=true;
				break;
			}

			if(!found_path)
			{
				cout<<"\n Cannot find path";
			}	

		}
 
		vector<float> px;
		vector<float> py;
		vector<float> clen;
		vector<float> dx;
		vector<float> dy;
		
		float t=0;

		for(int i=0;i<rx.size()-1;i++)
			while(t>=0.0 || t<1.0)
			{
				px.push_back( t*rx[i+1] + (1.0-t)*rx[i]);
				py.push_back( t*ry[i+1] + (1.0-t)*ry[i]);
				t+=stepsize;
			}

		for(int i=1;i<px.size();i++)
		{
			dx.push_back(px[i]-px[i-1]);
			dy.push_back(py[i]-py[i-1]);
		}

		for(int i=0;i<dx.size();i++)
			clen.push_back( sqrt(dx[i]*dx[i] + dy[i]*dy[i]) );


		if(px.size()==0)
			return NULL;

		new_node->P.x=px[px.size()-1];
		new_node->P.y=py[py.size()-1];
		
		float c=0;

		for(int i=0;i<clen.size();i++)
		{
			if(clen[i]>0)
				c+=clen[i];

			if(clen[i]<0)
				c+=-1*clen[i];
		}

		new_node->cost+=c;
		new_node->parent=nearest;

		return new_node;		

	}





	float distance(Point a, Point b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}




	Node* best_parent(Node* N)
	{
		vector<Node*> q= near_nodes(N);
		//cout<<"6666666666"<<q.size()<<endl;
		Node* best= new Node;
		best->cost = 10000000;
		float best_cost = best->cost;
		float now_cost = best->cost;
		// cout<<"innnnnnnnnnnnnnn"<<endl;
		for(int i= 0; i<q.size(); i++)
		{
			//cout<<"cost"<<" "<<q[i]->cost<<endl;
			if(!(q[i]== N))
			{
				//cout<<"shoot "<<q[i]->cost<<endl;
				//+ distance(q[i]->P, N->P)
				now_cost = (q[i]->cost + distance(q[i]->P, N->P));
				if(now_cost < best_cost)
				{
					//cout<<"in here"<<q[i]->P.x<<" "<<q[i]->P.y<<" "<<linejoin_check(q[i], N)<<endl;
					
					if((linejoin_check(q[i], N)))
					{
						best= q[i];
						// cout<<"asdfghjkl"<<endl;
						best_cost = now_cost;
						// best->cost = best_cost;
					}
				}
			}
		}
		// cout<<"fu"<<endl;
		if(!(q.size()==1))
		{
			// cout<<"in here"<<best->P.x<<" "<<best->P.y<<endl;
			// cout<<"8888888888888"<<endl;
			N->cost= best_cost;
			N->parent= best;
			return best;
		}
		else
		{
			// cout<<"1111111111111"<<endl;
			return N->parent;
		}	
	}

	void rewire(Node* N)
	{
		vector<Node*> w= near_nodes(N);


		if(!(final->parent== NULL) && (distance(final->P, N->P)<radius))
			w.push_back(final);
		//cout<<w.size();
		Node* bestparent_n= best_parent(N);


		for(int i=0; i<w.size(); i++)
		{
			if(bestparent_n== w[i])
			{
				w.erase(w.begin()+ i);
				break;
			}
			
		}


		
		for(int i=0; i<w.size(); i++)
		{
			if((N->cost+ distance(N->P, w[i]->P)) < w[i]->cost)
			{
				

				//cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
				if((linejoin_check(N, w[i])))
				{
					w[i]->parent= N;
					w[i]->cost= N->cost+ distance(N->P, w[i]->P);
				}

			}
		}

	}

	bool goal_check(Node* N, Point goal)
	{
		if(distance(N->P, goal) < goal_radius)
		{
			final->parent= N;
			final->P= goal;
			final->cost= N->cost+ distance(N->P, goal);
			return true;
		}
		return false;
	}

	vector<Node*> shortest_path(Node* start, Node* final)
	{
		vector<Node*> path;
		
		Node* a= final;
		int i=1;
			while(!(a->parent== NULL))
			{
				//cout<<i<<endl;

				path.push_back(a);
				
				//cout<<path.size()<<endl;
				if((a->parent==start))
					path.push_back(start);
				a= a->parent;
				
				i++;
				
			}
			
			
			//path.push_back(start->P);
		return path;
	}


};



vector<Node*> find_path()
{
	//cout<<3333333333333<<endl;
	RRT rrt;	
	bool c= 0;

	int iter_max= 3000;
	vector<Node*> Path_nodes;
	vector<Point> random_nodes;
	Node* rand_n= new Node;
	Node* nearest_1= new Node;
	Node* new_n= new Node;

	int check= 1;


	for(int i= 1; i<iter_max; i++)
	{
		
		//cout<<333333333564333<<endl;

		rand_n= rrt.generate_randnode();
		//cout<<3333333333333<<endl;
		random_nodes.push_back(rand_n->P);
		nearest_1= rrt.nearest_node(rand_n);
		
		new_n= rrt.new_node(nearest_1, rand_n);
		if(!(new_n==NULL))
		{
			

			c= !(rrt.final->parent== NULL);
			//cout<<c<<endl;

			if(c==1)
			{
				
				rrt.best_parent(new_n);
				rrt.rewire(new_n);
				if(check==1)
				{
					if(rrt.goal_check(new_n, goal))
					{
						check=0;
					}
				
				}
				Path_nodes= rrt.shortest_path(rrt.start, rrt.final);
				
			}
			else
			{
				rrt.best_parent(new_n);				
				rrt.rewire(new_n);
				if(check==1)
				{
					if(rrt.goal_check(new_n, goal))
					{
						check=0;
					}
				}
			}
		}


		

	}
	float q= 0;

	vector<Point> Path;

	for(int i=0; i< Path_nodes.size(); i++)
	{
		Path.push_back(Path_nodes[i]->P);
	}

	 //for(int i=0;i<Path.size();i++)
	 //{
	 	//cout<<Path[i].x<< ", "<< Path[i].y<< endl;

	 //}

	// cout<<5555555<<endl;

	// vector<Node*> all_1= rrt.all_nodes;
	 
	//  cout<<rrt.linejoin_check(rrt.start, rrt.final)<<endl<<666<<endl;;

	 

	 // for(int i=0;i<Path.size();i++)
	 // {
	 // 	cout<<Path[i].x<< ", "<< Path[i].y<< endl<<Path_nodes[i]->cost<<endl;
	 // 	if(i>0)
	 // 		cout<<rrt.distance(Path[i], Path[i-1])<<endl<<endl;

	 // }

	  cout<<endl<<rrt.final->cost<<endl;
	 
	 
	 
	 
	 int s=1;
	 return Path_nodes;

}


void publisher(vector<Point> Path, ros::Publisher chatter_pub, nav_msgs::Path poses)
{
	
	RRT a_1;
	
		  
		  	// nav_msgs::OccupancyGrid MyGrid;
		  
		  	// MyGrid.header.stamp = ros::Time::now();
		  	// MyGrid.header.frame_id = "map";
		  	// MyGrid.info.resolution = 1.0;
		  	// MyGrid.info.origin.position.x = 0.0;
		  	// MyGrid.info.origin.position.y = 0.0;
		  	// MyGrid.info.origin.position.z = 0.0;
		  	// MyGrid.info.origin.orientation.x = 0.0;
		  	// MyGrid.info.origin.orientation.y = 0.0;
		  	// MyGrid.info.origin.orientation.z = 0.0;
		  	// MyGrid.info.origin.orientation.w = 0.0;
		  	// MyGrid.info.width = RoadMapW;
		  	// MyGrid.info.height = RoadMapH;
		  	// MyGrid.info.map_load_time = ros::Time::now();
		  


		  	// bool b= true;
		  	// // int xcvb = 0;
		  	// for(int i=0;i<RoadMapW;i++)
		  	// {
		  	// 	for(int j=0;j<RoadMapH;j++)
		  	// 	{

		  	// 		Node* e = new Node;
		  	// 		//cout<<"$$$$"<<endl;
		  	// 		Point abc = {i, j};
		  	// 		e->P.x= i;
		  	// 		e->P.y= j;
		  	// 		//cout<<99999<<endl;
		  
		  	// 			if(!a_1.obstacle_check(e))
		  	// 			{
		  	// 				// xcvb++;
		  	// 				MyGrid.data.push_back(100);
		  	// 				break;
		  	// 			}
		  			
		  	// 		if(b)
		  	// 		{
		  	// 			// xcvb++;
		  	// 			MyGrid.data.push_back(0);
		  	// 		}
		  	// 	}
		  	// }



		  	// cout<<xcvb<<" asd"<<endl;
		  	 // while (ros::ok())
		  	 {
		  	 	poses.poses.clear();
		  		// cout<<count<<endl;
		  
		  		for(int i=0; i<Path.size(); i++)
		  		{
		  			geometry_msgs::PoseStamped vertex;
		  			vertex.pose.position.x= Path[i].x;
		  			vertex.pose.position.y= Path[i].y;
		  			vertex.pose.position.z= 0;
		  
		  		
		  
		  		
		  			poses.poses.push_back(vertex);
		  		}
		  
		  		chatter_pub.publish(poses);
		  		//map_pub.publish(MyGrid);
		  
		  
		      	//count++;
		  		
		  	 }  
}



void RoadCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//cout<<666666<<endl;
	if(!msg->data.empty())
	{
		checkMap = true;

		int i,j;
		RoadMapW = msg->info.width;
		RoadMapH = msg->info.height;
		begx = RoadMapW/2;
		begy = 0;
		goalx = 20;
		goaly = 20;

		std::vector<int> v;
		for (i=0; i<RoadMapW; i++) v.push_back(0);
		for (i=0; i<RoadMapH; i++) obstacle_map.push_back(v);

			//cout << "2\n";

		for(i=RoadMapH-1;i>=0;i--)//putting values
		{
			for(j=0;j<RoadMapW;j++)
			{
				// int i1;
				// i1=RoadMapH-1-i;
				if(msg->data[i*RoadMapW+j]==2)
				{
					obstacle_map[j][i] = msg->data[i*RoadMapW+j];
				}
			}
		}

		vector<Point> Path;
 	//cout<<"###"<<checkMap<<endl;

  
	if(checkMap == true )
		{

		  	//cout<<2222;
		  	vector<Node*> Path_nodes= find_path();
		  	//cout<<"%%%%%%%%%%%%%%%%%"<<endl;
		  	//cout<<555<<endl<<Path_nodes.size()<<endl;
		  	
		  	for(int i=0; i< Path_nodes.size(); i++)
		  	{
		  		Path.push_back(Path_nodes[i]->P);
		  	}
		}

		 


		  publisher(Path, chatter_pub, poses);
	

		//cout<<RoadMapW<<","<<RoadMapH<<endl<<endl;
		//cout << "3\n";
	}
}






int main(int argc, char **argv)
{
	/*for(int i=0; i<=gridx; i++)
		for(int j=0; j<=gridy;j++)
			arr_1[i][j]= 0;
	for(int i=0; i<s; i++)
	{
		for(int j=arr[i][0]-1; j<arr[i][1]+1; j++)
		{
			for(int k=arr[i][2]-1; k<arr[i][3]+1; k++)
			{
				if(j>=0 and j<100 and k>=0 and k<100)
					arr_1[j][k]= 1;
			}
		}
	}*/

	RRT a_1;

	//cout<<checkMap<<endl;
	ros::init(argc, argv, "rrt");

	ros::NodeHandle LRSubNode;
	ros::Subscriber SubRoad = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("occ_map", 1, RoadCallback);

	
 //  	vector<Point> Path;
 // 	//cout<<"###"<<checkMap<<endl;
  
	// if(checkMap == true )
	// 	{
	// 	  	//cout<<2222;
	// 	  	vector<Node*> Path_nodes= find_path();
	// 	  	//cout<<555<<endl<<Path_nodes.size()<<endl;
		  	
	// 	  	for(int i=0; i< Path_nodes.size(); i++)
	// 	  	{
	// 	  		Path.push_back(Path_nodes[i]->P);
	// 	  	}
	// 	  }
	// 	  		int count= 0;
		  
		  	 
		ros::NodeHandle v;
		  	 //ros::Publisher chatter_pub= v.advertise< nav_msgs/Path >("PATH", 1000);
		  	 chatter_pub = v.advertise<nav_msgs::Path>("Path", 1);
		  	 ros::Publisher map_pub = v.advertise<nav_msgs::OccupancyGrid>("MyGrid", 1);
		  	 poses.header.frame_id = "map";
		  
		  	

		  
	 	  	 //ros::NodeHandle v;
	// 	  	 //ros::Publisher chatter_pub= v.advertise< nav_msgs/Path >("PATH", 1000);
	// 	  	 ros::Publisher chatter_pub = v.advertise<nav_msgs::Path>("PATH", 1);
	// 	  	 ros::Publisher map_pub = v.advertise<nav_msgs::OccupancyGrid>("MyGrid", 1);
		  
	// 	  	nav_msgs::Path poses;
	// 	  	poses.header.frame_id = "map";
		  
	// 	  	nav_msgs::OccupancyGrid MyGrid;
		  
	// 	  	MyGrid.header.stamp = ros::Time::now();
	// 	  	MyGrid.header.frame_id = "map";
	// 	  	MyGrid.info.resolution = 1.0;
	// 	  	MyGrid.info.origin.position.x = 0.0;
	// 	  	MyGrid.info.origin.position.y = 0.0;
	// 	  	MyGrid.info.origin.position.z = 0.0;
	// 	  	MyGrid.info.origin.orientation.x = 0.0;
	// 	  	MyGrid.info.origin.orientation.y = 0.0;
	// 	  	MyGrid.info.origin.orientation.z = 0.0;
	// 	  	MyGrid.info.origin.orientation.w = 0.0;
	// 	  	MyGrid.info.width = RoadMapW;
	// 	  	MyGrid.info.height = RoadMapH;
	// 	  	MyGrid.info.map_load_time = ros::Time::now();
		  
	// 	  	bool b= true;
	// 	  	// int xcvb = 0;
	// 	  	for(int i=0;i<RoadMapW;i++)
	// 	  	{
	// 	  		for(int j=0;j<RoadMapH;j++)
	// 	  		{
	// 	  			Node* e;
	// 	  			e->P.x= i;
	// 	  			e->P.y= j;
		  
	// 	  				if(!a_1.obstacle_check(e))
	// 	  				{
	// 	  					// xcvb++;
	// 	  					MyGrid.data.push_back(100);
	// 	  					break;
	// 	  				}
		  			
	// 	  			if(b)
	// 	  			{
	// 	  				// xcvb++;
	// 	  				MyGrid.data.push_back(0);
	// 	  			}
	// 	  		}
	// 	  	}
	// 	  	// cout<<xcvb<<" asd"<<endl;
	// 	  	 // while (ros::ok())
	// 	  	 {
	// 	  	 	poses.poses.clear();
	// 	  		 cout<<count<<endl;
		  
	// 	  		for(int i=0; i<Path.size(); i++)
	// 	  		{
	// 	  			geometry_msgs::PoseStamped vertex;
	// 	  			vertex.pose.position.x= Path[i].x;
	// 	  			vertex.pose.position.y= Path[i].y;
	// 	  			vertex.pose.position.z= 0;
		  
		  		
		  
		  		
	// 	  			poses.poses.push_back(vertex);
	// 	  		}
		  
	// 	  		chatter_pub.publish(poses);
	// 	  		map_pub.publish(MyGrid);
		  
		  
	// 	      	count++;
		  		
	// 	  	 }  
		  	 
				ros::Rate loop_rate(10);
				ros::spin();
		  
				loop_rate.sleep();

	return 0;
}
