#include<iostream>
#include<vector>
#include<queue>

using namespace std;
typedef pair<int,int> pi;

int h,w;
int dx[4]={-1,0,1,0};
int dy[4]={0,-1,0,1};

vector<vector<bool>> flag;

struct comp{
	bool operator()(pi x,pi y)const{
		return x.first+x.second>y.first+y.second;
	}
};

pi field[16][16];
pi index[16][16];
void dijkstr(int sx,int sy,int ex,int ey){
	int ox,oy;
	flag.resize(h,vector<bool>(w,false));
	ox=sx,oy=sy;
	priority_queue<pi,vector<pi>,comp> que;
	que.push(pi(abs(ex-sx),abs(ey-sy)));
	
	while(!que.empty()){
		pi v=que.top();que.pop();
		int vx=v.first;
		int vy=v.second;
		cout<<vx+vy<<endl;
		if(vx==0 && vy==0)return;
		
		int min=1000000000;
		int minx,miny;
		cout<<"nx ny"<<endl;
		for(int i=0;i<4;i++){
			int nx=ox+dx[i];
			int ny=oy+dy[i];
			
			minx=miny=20;
			cout<<nx<<" "<<ny<<endl;
			if(nx>=0 && nx<w && ny>=0 && ny<h){
				if(min>abs(ex-nx)+abs(ey-ny)){
					min=abs(ex-nx)+abs(ey-ny);
					que.push(pi(abs(ex-nx),abs(ey-ny)));
					index[ny][nx]=pi(ox,oy);
					minx=nx;miny=ny;
				}
			}
		}

		ox=minx;oy=miny;

	}
	return;
}



int main(void){
	cin>>h>>w;
	field[0][0]=pi(1,1);
	field[0][1]=pi(1,0);
	field[1][0]=pi(0,0);
	field[1][1]=pi(0,1);
	

	dijkstr(0,0,1,1);
	
	return 0;
}