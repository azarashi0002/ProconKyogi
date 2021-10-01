#include<iostream>
#include<vector>
#include<queue>
#include<random>
#include<ctime>
#include<cstdlib>
#include<fstream>
#include<string>

using namespace std;
using pi = pair<int, int>;

int h, w;
int dx[4] = { -1,0,1,0 };
int dy[4] = { 0,-1,0,1 };
int selectcost,rotatecost;

struct comp {
	bool operator()(pi x, pi y)const {
		return x.first + x.second > y.first + y.second;
	}
};

pi field[16][16];
int rot[16][16];
vector<vector<pi>> index;
vector<vector<bool>> flag;



void printb() {

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			cout << boolalpha << flag[i][j] << " ";
		}
		cout << endl;
	}
}

void print() {
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			cout << field[i][j].first << " " << field[i][j].second << " ";
		}
		cout << endl;
	}
}

void printi() {
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			cout << index[i][j].first << " " << index[i][j].second << " ";
		}
		cout << endl;
	}
}

void swap(pi& x, pi& y) {
	//cout << "swaped!" << endl;
  	
	pi t = x;
	x = y; y = t;
	return;
}

void judge() {
	flag.assign(h, vector<bool>(w, false));
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			if (field[i][j].first == i and field[i][j].second == j) {
				flag[i][j] = true;
			}
		}
	}
	return;
}


int d(int sx, int sy, int ex, int ey) {
	static int cnt = 0;
	cnt++;

	if (sx == ex and sy == ey)return cnt;
	if (flag[sy][sx])return -1;

	for (int i = 0; i < h; i++) {
		int nx = sx + dx[i];
		int ny = sy + dy[i];
		if (nx >= 0 and nx < w and ny >= 0 and ny < h) {

			d(nx, ny, ex, ey);
		}
	}
	return -1;

}
void dijkstr(int sx, int sy, int ex, int ey) {
	//index.assign(h, vector<pi>(w, pi(0, 0)));
    for(int i=0;i<h;i++){
        for(int j=0;j<w;j++){
            index[i][j]=pi(0,0);
        }
    }
	//printi();
	int ox, oy;
	ox = sx, oy = sy;
	priority_queue<pi, vector<pi>, comp> que;
	que.push(pi(abs(ex - sx), abs(ey - sy)));

	while (!que.empty()) {
		pi v = que.top(); que.pop();
		int vx = v.first;
		int vy = v.second;
		//cout << vx + vy << endl;
		if (vx == 0 and vy == 0)return;

		int min = 1000000000;
		int minx, miny;
		int nx, ny;
		minx = miny = 20;
		//cout << "ny nx" << endl;
		for (int i = 0; i < 4; i++) {
			nx = ox + dx[i];
			ny = oy + dy[i];

			if (nx == -1) {
				nx += w;
			}
			else if (ny == -1) {
				ny += h;
			}
			else if (ny == h or nx == w) {
				ny %= h;
				nx %= w;
			}
			//cout << ny << " " << nx << endl;
			if (!flag[ny][nx]) {
				if (nx >= 0 and nx < w and ny >= 0 and ny < h) {
					if (min > abs(ex - nx) + abs(ey - ny)) {
						min = abs(ex - nx) + abs(ey - ny);
						que.push(pi(abs(ex - nx), abs(ey - ny)));
						//cout << ny << " " << nx << endl;

						minx = nx; miny = ny;
					}
				}


			}
		}
		if (minx == 20 or miny == 20)return;
		//cout << "seleceted" << endl;
		//cout << miny << " " << minx << endl;
		index[miny][minx] = pi(oy, ox);
		ox = minx; oy = miny;


	}
	return;
}

int main(void) {
	fstream file("PLANA.txt");
  	ofstream ans("ANS.txt");
	string str,key=" ";
	getline(file,str);
	size_t pos = 0;
	string dec;
	pos = str.find(key);
	dec = str.substr(0, pos);
	
	h = atoi(dec.c_str());
	str.erase(0, pos + 1);
	pos = str.find(key);
	dec = str.substr(0, pos);
	w = atoi(dec.c_str());
	str.erase(0, pos + 1);
	
	getline(file,str);
	pos=str.find(key);
	dec=str.substr(0,pos);
	selectcost=atoi(dec.c_str());
	str.erase(0,pos+1);
	pos=str.find(key);
	dec=str.substr(0,pos);
	rotatecost=atoi(dec.c_str());
	str.erase(0,pos+1);
	
	
	for (int i = 0; i < h; i++) {
		getline(file, str);
		for (int j = 0; j < w; j++) {
			pos = str.find(key);
			dec = str.substr(0, pos);
			field[i][j].first = atoi(dec.c_str());
			str.erase(0, pos + 1);
			pos = str.find(key);
			dec = str.substr(0, pos);
			field[i][j].second = atoi(dec.c_str());
			str.erase(0, pos + 1);
			pos = str.find(key);
			dec = str.substr(0, pos);
			rot[i][j] = atoi(dec.c_str());
			str.erase(0, pos + 1);
		}
	}


	index.resize(h, vector<pi>(w, pi(0, 0)));
	flag.resize(h, vector<bool>(w, false));

	

	string swapcode[10000];
	pi select[100000];
  	int costnum[10000]={0};
	int scnt = 0;
	for (int sy = 0; sy < h; sy++) {
		for (int sx = 0; sx < w; sx++) {
			int ox, oy;
			ox = oy = -1;
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < w; j++) {
					if (field[i][j].first == sy and field[i][j].second == sx) {
						oy = i;
						ox = j;
						break;
					}
				}
				if (ox != -1)break;
			}
			select[scnt] = pi(oy, ox);
			if (flag[oy][ox])continue;
			dijkstr(sx, sy, ox, oy);

			int oox, ooy;
			while (!(oy == sy and ox == sx)) {
				oox = ox, ooy = oy;
				auto v = index[oy][ox];
				oy = v.first;
				ox = v.second;
				if (oy - ooy == 1 or oy - ooy == -(h - 1))
					swapcode[scnt] += "D";
				else if (oy - ooy == -1 or oy - ooy == (h - 1))
					swapcode[scnt] += "U";
				else if (ox - oox == 1 or ox - oox == -(w - 1))
					swapcode[scnt] += "R";
				else if (ox - oox == -1 or ox - oox == w - 1)
					swapcode[scnt] += "L";
				swap(field[ooy][oox], field[oy][ox]);
        			costnum[scnt]++;
				swap(rot[ooy][oox], rot[oy][ox]);

			}
			if (swapcode[scnt].size() == 0) {
				flag[sy][sx] = true;
				continue;

			}
			scnt++;
			//cout << oy << " " << ox << endl;
			flag[sy][sx] = true;

		}
	}
	int sum=0;
	cout << "result" << endl;
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++)ans << rot[i][j];
	}
	ans << endl;
	ans << scnt << endl;
	for (int i = 0; i < scnt; i++) {
		ans << hex << select[i].first << select[i].second << endl;
    		ans << std::dec << costnum[i] << endl;
    		sum+=costnum[i];
		ans << swapcode[i] << endl;
	}
	printf("selectコスト:%d rotateコスト%d\n",scnt*selectcost,sum*rotatecost);
	return 0;
}
